import math
import time
import asyncio
import numpy as np
from mavsdk_grpc import System
from mavsdk_grpc.offboard import PositionNedYaw, VelocityNedYaw, OffboardError

# Waypoints: name, meters north of start, meters east of start
waypoints = [
    ("ORIGIN", 0, 0),
    ("A", 5, 0),
    ("B", 12, 0),
    ("C", 2, 13),
    ("D", 40, 10)
]

ALT = 2.0                # meters, positive up
CONNECT_ADDRESS = "udpin://0.0.0.0:14540"  # change for your hardware / router
ARC_SPEED = 1.5          # average speed along the return arc, m/s
ARC_CLOCKWISE = True     # which side the return arc bulges toward
YAW = 0.0                # heading in degrees


async def get_position_velocity(drone):
    """Return one position/velocity sample in the local NED frame."""
    async for sample in drone.telemetry.position_velocity_ned():
        return sample


def to_array(position):
    """Turn a MAVSDK position into a numpy array [north, east, down]."""
    return np.array([position.north_m, position.east_m, position.down_m])


async def goto_local(drone, origin, north, east, alt, speed=3.0, threshold=0.2):
    """
    Fly to a point (north, east meters from origin, alt meters up).

    A setpoint moves toward the target at `speed` and the drone follows it.
    Returns once the drone is within `threshold` meters of the target.
    """
    target = np.array([origin[0] + north, origin[1] + east, -alt])

    # the setpoint starts at the drone's current position
    setpoint = to_array((await get_position_velocity(drone)).position)

    last_time = time.monotonic()
    for num in range(1000):
        # how far the setpoint may move this pass: speed * time since last pass
        now = time.monotonic()
        step_size = speed * (now - last_time)
        last_time = now

        # direction and distance from the SETPOINT to the target
        delta = target - setpoint
        distance = np.linalg.norm(delta)

        # move the setpoint toward the target (or snap onto it if close)
        if distance > step_size:
            setpoint = setpoint + delta / distance * step_size
        else:
            setpoint = target.copy()

        await drone.offboard.set_position_ned(
            PositionNedYaw(*setpoint.tolist(), YAW))

        # check how close the drone actually is
        position = to_array((await get_position_velocity(drone)).position)
        error = np.linalg.norm(position - target)
        if error < threshold:
            print(f"  arrived, error {error:.2f} m")
            return

    print("  goto_local gave up before reaching the target")


async def fly_arc_to_origin(drone, origin, start_north, start_east, alt,
                            speed=2.0, clockwise=True):
    """
    Fly a semicircle from (start_north, start_east) back to the origin.

    The circle's diameter is the straight line between the two points, so
    the arc ends exactly at the origin. The drone eases in and out, so it
    starts and finishes the arc at rest.
    """
    center_north_rel = start_north / 2
    center_east_rel = start_east / 2
    radius = math.hypot(start_north, start_east) / 2
    if radius < 0.1:
        return  # already at the origin

    start_angle = math.atan2(start_east - center_east_rel,
                             start_north - center_north_rel)
    total_sweep = math.pi if clockwise else -math.pi
    duration = math.pi * radius / speed
    center_north = origin[0] + center_north_rel
    center_east = origin[1] + center_east_rel

    start_time = time.monotonic()
    while True:
        progress = min((time.monotonic() - start_time) / duration, 1.0)
        eased = 3 * progress ** 2 - 2 * progress ** 3
        eased_rate = (6 * progress - 6 * progress ** 2) / duration
        angle = start_angle + total_sweep * eased
        angular_rate = total_sweep * eased_rate

        await drone.offboard.set_position_velocity_ned(
            PositionNedYaw(center_north + radius * math.cos(angle),
                           center_east + radius * math.sin(angle),
                           -alt, YAW),
            VelocityNedYaw(-radius * angular_rate * math.sin(angle),
                           radius * angular_rate * math.cos(angle),
                           0.0, YAW))

        if progress >= 1.0:
            break
        await asyncio.sleep(0.05)


async def main():
    drone = System()
    print("Connecting...")
    await drone.connect(system_address=CONNECT_ADDRESS)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    print("Waiting for local position estimate...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("Local position OK")
            break

    await drone.telemetry.set_rate_position_velocity_ned(10)

    # capture the origin BEFORE takeoff so it matches the launch point
    position = (await get_position_velocity(drone)).position
    origin = (position.north_m, position.east_m)

    armed = False
    offboard_started = False
    try:
        await drone.action.set_takeoff_altitude(ALT)
        print("Arming...")
        await drone.action.arm()
        armed = True

        print(f"Taking off to {ALT} m...")
        await drone.action.takeoff()

        # wait until close to takeoff altitude (with a 30s timeout)
        takeoff_start = time.monotonic()
        while time.monotonic() - takeoff_start < 30:
            position = (await get_position_velocity(drone)).position
            if -position.down_m >= ALT * 0.95:
                break
        await asyncio.sleep(1)  # let it settle

        # PX4 needs a setpoint before offboard can start: hold current position
        position = (await get_position_velocity(drone)).position
        await drone.offboard.set_position_ned(
            PositionNedYaw(position.north_m, position.east_m,
                           position.down_m, YAW))
        try:
            await drone.offboard.start()
            offboard_started = True
        except OffboardError as error:
            print(f"Offboard start failed: {error._result.result}")
            return

        # visit every waypoint
        for name, waypoint_north, waypoint_east in waypoints:
            print(f"Flying to waypoint {name}")
            await goto_local(drone, origin, waypoint_north, waypoint_east, ALT)

        # return to the origin along a semicircular arc
        last_name, last_north, last_east = waypoints[-1]
        print("Returning to origin along an arc")
        await fly_arc_to_origin(drone, origin, last_north, last_east, ALT,
                                speed=ARC_SPEED, clockwise=ARC_CLOCKWISE)

        # tighten up over the origin before landing
        print("Settling over origin")
        await goto_local(drone, origin, 0.0, 0.0, ALT, speed=0.5, threshold=0.1)

    except Exception as error:
        print(f"Drone operation failed: {error}")

    #run this no matter what happens (including errors) to ensure the drone lands and offboard is stopped
    finally:
        if offboard_started:
            try:
                await drone.offboard.stop()
            except OffboardError as error:
                print(f"Offboard stop failed: {error._result.result}")
        if armed:
            print("Landing...")
            await drone.action.land()
            async for in_air in drone.telemetry.in_air():
                if not in_air:
                    print("Drone has landed.")
                    break
            position = (await get_position_velocity(drone)).position
            miss = math.hypot(position.north_m - origin[0],
                              position.east_m - origin[1])
            print(f"Landed {miss:.2f} m from origin")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Execution stopped by user")