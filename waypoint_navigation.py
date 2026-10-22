import math
import time
import asyncio
import csv
from mavsdk_grpc import System
from mavsdk_grpc.action import OrbitYawBehavior

# waypoints - Name, Meters north of start, Meters east of start
waypoints = [
    ("ORIGIN", 0, 0),
    ("A", 5, 0),
    ("B", 12, 0),
    ("C", 2, 13),
    ("D", 40, 10)
]

ALT = 10                   # relative altitude in meters
EARTH_RADIUS = 6378137.0  # earth's radius in meters


# turns a starting coordinate and number of meters north and east into a second set of coordinates
def meters_to_coords(lat, lon, meters_north, meters_east):
    lat_rad = math.radians(lat)
    dLat = meters_north / EARTH_RADIUS
    dLon = meters_east / (EARTH_RADIUS * math.cos(lat_rad))
    return lat + math.degrees(dLat), lon + math.degrees(dLon)



# turns two sets of coordinates into the distance in meters
def coords_to_meters(lat1, lat2, lon1, lon2, alt1, alt2):
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    lon1_rad = math.radians(lon1)
    lon2_rad = math.radians(lon2)

    # haversine formula to calculate distance horizontally
    dLat = lat2_rad - lat1_rad
    dLon = lon2_rad - lon1_rad

    a = (math.sin(dLat / 2) ** 2
         + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dLon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # distance horizontally
    horizontal_distance = EARTH_RADIUS * c

    # distance vertically
    vertical_distance = alt2 - alt1

    return math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

    lat_rad = math.radians(start_lat)
    meters_north = math.radians(lat - start_lat) * EARTH_RADIUS
    meters_east = math.radians(lon - start_lon) * \
        EARTH_RADIUS * math.cos(lat_rad)
    return meters_north, meters_east



# find the center of the circle the drone orbits around (between 2 waypoints)
def find_orbit_center_cartesian(waypoint1, waypoint2):
    y1, x1 = waypoint1[1], waypoint1[2]
    y2, x2 = waypoint2[1], waypoint2[2]

    center_vert_pos = (y1 + y2) / 2
    center_horiz_pos = (x1 + x2) / 2

    return center_vert_pos, center_horiz_pos



# wait until the drone is within a certain distance of the waypoint before continuing
async def wait_until_close(drone, newCoords, ABS_ALT, distance_threshold):
    while True:
        # get the drone's current location
        async for pos in drone.telemetry.position():
            # calculate the distance from the drone's current position to the waypoint
            distance = coords_to_meters(
                pos.latitude_deg, newCoords[0], pos.longitude_deg, newCoords[1], pos.absolute_altitude_m, ABS_ALT)
            break
        # if the drone is within the specified threshold of the waypoint, print the current position in meters from start and break the loop
        if distance < distance_threshold:
            print("Waypoint reached!")
            async for local_pos in drone.telemetry.position_velocity_ned():
                print(
                    "Current Axis Coordinates:",
                    local_pos.position.north_m,
                    local_pos.position.east_m,
                    local_pos.position.down_m
                )
                break
            break
    return



# find the distance between two waypoints in meters
def distance_between_waypoints(waypoint1, waypoint2):
    y1, x1 = waypoint1[1], waypoint1[2]
    y2, x2 = waypoint2[1], waypoint2[2]

    return math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)



# log the drone's postion in meters from the starting point to a CSV file
async def log_position(drone, filename="flight_log.csv"):
    start = time.time()
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "north_m", "east_m", "alt_m"])
        async for pv in drone.telemetry.position_velocity_ned():
            p = pv.position
            writer.writerow([round(time.time() - start, 2),
                            p.north_m, p.east_m, -p.down_m])
            f.flush()  # so data isn't lost if the script crashes



# log the drone's motor outputs to a CSV file
async def log_motors(drone, start_time, filename="motor_log.csv"):
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "motor1", "motor2", "motor3", "motor4"])
        async for status in drone.telemetry.actuator_output_status():
            outputs = list(status.actuator[:4])
            writer.writerow([round(time.time() - start_time, 2), *outputs])
            f.flush()


async def main():
    drone = System()

    try:
        print("Connecting to drone")
        await drone.connect(system_address="udpin://0.0.0.0:14540")

        # if connection state changes, print connected and break loop
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("drone connected")
                break
            await asyncio.sleep(1)

        print("Waiting for position checks")

        # Do global and home position checks
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("Position checks passed!")
                break

        # set up logging for the CSV file
        # 10 Hz logging
        await drone.telemetry.set_rate_position_velocity_ned(10)
        log_task = asyncio.create_task(log_position(drone))
        await drone.telemetry.set_rate_actuator_output_status(10)
        motor_task = asyncio.create_task(log_motors(drone, time.time()))

        await drone.action.set_takeoff_altitude(ALT)

        print("Arming...")
        await drone.action.arm()

        print(f"Taking off to {ALT} meters...")
        await drone.action.takeoff()

        # wait for drone to reach takeoff height, then break out of loop and continue with waypoint navigation
        async for pos in drone.telemetry.position():
            if pos.relative_altitude_m >= ALT * 0.95:
                print(f"Reached target altitude of {ALT} meters")
                break

        print("Takeoff command sent.")

        # store the drone's starting coordinates for use in waypoint navigation (lattitude, longitude, absolute altitude)
        startCoords = []
        async for pos in drone.telemetry.position():
            startCoords = [pos.latitude_deg,
                           pos.longitude_deg, pos.absolute_altitude_m]
            break
        # store the absolute altitude for use in waypoint navigation
        ABS_ALT = startCoords[2]
        # loop through the waypoints and fly to each one, printing the current position in meters after reaching each waypoint
        for waypoint in waypoints:
            name, north_m, east_m = waypoint
            # calculate the new coordinates based on the starting coordinates and the waypoint offsets
            newCoords = meters_to_coords(
                startCoords[0], startCoords[1], north_m, east_m)

            # fly to the new coordinates
            print(f"Flying to waypoint {name}")
            await drone.action.goto_location(newCoords[0], newCoords[1], ABS_ALT, 0)

            # calculate when to stop based on the drone's current position and the distance to the waypoint
            await wait_until_close(drone, newCoords, ABS_ALT, distance_threshold=0.5)

        # find the coordinates of the center of the circle between the two waypoints
        center_vert_pos, center_horiz_pos = find_orbit_center_cartesian(
            waypoints[0], waypoints[-1])
        center_lat, center_lon = meters_to_coords(
            startCoords[0], startCoords[1], center_vert_pos, center_horiz_pos)

        radius = distance_between_waypoints(waypoints[0], waypoints[-1]) / 2
        print(
            f"Orbiting around center at ({center_lat}, {center_lon}) with radius {radius} meters")
        await drone.action.do_orbit(radius, 10.0, OrbitYawBehavior.HOLD_INITIAL_HEADING, center_lat, center_lon, ABS_ALT)
        await wait_until_close(drone, (startCoords[0], startCoords[1]), ABS_ALT, distance_threshold=1)
        await drone.action.hold()
        await asyncio.sleep(1)

    except Exception as e:
        print(f"Drone operation failed: {e}")

    # land the drone in the original launch position
    await drone.action.return_to_launch()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Drone has landed.")
            break

    # end logging for the drone's CSV file
    log_task.cancel()
    motor_task.cancel()
    return


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Execution stopped by user")
