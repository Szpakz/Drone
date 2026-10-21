import math, time, asyncio
from mavsdk_grpc import System
from mavsdk_grpc.action import OrbitYawBehavior

# waypoints - Name, Meters north of start, Meters east of start
waypoints = [
    ("A", 0, 0),
    ("B", 10, 0),
    ("C", 10, 10),
    ("D", 0, 10)
]

ALT = 10.0
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

        await drone.action.set_takeoff_altitude(ALT)

        print("Arming...")
        await drone.action.arm()

        print(f"Taking off to {ALT} meters...")
        await drone.action.takeoff()

        await asyncio.sleep(12)  # wait for the drone to reach takeoff altitude

        print("Takeoff command sent.")

        # Reference point for ALL waypoints: latitude, longitude, absolute altitude
        startCoords = []

        async for pos in drone.telemetry.position():
            startCoords = [
                pos.latitude_deg,
                pos.longitude_deg,
                pos.absolute_altitude_m
            ]
            break

        for waypoint in waypoints:
            name, north_m, east_m = waypoint

            # FIX: offsets are measured from the START point, not the drone's current position
            newCoords = meters_to_coords(
                startCoords[0],
                startCoords[1],
                north_m,
                east_m
            )

            print(f"Flying to waypoint {name}")

            await drone.action.goto_location(
                newCoords[0],
                newCoords[1],
                startCoords[2],  # hold the altitude captured after takeoff
                0
            )

            while True:
                async for pos in drone.telemetry.position():
                    distance = coords_to_meters(
                        pos.latitude_deg,
                        newCoords[0],
                        pos.longitude_deg,
                        newCoords[1],
                        pos.absolute_altitude_m,
                        startCoords[2]
                    )
                    break

                if distance < 1.0:  # within 1 meter
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

                await asyncio.sleep(0.5)


        center_lat, center_lon = meters_to_coords(startCoords[0], startCoords[1], 0, 5)

        await drone.action.do_orbit(
            5.0,                                          # radius_m
            2.0,                                          # velocity_ms
            OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER, # yaw_behavior
            center_lat,
            center_lon,
            startCoords[2]                                # absolute_altitude_m
        )

        await asyncio.sleep(math.pi * 5.0 / 2.0)  # half lap, about 7.9 s

        await drone.action.goto_location(startCoords[0], startCoords[1], startCoords[2], 0)
        await asyncio.sleep(3)
        await asyncio.sleep(3)

    except Exception as e:
        print(f"Drone operation failed: {e}")

    await drone.action.land()

    return


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Execution stopped by user")