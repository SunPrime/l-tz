from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import math
import argparse

# --- Constants ---
TARGET_ALTITUDE = 100  # meters
TARGET_YAW = 350  # degrees
DEFAULT_CONNECTION_STRING = 'tcp:127.0.0.1:5762'

PITCH_PWM_FORWARD = 1600
ROLL_PWM_RATE = 100
YAW_PWM_RATE = 100
THROTTLE_PWM_NEUTRAL = 1500
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000
YAW_SPEED = 10

DISTANCE_TOLERANCE = 5  # meters
INITIAL_YAW_ALIGN_TOLERANCE = 10  # degrees
YAW_TOLERANCE = 1  # degrees
ALIGN_TIMEOUT = 60  # seconds
FLIGHT_TIMEOUT = 360  # seconds
YAW_TIMEOUT = 60  # seconds

# Point B (Target)
target_location = LocationGlobalRelative(50.443326, 30.448078, 100)

# --- Functions ---
def connect_vehicle(connection_string):
    """Connects to the vehicle and handles potential connection errors."""
    print(f"Connecting to vehicle on: {connection_string}")
    try:
        vehicle = connect(connection_string, wait_ready=True, timeout=60, heartbeat_timeout=30)
        print("Vehicle connected!")
        return vehicle
    except APIException as e:
        print(f"API connection error: {e}")
        return None
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None

def arm_and_takeoff(vehicle, target_altitude):
    """Arms the vehicle and performs takeoff to the specified altitude in GUIDED mode."""
    print("Initial pre-flight check...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)

    print("Setting GUIDED mode")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print(f" Waiting for GUIDED mode confirmation (current: {vehicle.mode.name})...")
        time.sleep(1)

    print("Arming motors")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Vehicle armed!")

    print(f"Taking off to altitude {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {current_altitude:.2f} m")
        if current_altitude >= target_altitude * 0.95:
            print("Target altitude reached!")
            break
        time.sleep(1)

def get_distance_metres(location1, location2):
    """Returns the distance in meters between two LocationGlobal points (simplified formula)."""
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return math.sqrt((dlat**2) + (dlong**2)) * 1.113195e5

def get_bearing(location1, location2):
    """Returns the bearing from point 1 to point 2 in degrees."""
    off_x = location2.lon - location1.lon
    off_y = location2.lat - location1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    return bearing % 360  # Normalize to 0-360

def set_channel_override(vehicle, channel, pwm_value):
    """Sets the PWM value for the specified channel."""
    overrides = vehicle.channels.overrides
    if pwm_value is None and str(channel) in overrides:
        del overrides[str(channel)]
    elif pwm_value is not None:
        overrides[str(channel)] = int(pwm_value)
    vehicle.channels.overrides = overrides # Update the vehicle's override dictionary

def condition_yaw(vehicle, heading, relative=False, yaw_speed=10, is_clockwise=1):
    """Sends MAV_CMD_CONDITION_YAW command to vehicle."""
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
        heading, yaw_speed, is_clockwise, is_relative, 0, 0, 0)
    vehicle.send_mavlink(msg)

def normalize_angle(angle):
    """Normalizes an angle to the range -180 to +180 degrees."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def calculate_roll_pwm(bearing_diff, max_rate):
    """Calculates the roll PWM offset based on the bearing difference."""
    roll_pwm_offset = max(-max_rate, min(max_rate, int(bearing_diff * (max_rate / 90.0))))
    return PWM_NEUTRAL + roll_pwm_offset

# --- Main Script ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Controls the drone for flight from A to B in ALT_HOLD mode.')
    parser.add_argument('--connect', default=DEFAULT_CONNECTION_STRING,
                        help="Vehicle connection string (default: %(default)s)")
    args = parser.parse_args()

    vehicle = connect_vehicle(args.connect)

    if vehicle is None:
        print("Failed to connect to vehicle. Exiting.")
        exit()

    try:
        # --- Takeoff ---
        arm_and_takeoff(vehicle, TARGET_ALTITUDE)
        print("Takeoff complete.")

        # --- Switch to ALT_HOLD ---
        print("Switching to ALT_HOLD mode...")
        vehicle.mode = VehicleMode("ALT_HOLD")
        start_time = time.time()
        while vehicle.mode.name != "ALT_HOLD":
            print(f" Waiting for ALT_HOLD mode confirmation (current: {vehicle.mode.name})...")
            if time.time() - start_time > 10:
                raise TimeoutError("Failed to switch to ALT_HOLD")
            time.sleep(0.5)
        print("ALT_HOLD mode set.")

        # --- Align Course to Point B ---
        print(f"Adjusting course to point B ({target_location.lat}, {target_location.lon})...")
        align_start_time = time.time()
        while True:
            if time.time() - align_start_time > ALIGN_TIMEOUT:
                print("!!! Yaw alignment timeout !!!")
                break

            current_location = vehicle.location.global_relative_frame
            current_heading = vehicle.heading
            bearing_to_target = get_bearing(current_location, target_location)
            bearing_diff = normalize_angle(bearing_to_target - current_heading)

            print(f" Alignment: Heading={current_heading:.1f}°, Target={bearing_to_target:.1f}°, Difference={bearing_diff:.1f}°")

            if abs(bearing_diff) <= INITIAL_YAW_ALIGN_TOLERANCE:
                print("Course aligned")
                vehicle.channels.overrides = {}
                time.sleep(0.5)
                break

            roll_pwm = calculate_roll_pwm(bearing_diff, 120) # Using a slightly higher rate for alignment
            overrides = {
                '1': roll_pwm,
                '2': PWM_NEUTRAL,
                '3': THROTTLE_PWM_NEUTRAL,
                '4': PWM_NEUTRAL
            }
            vehicle.channels.overrides = overrides
            time.sleep(0.1)

        # --- Flight to Point B in ALT_HOLD ---
        print(f"Starting flight to point B: {target_location.lat}, {target_location.lon} at altitude {TARGET_ALTITUDE} m")
        flight_start_time = time.time()
        while True:
            if time.time() - flight_start_time > FLIGHT_TIMEOUT:
                print("Flight timeout to point B!")
                break

            current_location = vehicle.location.global_relative_frame
            distance_to_target = get_distance_metres(current_location, target_location)
            bearing_to_target = get_bearing(current_location, target_location)
            current_heading = vehicle.heading

            log_extra = ""
            applied_roll = vehicle.channels.overrides.get('1', None)
            if applied_roll:
                log_extra += f" RollPWM={applied_roll}"

            if distance_to_target < 100:
                bearing_diff = normalize_angle(bearing_to_target - current_heading)
                print(f"CLOSE ({distance_to_target:.1f}m): "
                      f"TgtBear={bearing_to_target:.1f}, CurHead={current_heading:.1f}, "
                      f"BearDiff={bearing_diff:.1f}, GS={vehicle.groundspeed:.2f}{log_extra}")
            else:
                print(f"Dist: {distance_to_target:.1f}m, Bear: {bearing_to_target:.1f}°, Head: {current_heading}°, GS: {vehicle.groundspeed:.2f} m/s")

            if distance_to_target <= DISTANCE_TOLERANCE:
                print("Point B reached!")
                vehicle.channels.overrides = {}
                print("Movement overrides reset.")
                break

            # --- Calculate Control Inputs ---
            yaw_pwm = PWM_NEUTRAL
            throttle_pwm = THROTTLE_PWM_NEUTRAL
            pitch_pwm = PITCH_PWM_FORWARD
            bearing_diff = normalize_angle(bearing_to_target - current_heading)
            roll_pwm = calculate_roll_pwm(bearing_diff, ROLL_PWM_RATE)

            # Limit PWM values (already done in calculate_roll_pwm, but good to be explicit)
            pitch_pwm = max(PWM_MIN, min(PWM_MAX, pitch_pwm))
            roll_pwm = max(PWM_MIN, min(PWM_MAX, roll_pwm))
            throttle_pwm = max(PWM_MIN, min(PWM_MAX, throttle_pwm))
            yaw_pwm = max(PWM_MIN, min(PWM_MAX, yaw_pwm))

            overrides = {
                '1': roll_pwm,
                '2': pitch_pwm,
                '3': throttle_pwm,
                '4': yaw_pwm
            }
            vehicle.channels.overrides = overrides
            time.sleep(0.1)

        # --- Turning to Target Yaw ---
        print(f"Reached Point B. Turning to yaw {TARGET_YAW} degrees...")
        yaw_start_time = time.time()
        vehicle.mode = VehicleMode("GUIDED") # Switch to GUIDED for yaw control
        while True:
            if time.time() - yaw_start_time > YAW_TIMEOUT:
                print("Yaw turn timeout!")
                break

            current_heading = vehicle.heading
            heading_diff = normalize_angle(TARGET_YAW - current_heading)

            print(f" Current Heading: {current_heading:.1f} deg, Target: {TARGET_YAW} deg, Error: {heading_diff:.1f} deg")

            if abs(heading_diff) <= YAW_TOLERANCE:
                print("Target yaw reached!")
                vehicle.channels.overrides = {}
                print("Yaw overrides reset.")
                break
            else:
                # Determine the direction to turn
                direction = 1 if heading_diff > 0 else -1  # Clockwise or counter-clockwise

                # Send the condition_yaw command
                condition_yaw(vehicle, TARGET_YAW, relative=False, yaw_speed=YAW_SPEED * abs(heading_diff) / 30.0, is_clockwise=direction)

                time.sleep(0.1)

        # --- Completion ---
        print("Script finished mission execution.")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(5)

    except TimeoutError as e:
        print(f"Timeout error: {e}")
        print("Attempting to reset overrides...")
        if 'vehicle' in locals() and vehicle.armed:
            vehicle.channels.overrides = {}
    except Exception as e:
        print(f"An error occurred: {e}")
        print("Attempting to reset overrides...")
        if 'vehicle' in locals() and vehicle.armed:
            vehicle.channels.overrides = {}
    finally:
        if 'vehicle' in locals():
            if vehicle.channels.overrides:
                print("Final reset of overrides before closing...")
                vehicle.channels.overrides = {}
                time.sleep(1)
            if vehicle.connected:
                print("Closing connection to the vehicle...")
                vehicle.close()
                print("Connection closed.")