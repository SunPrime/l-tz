from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import math
import argparse # For processing command line arguments (connection address)

TARGET_ALTITUDE = 100  # m
TARGET_YAW = 350     # degrees
CONNECTION_STRING = 'tcp:127.0.0.1:5762' # Use a free SITL port

PITCH_PWM_FORWARD = 1600 # PWM for forward movement (greater than 1500)
ROLL_PWM_RATE = 100      # Coefficient for turning (PWM deviation from 1500)
YAW_PWM_RATE = 100       # Coefficient for yaw turning
THROTTLE_PWM_NEUTRAL = 1500 # Neutral throttle for altitude hold in ALT_HOLD
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000
YAW_SPEED = 10

# --- Functions ---
def connect_vehicle(connection_string):
    """
    Connects to the vehicle using the specified connection string.
    """
    print(f"Connecting to vehicle on: {connection_string}")
    try:
        # Connect to the vehicle (SITL or real)
        # Wait for heartbeat for 30 seconds
        vehicle = connect(connection_string, wait_ready=True, timeout=60, heartbeat_timeout=30)
        print("Vehicle connected!")
        # vehicle.wait_ready('autopilot_version') # Wait to receive autopilot version
        # print(f"Autopilot version: {vehicle.version}")
        return vehicle
    except APIException as e:
        print(f"API connection error: {e}")
        return None
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms (activates motors) the vehicle and takes off to the specified altitude.
    Uses GUIDED mode for safe takeoff.
    """
    print("Initial pre-flight check...")
    # Wait until the vehicle is ready for arming
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

    print(f"Taking off to altitude {aTargetAltitude} meters...")
    vehicle.simple_takeoff(aTargetAltitude) # Takeoff command

    # Wait to reach the target altitude
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {current_altitude:.2f} m")
        # Check if we have reached the altitude with a small tolerance (95%)
        if current_altitude >= aTargetAltitude * 0.95:
            print("Target altitude reached!")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the distance in meters between two LocationGlobal points.

    Uses a simplified formula for short distances:
    https://gis.stackexchange.com/a/61956/160970
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing from point 1 to point 2 in degrees.
    """
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Moves the vehicle for `duration` seconds by sending NED velocity commands.
    Not used directly in ALT_HOLD mode, but useful for GUIDED_NOGPS.
    For ALT_HOLD, we use channel overrides.
    """
    # This function is here as an example, but is not used for ALT_HOLD flight
    # Requires `from pymavlink import mavutil` which is not imported currently
    # msg = vehicle.message_factory.set_position_target_local_ned_encode(
    #     0,       # time_boot_ms (not used)
    #     0, 0,    # target system, target component
    #     mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
    #     0b0000111111000111, # type_mask (velocity only)
    #     0, 0, 0, # x, y, z position (ignored)
    #     velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
    #     0, 0, 0, # x, y, z acceleration (ignored)
    #     0, 0)    # yaw, yaw_rate (ignored)

    # # Send the command multiple times for reliability
    # for _ in range(duration):
    #     vehicle.send_mavlink(msg)
    #     time.sleep(1)
    pass # Function body commented out as mavutil is not imported

def set_channel_override(vehicle, channel, pwm_value):
    """
    Sets the PWM value for the specified channel.
    """
    if channel not in vehicle.channels.overrides:
        vehicle.channels.overrides = {} # Initialize if not already done

    if pwm_value is None: # If None, remove the override for this channel
        if channel in vehicle.channels.overrides:
            del vehicle.channels.overrides[channel]
            # Important! Need to send an empty dictionary or a dictionary with remaining overrides
            # for the change to apply. Simply deleting from the local dictionary is not enough.
            # We will resend the entire dictionary in the main function.
    else:
        vehicle.channels.overrides[str(channel)] = int(pwm_value)
    # Sending the updated dictionary happens outside this function

def condition_yaw(vehicle, heading, relative=False, yaw_speed=10, is_clockwise=1):
    """Sends MAV_CMD_CONDITION_YAW command to vehicle."""
    if relative:
        is_relative = 1  # Yaw relative to direction of travel
    else:
        is_relative = 0  # Yaw is an absolute angle North(0)-West(-90)

    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0,      # confirmation
        heading,    # param 1, yaw in degrees
        yaw_speed,  # param 2, yaw speed deg/s
        is_clockwise, # param 3, direction -1 ccw, +1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)
    
# --- Main Script Code ---

# --- Parameters ---
# Point A (Takeoff/Home) - will be taken from SITL/vehicle after takeoff
# Point B (Target)
target_location = LocationGlobalRelative(50.443326, 30.448078, 100)
#target_location = LocationGlobalRelative(50.449296, 30.457242, 0)

# Tolerances
DISTANCE_TOLERANCE = 5  # m - Allowable distance to point B
YAW_TOLERANCE = 0       # degrees - Allowable deviation from target yaw

# --- Initialization and Connection ---
parser = argparse.ArgumentParser(description='Controls the drone for flight from A to B in ALT_HOLD mode.')
parser.add_argument('--connect', default=CONNECTION_STRING,
                    help="Vehicle connection string (default: %(default)s)")
args = parser.parse_args()

vehicle = connect_vehicle(args.connect)

if vehicle is None:
    print("Failed to connect to vehicle. Exiting.")
    exit() # Exit the script if connection failed

try:
    # --- Takeoff ---
    arm_and_takeoff(vehicle, TARGET_ALTITUDE)
    print("Takeoff complete.")

    # --- Switching to ALT_HOLD ---
    print("Switching to ALT_HOLD mode...")
    vehicle.mode = VehicleMode("ALT_HOLD")
    start_time = time.time()
    while vehicle.mode.name != "ALT_HOLD":
        print(f" Waiting for ALT_HOLD mode confirmation (current: {vehicle.mode.name})...")
        if time.time() - start_time > 10: # Timeout 10 seconds
             print("Error: Failed to switch to ALT_HOLD.")
             raise TimeoutError("Failed to switch to ALT_HOLD")
        time.sleep(0.5)
    print("ALT_HOLD mode set.")
    
    print(f"Adjusting course to point B ({target_location.lat}, {target_location.lon})...")

    INITIAL_YAW_ALIGN_TOLERANCE = 10 # degrees - How close heading needs to be
    ALIGN_TIMEOUT = 60 # seconds - Max time for alignment

    align_start_time = time.time()
    while True:
        if time.time() - align_start_time > ALIGN_TIMEOUT:
            print("!!! Yaw alignment timeout !!!")
            break # Proceed to main flight loop anyway? Or raise error? Let's proceed for now.

        current_location = vehicle.location.global_relative_frame # Needed for bearing calc
        current_heading = vehicle.heading
        bearing_to_target = get_bearing(current_location, target_location)
        bearing_diff = bearing_to_target - current_heading
        # Normalize bearing_diff (-180 to +180)
        if bearing_diff > 180: bearing_diff -= 360
        elif bearing_diff < -180: bearing_diff += 360

        print(f" Alignment: Heading={current_heading:.1f}°, Target={bearing_to_target:.1f}°, Difference={bearing_diff:.1f}°")

        # Check if aligned
        if abs(bearing_diff) <= INITIAL_YAW_ALIGN_TOLERANCE:
            print("Course aligned")
            vehicle.channels.overrides = {} # Clear overrides before main loop
            time.sleep(0.5) # Short pause
            break # Exit alignment loop

        # --- Calculate Roll for Turning (No Pitch) ---
        align_roll_pwm_rate = 120 # Maybe slightly higher for initial turn? Tune this.
        roll_pwm_offset = max(-align_roll_pwm_rate, min(align_roll_pwm_rate, int(bearing_diff * (align_roll_pwm_rate / 90.0))))
        roll_pwm = PWM_NEUTRAL + roll_pwm_offset
        roll_pwm = max(PWM_MIN, min(PWM_MAX, roll_pwm)) # Limit

        # --- Set Overrides (Roll + Neutral Pitch/Throttle/Yaw) ---
        overrides = {
            '1': roll_pwm,              # Roll for turning
            '2': PWM_NEUTRAL,           # <<<< PITCH NEUTRAL (1500) >>>>
            '3': THROTTLE_PWM_NEUTRAL,  # Throttle Neutral
            '4': PWM_NEUTRAL            # Yaw Neutral
        }
        vehicle.channels.overrides = overrides

        time.sleep(0.1) # Loop delay

    # --- Flight to Point B in ALT_HOLD ---
    print(f"Starting flight to point B: {target_location.lat}, {target_location.lon} at altitude {TARGET_ALTITUDE} m")

    flight_start_time = time.time()
    FLIGHT_TIMEOUT = 360 # Maximum flight time to point B (6 minutes)

    while True:
        if time.time() - flight_start_time > FLIGHT_TIMEOUT:
            print("Flight timeout to point B!")
            break

        current_location = vehicle.location.global_relative_frame
        distance_to_target = get_distance_metres(current_location, target_location)
        bearing_to_target = get_bearing(current_location, target_location)
        current_heading = vehicle.heading

        log_extra = ""
        applied_roll = vehicle.channels.overrides.get('1', None) # Get applied roll PWM if dict exists
        if applied_roll: log_extra += f" RollPWM={applied_roll}"

        if distance_to_target < 100: # Log more details when close
             bearing_diff = bearing_to_target - current_heading
             # Normalize bearing_diff (-180 to +180)
             if bearing_diff > 180: bearing_diff -= 360
             elif bearing_diff < -180: bearing_diff += 360
             print(f"CLOSE ({distance_to_target:.1f}m): "
                   f"TgtBear={bearing_to_target:.1f}, CurHead={current_heading:.1f}, "
                   f"BearDiff={bearing_diff:.1f}, GS={vehicle.groundspeed:.2f}{log_extra}")
        else: # Normal logging when further away
             print(f"Dist: {distance_to_target:.1f}m, Bear: {bearing_to_target:.1f}°, Head: {current_heading}°, GS: {vehicle.groundspeed:.2f} m/s")

        # Check if we have reached point B
        if distance_to_target <= DISTANCE_TOLERANCE:
            print("Point B reached!")
            # Reset overrides to stop movement
            vehicle.channels.overrides = {}
            print("Movement overrides reset.")
            break # Exit the flight loop

        # --- Calculation of Control Inputs (Overrides) ---

        # 1. Yaw - Channel 4: Not used for turning during flight for now, keep neutral
        yaw_pwm = PWM_NEUTRAL

        # 2. Throttle - Channel 3: Keep neutral to maintain altitude in ALT_HOLD
        throttle_pwm = THROTTLE_PWM_NEUTRAL

        # 3. Pitch - Channel 2: Forward/backward movement
        # Simply fly forward with a fixed speed (PWM)
        pitch_pwm = PITCH_PWM_FORWARD

        # 4. Roll - Channel 1: Turn to align with the bearing to the target
        bearing_diff = bearing_to_target - current_heading
        # Normalize the bearing difference (-180 to +180)
        if bearing_diff > 180:
            bearing_diff -= 360
        elif bearing_diff < -180:
            bearing_diff += 360

        # Calculate PWM for roll proportional to the bearing error
        # Limit the maximum roll deflection
        roll_pwm_offset = max(-ROLL_PWM_RATE, min(ROLL_PWM_RATE, int(bearing_diff * (ROLL_PWM_RATE/90.0)))) # Simple P-controller
        roll_pwm = PWM_NEUTRAL + roll_pwm_offset

        # Limit PWM within acceptable range
        pitch_pwm = max(PWM_MIN, min(PWM_MAX, pitch_pwm))
        roll_pwm = max(PWM_MIN, min(PWM_MAX, roll_pwm))
        throttle_pwm = max(PWM_MIN, min(PWM_MAX, throttle_pwm))
        yaw_pwm = max(PWM_MIN, min(PWM_MAX, yaw_pwm))

        # Send the overrides command
        overrides = {
            '1': roll_pwm,       # Roll
            '2': pitch_pwm,      # Pitch
            '3': throttle_pwm,   # Throttle
            '4': yaw_pwm         # Yaw
        }
        vehicle.channels.overrides = overrides

        time.sleep(0.1) # Pause between control iterations

    # --- Turning to Target Yaw ---
    print(f"Reached Point B. Turning to yaw {TARGET_YAW} degrees...")

    yaw_start_time = time.time()
    YAW_TIMEOUT = 60 # Maximum time for yaw turn (1 minute)
    vehicle.mode = VehicleMode("GUIDED")
    
    while True:
         if time.time() - yaw_start_time > YAW_TIMEOUT:
            print("Yaw turn timeout!")
            break

         current_heading = vehicle.heading
         heading_diff = TARGET_YAW - current_heading
         # Normalize the bearing difference (-180 to +180)
         if heading_diff > 180:
             heading_diff -= 360
         elif heading_diff < -180:
             print(f" :heading_diff {heading_diff:.1f} deg")
             heading_diff += 360

         print(f" Current Heading: {current_heading} deg, Target: {TARGET_YAW} deg, Error: {heading_diff:.1f} deg")
    
         # Check if target yaw is reached
         if abs(heading_diff) <= YAW_TOLERANCE:
             print("Target yaw reached!")
             vehicle.channels.overrides = {} # Reset all overrides
             print("Yaw overrides reset.")
             break
         else:
                # Determine the direction to turn
                direction = 1 if heading_diff > 0 else -1  # Clockwise or counter-clockwise

                # Send the condition_yaw command
                condition_yaw(vehicle, TARGET_YAW, relative=False, yaw_speed=YAW_SPEED * abs(heading_diff) / 30.0, is_clockwise=direction) # Adjust speed based on error

                time.sleep(0.1)

    # --- Completion ---
    print("Script finished mission execution.")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(5)

except TimeoutError as e:
    print(f"Timeout error: {e}")
    print("Attempting to reset overrides...")
    if 'vehicle' in locals() and vehicle.armed: # Check if vehicle exists and is armed
        vehicle.channels.overrides = {}
except Exception as e:
    print(f"An error occurred: {e}")
    print("Attempting to reset overrides...")
    if 'vehicle' in locals() and vehicle.armed: # Check if vehicle exists and is armed
        vehicle.channels.overrides = {}
finally:
    # In any case (success or error), reset overrides and close the connection
    if 'vehicle' in locals() and vehicle.channels.overrides: # Check if vehicle exists
         print("Final reset of overrides before closing...")
         vehicle.channels.overrides = {}
         time.sleep(1) # Allow time for the command to be sent
    if 'vehicle' in locals() and vehicle.is_connected: # Check if vehicle exists and is connected
        print("Closing connection to the vehicle...")
        vehicle.close()
        print("Connection closed.")