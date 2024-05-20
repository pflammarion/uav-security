import time
from pymavlink import mavutil, dialects
import pymavlink.dialects.v20.all as dialect

# Connect to the vehicle
vehicle = mavutil.mavlink_connection(device="udpin:127.0.0.1:14550")

# Wait for a heartbeat to ensure connection
vehicle.wait_heartbeat()
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# Function to change vehicle mode
def set_vehicle_mode(mode):
    mode_id = vehicle.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Unknown mode: {mode}")
        return False

    vehicle.set_mode(mode_id)
    return True

# Function to arm the vehicle
def arm_vehicle():
    vehicle.arducopter_arm()
    vehicle.motors_armed_wait()
    print("Vehicle armed")

# Function to disarm the vehicle
def disarm_vehicle():
    vehicle.arducopter_disarm()
    vehicle.motors_disarmed_wait()
    print("Vehicle disarmed")

# Function to send a heartbeat message to maintain connection
def send_heartbeat():
    while True:
        vehicle.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_GCS,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            base_mode=0,
            custom_mode=0,
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE
        )
        time.sleep(1)

# Function to send a reboot command to the UAV
def send_reboot_command():
    print("Sending reboot command to the UAV")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,  # Confirmation
        1,  # Param 1: 1 to reboot the autopilot
        0, 0, 0, 0, 0, 0  # Other params set to 0
    )

    ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Reboot command accepted")
        else:
            print(f"Reboot command failed with result: {ack.result}")

# Main function
def main():
    # Start sending heartbeat messages in the background
    import threading
    heartbeat_thread = threading.Thread(target=send_heartbeat)
    heartbeat_thread.daemon = True
    heartbeat_thread.start()

    # Wait a few seconds to establish a stable connection
    time.sleep(5)

    # Ensure the vehicle is in the correct mode
    if not set_vehicle_mode('STABILIZE'):
        print("Failed to set mode to STABILIZE")
        return

    #Ensure the vehicle is disarmed
    disarm_vehicle()

    # Send the reboot command
    send_reboot_command()

    # Wait to observe the effects
    time.sleep(10)

if __name__ == "__main__":
    main()
