import time
from pymavlink import mavutil, dialects

# Define the new speed parameters
NEW_SPEED = {
    "speed_type": mavutil.mavlink.MAV_SPEED_VX,
    "speed": 5,  # Desired speed value
    "lateral": 0,  # Only used for speed types MAV_SPEED_VX and MAV_SPEED_VY
    "id": 0  # Change to non-zero for a persistent change
}

# Connect to the vehicle
vehicle = mavutil.mavlink_connection(device="udpin:127.0.0.1:14550")

# Wait for a heartbeat
vehicle.wait_heartbeat()
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)


# Intercept and modify the speed command
def intercept_and_modify_speed():
    while True:
        msg = vehicle.recv_match(blocking=True)
        if msg:
            msg_type = msg.get_type()

            if msg_type == 'COMMAND_LONG' and msg.command == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
                print("Intercepted speed command:", msg)

                # Modify the speed parameters
                modified_msg = mavutil.mavlink.MAVLink_command_long_message(
                    msg.target_system,
                    msg.target_component,
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    msg.confirmation,
                    NEW_SPEED["speed_type"],
                    NEW_SPEED["speed"],
                    NEW_SPEED["lateral"],
                    NEW_SPEED["id"],
                    0, 0, 0  # Not used
                )
                # Send the modified message
                vehicle.mav.send(modified_msg)
                print("Sent modified speed command")
            else:
                vehicle.mav.send(msg)


# Main function
def main():
    intercept_and_modify_speed()


# Run the main function
if __name__ == "__main__":
    main()
