#!/usr/bin/env python3
"""
Dynamixel ID Scanner and Setter
"""
from dynamixel_sdk import *

PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'

ADDR_ID = 7
ADDR_BAUDRATE = 8
ADDR_OPERATING_MODE = 11

def scan_servos():
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
    
    if not port_handler.openPort():
        print("Failed to open port")
        return
    
    if not port_handler.setBaudRate(BAUDRATE):
        print("Failed to set baudrate")
        return
    
    print("Scanning for Dynamixel servos (ID 0-10)...")
    found_servos = []
    
    for servo_id in range(0, 11):
        # Try to ping the servo
        dxl_model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, servo_id)
        
        if dxl_comm_result == COMM_SUCCESS:
            print(f"  [ID: {servo_id}] Model: {dxl_model_number}")
            found_servos.append(servo_id)
    
    port_handler.closePort()
    return found_servos

def set_servo_id(old_id, new_id):
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
    
    if not port_handler.openPort():
        print("Failed to open port")
        return False
    
    if not port_handler.setBaudRate(BAUDRATE):
        print("Failed to set baudrate")
        return False
    
    # Disable torque first (required to change ID)
    packet_handler.write1ByteTxRx(port_handler, old_id, 64, 0)
    time.sleep(0.1)
    
    # Write new ID
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
        port_handler, old_id, ADDR_ID, new_id)
    
    if dxl_comm_result == COMM_SUCCESS:
        print(f"Successfully changed ID from {old_id} to {new_id}")
        success = True
    else:
        print(f"Failed to change ID: {packet_handler.getTxRxResult(dxl_comm_result)}")
        success = False
    
    port_handler.closePort()
    return success

def set_operating_mode(servo_id, mode=3):
    """Set operating mode (3 = position control)"""
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
    
    port_handler.openPort()
    port_handler.setBaudRate(BAUDRATE)
    
    # Disable torque
    packet_handler.write1ByteTxRx(port_handler, servo_id, 64, 0)
    time.sleep(0.1)
    
    # Set mode
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
        port_handler, servo_id, ADDR_OPERATING_MODE, mode)
    
    if dxl_comm_result == COMM_SUCCESS:
        print(f"Set servo {servo_id} to operating mode {mode}")
    
    port_handler.closePort()

if __name__ == "__main__":
    import time
    
    print("=== Dynamixel Setup Tool ===\n")
    
    # Scan for servos
    found = scan_servos()
    
    if not found:
        print("\nNo servos found! Check connections and baudrate.")
    else:
        print(f"\nFound {len(found)} servo(s): {found}")
        
        # If you have servos with default ID (likely 1), set them properly
        print("\n--- Setup Instructions ---")
        print("1. Connect ONLY the pan servo, run this script, note its ID")
        print("2. If ID is not 1, change it to 1")
        print("3. Disconnect pan servo, connect ONLY tilt servo")
        print("4. Change tilt servo ID to 2")
        print("5. Connect both servos in daisy chain\n")
        
        # Interactive ID change
        change = input("Change a servo ID? (y/n): ")
        if change.lower() == 'y':
            old = int(input("Current ID: "))
            new = int(input("New ID: "))
            if set_servo_id(old, new):
                # Set operating mode after ID change
                set_operating_mode(new, 3)
