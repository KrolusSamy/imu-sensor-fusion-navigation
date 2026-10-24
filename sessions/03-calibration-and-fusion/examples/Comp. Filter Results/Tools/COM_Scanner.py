# ==========================================================================================
# File Name         : COM_Scanner.py
# Description       : Utility script to list all available serial ports and help students 
#                     identify which port their microcontroller is connected to.
# Requirements      : pip install pyserial
# ==========================================================================================

import serial.tools.list_ports

def scan_ports():
    print("🔍 Scanning for connected USB/Serial devices...\n")
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("❌ No COM ports found. Please check your USB cable and connections.")
        return

    for port in ports:
        print(f"📍 Port: {port.device}")
        print(f"   Description: {port.description}")
        
        # Simple heuristic to identify common serial chips
        desc_lower = port.description.lower()
        if "arduino" in desc_lower or "ch340" in desc_lower or "cp210x" in desc_lower or "usbmodem" in desc_lower:
            print("   ⭐ -> TIP: This is highly likely to be your microcontroller!")
            
        print("-" * 50)

if __name__ == "__main__":
    scan_ports()