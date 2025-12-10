import serial
import serial.tools.list_ports
import time
import sys

def list_available_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    available_ports = []
    
    print("\nAvailable COM ports:")
    print("-" * 50)
    for i, port in enumerate(ports):
        print(f"{i + 1}. {port.device} - {port.description}")
        available_ports.append(port.device)
    print("-" * 50)
    
    return available_ports

def connect_to_esp(port, baudrate=115200, timeout=1):
    """Establish serial connection to ESP"""
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for connection to stabilize
        print(f"\nConnected to {port} at {baudrate} baud")
        print("Reading data... (Press Ctrl+C to stop)\n")
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        return None

def read_serial_data(ser):
    """Read and display data from serial port"""
    try:
        while True:
            if ser.in_waiting > 0:
                # Read line from serial port
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    print(data)
    except KeyboardInterrupt:
        print("\n\nStopping serial reader...")
    except Exception as e:
        print(f"\nError reading data: {e}")
    finally:
        ser.close()
        print("Serial connection closed.")

def find_esp_port():
    """Automatically find ESP device port"""
    ports = serial.tools.list_ports.comports()
    
    # Common ESP device identifiers
    esp_identifiers = ['CH340', 'CP210', 'UART', 'USB-SERIAL', 'USB Serial', 'FTDI']
    
    for port in ports:
        description = port.description.upper()
        # Check if port description contains ESP identifiers
        if any(identifier.upper() in description for identifier in esp_identifiers):
            return port.device
    
    # If no ESP-specific port found, return the first available port
    if ports:
        return ports[0].device
    
    return None

def main():
    print("Auto-connecting to ESP device...")
    
    # Automatically find ESP port
    selected_port = find_esp_port()
    
    if not selected_port:
        print("No serial ports found!")
        print("\nAvailable ports:")
        list_available_ports()
        return
    
    # Default baudrate for ESP devices
    baudrate = 115200
    
    print(f"Using port: {selected_port}")
    print(f"Baudrate: {baudrate}")
    
    # Connect to ESP
    ser = connect_to_esp(selected_port, baudrate)
    
    if ser:
        # Read data continuously
        read_serial_data(ser)

if __name__ == "__main__":
    main()