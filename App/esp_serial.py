import serial
import struct
import time
import hashlib
from pathlib import Path
from typing import Optional, List, Tuple

class ESP32S3Flasher:
    """Pure Python ESP32-S3 flasher using ROM bootloader protocol."""
    
    # ESP32-S3 ROM bootloader commands
    ESP_FLASH_BEGIN = 0x02
    ESP_FLASH_DATA = 0x03
    ESP_FLASH_END = 0x04
    ESP_MEM_BEGIN = 0x05
    ESP_MEM_END = 0x06
    ESP_MEM_DATA = 0x07
    ESP_SYNC = 0x08
    ESP_WRITE_REG = 0x09
    ESP_READ_REG = 0x0a
    ESP_SPI_ATTACH = 0x0D
    ESP_CHANGE_BAUDRATE = 0x0F
    ESP_SPI_FLASH_MD5 = 0x13
    
    # Flash parameters
    FLASH_WRITE_SIZE = 0x400  # 1024 bytes per packet
    FLASH_SECTOR_SIZE = 0x1000  # 4096 bytes
    FLASH_BLOCK_SIZE = 0x10000  # 64KB
    
    def __init__(self, port: str, baud_rate: int = 115200):
        """
        Initialize ESP32-S3 flasher.
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0' or 'COM3')
            baud_rate: Initial baud rate (default: 115200)
        """
        self.port = port
        self.baud_rate = baud_rate
        self.ser: Optional[serial.Serial] = None
        
    def connect(self) -> bool:
        """Connect to ESP32-S3 and enter bootloader mode."""
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=3,
                write_timeout=3
            )
            
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            
            # Enter bootloader mode
            print("Entering bootloader mode...")
            self._reset_to_bootloader()
            time.sleep(0.25)  # Give it more time
            
            # Sync with bootloader
            print("Syncing with bootloader...")
            if not self._sync():
                print("Failed to sync with bootloader")
                print("Make sure:")
                print("  1. The ESP32-S3 is connected properly")
                print("  2. No other program is using the serial port")
                print("  3. Try pressing the BOOT button while resetting")
                return False
            
            print("Successfully synced with ESP32-S3 bootloader")
            return True
            
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return False
    
    def _reset_to_bootloader(self):
        """Reset ESP32 into bootloader mode using DTR/RTS."""
        # DTR = GPIO0, RTS = EN (reset)
        # To enter bootloader: GPIO0 must be LOW when reset goes HIGH
        
        self.ser.setDTR(False)  # IO0 = HIGH
        self.ser.setRTS(True)   # EN = LOW (reset)
        time.sleep(0.1)
        
        self.ser.setDTR(True)   # IO0 = LOW (boot mode)
        time.sleep(0.05)
        
        self.ser.setRTS(False)  # EN = HIGH (release reset)
        time.sleep(0.05)
        
        self.ser.setDTR(False)  # IO0 = HIGH (release)
    
    def _sync(self) -> bool:
        """Sync with the ROM bootloader."""
        # Clear any pending data
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
        # Sync packet: 0x08 command with 36 bytes of data (0x07 0x07 0x12 0x20 followed by 32x 0x55)
        sync_data = b'\x07\x07\x12\x20' + b'\x55' * 32
        
        for attempt in range(7):
            # Send sync command with SLIP encoding
            self._send_command(self.ESP_SYNC, sync_data)
            
            # Wait a bit for response
            time.sleep(0.1)
            
            # Try to read response
            try:
                response = self._read_response(timeout=0.5)
                if response is not None:
                    status, data = response
                    if status == 0:
                        # Send additional syncs to clear
                        for _ in range(7):
                            self._send_command(self.ESP_SYNC, sync_data)
                            self._read_response(timeout=0.1)
                        return True
            except Exception as e:
                pass
            
            time.sleep(0.05)
        
        return False
    
    def _send_command(self, op: int, data: bytes = b'', checksum: int = 0) -> bool:
        """Send a command to the bootloader using SLIP encoding."""
        # Packet format: 0x00 op size checksum data
        size = len(data)
        packet = struct.pack('<BBHI', 0x00, op, size, checksum) + data
        
        # SLIP encode the packet
        encoded = self._slip_encode(packet)
        
        self.ser.write(encoded)
        self.ser.flush()
        return True
    
    def _slip_encode(self, data: bytes) -> bytes:
        """Encode data using SLIP protocol."""
        # SLIP special bytes
        SLIP_END = 0xC0
        SLIP_ESC = 0xDB
        SLIP_ESC_END = 0xDC
        SLIP_ESC_ESC = 0xDD
        
        encoded = bytearray([SLIP_END])
        
        for b in data:
            if b == SLIP_END:
                encoded.extend([SLIP_ESC, SLIP_ESC_END])
            elif b == SLIP_ESC:
                encoded.extend([SLIP_ESC, SLIP_ESC_ESC])
            else:
                encoded.append(b)
        
        encoded.append(SLIP_END)
        return bytes(encoded)
    
    def _slip_decode(self, data: bytes) -> bytes:
        """Decode SLIP encoded data."""
        SLIP_END = 0xC0
        SLIP_ESC = 0xDB
        SLIP_ESC_END = 0xDC
        SLIP_ESC_ESC = 0xDD
        
        decoded = bytearray()
        i = 0
        
        while i < len(data):
            if data[i] == SLIP_END:
                i += 1
                continue
            elif data[i] == SLIP_ESC:
                i += 1
                if i < len(data):
                    if data[i] == SLIP_ESC_END:
                        decoded.append(SLIP_END)
                    elif data[i] == SLIP_ESC_ESC:
                        decoded.append(SLIP_ESC)
            else:
                decoded.append(data[i])
            i += 1
        
        return bytes(decoded)
    
    def _read_response(self, timeout: float = 3.0) -> Optional[Tuple[int, bytes]]:
        """Read response from bootloader with SLIP decoding."""
        old_timeout = self.ser.timeout
        self.ser.timeout = timeout
        
        try:
            # Read until we get a complete SLIP packet (ends with 0xC0)
            packet = bytearray()
            start_time = time.time()
            
            while (time.time() - start_time) < timeout:
                if self.ser.in_waiting > 0:
                    b = self.ser.read(1)
                    if b:
                        packet.extend(b)
                        # Check if we have a complete packet (ends with 0xC0 and has content)
                        if b[0] == 0xC0 and len(packet) > 1:
                            break
                else:
                    time.sleep(0.01)
            
            if len(packet) < 2:
                return None
            
            # SLIP decode
            decoded = self._slip_decode(bytes(packet))
            
            if len(decoded) < 8:
                return None
            
            # Parse header: direction(1) command(1) size(2) value(4)
            direction, command, size, value = struct.unpack('<BBHI', decoded[:8])
            
            if direction != 0x01:  # Response should have direction = 1
                return None
            
            # Extract data if any
            data = b''
            if size > 0 and len(decoded) >= 8 + size:
                data = decoded[8:8+size]
            
            return (value, data)
            
        finally:
            self.ser.timeout = old_timeout
    
    def change_baud_rate(self, baud_rate: int) -> bool:
        """Change baud rate for faster flashing."""
        print(f"Changing baud rate to {baud_rate}...")
        
        # Send baud rate change command
        data = struct.pack('<II', baud_rate, 0)
        self._send_command(self.ESP_CHANGE_BAUDRATE, data)
        
        time.sleep(0.05)
        self.ser.baudrate = baud_rate
        time.sleep(0.05)
        
        # Verify with sync
        return self._sync()
    
    def flash_begin(self, size: int, offset: int) -> bool:
        """Begin flash operation."""
        num_blocks = (size + self.FLASH_WRITE_SIZE - 1) // self.FLASH_WRITE_SIZE
        erase_size = (size + self.FLASH_SECTOR_SIZE - 1) // self.FLASH_SECTOR_SIZE * self.FLASH_SECTOR_SIZE
        
        # Calculate timeout (erasing takes time)
        timeout = max(10, erase_size // 100000)
        
        # ESP32-S3 needs 5 parameters: erase_size, num_blocks, block_size, offset, encrypted
        data = struct.pack('<IIIII', erase_size, num_blocks, self.FLASH_WRITE_SIZE, offset, 0)
        self._send_command(self.ESP_FLASH_BEGIN, data)
        
        response = self._read_response(timeout=timeout)
        if response is None:
            return False
        
        status, resp_data = response
        return status == 0
    
    def flash_data(self, data: bytes, seq: int) -> bool:
        """Write a block of data to flash."""
        # Pad data to FLASH_WRITE_SIZE
        if len(data) < self.FLASH_WRITE_SIZE:
            data += b'\xff' * (self.FLASH_WRITE_SIZE - len(data))
        
        # Calculate checksum
        checksum = sum(data) & 0xFF
        
        # Packet: size(4) seq(4) 0(4) 0(4) data
        packet = struct.pack('<IIII', len(data), seq, 0, 0) + data
        
        self._send_command(self.ESP_FLASH_DATA, packet, checksum)
        
        response = self._read_response(timeout=10)
        return response is not None and response[0] == 0
    
    def flash_end(self, reboot: bool = False) -> bool:
        """End flash operation."""
        data = struct.pack('<I', 1 if reboot else 0)
        self._send_command(self.ESP_FLASH_END, data)
        
        response = self._read_response()
        return response is not None and response[0] == 0
    
    def write_flash(self, offset: int, data: bytes, show_progress: bool = True) -> bool:
        """
        Write data to flash at specified offset.
        
        Args:
            offset: Flash offset address
            data: Binary data to write
            show_progress: Show progress bar
        
        Returns:
            bool: True if successful
        """
        print(f"\nWriting {len(data)} bytes at 0x{offset:08x}...")
        
        # Begin flash operation
        if not self.flash_begin(len(data), offset):
            print("Failed to begin flash operation")
            return False
        
        # Write data in blocks
        num_blocks = (len(data) + self.FLASH_WRITE_SIZE - 1) // self.FLASH_WRITE_SIZE
        
        for seq in range(num_blocks):
            start = seq * self.FLASH_WRITE_SIZE
            end = min(start + self.FLASH_WRITE_SIZE, len(data))
            block = data[start:end]
            
            if not self.flash_data(block, seq):
                print(f"\nFailed to write block {seq}")
                return False
            
            if show_progress:
                pct = (seq + 1) * 100 // num_blocks
                print(f"\rProgress: [{('#' * (pct // 2)).ljust(50)}] {pct}%", end='')
        
        if show_progress:
            print()  # New line after progress
        
        # End flash operation
        if not self.flash_end():
            print("Failed to end flash operation")
            return False
        
        print("✓ Write complete")
        return True
    
    def flash_file(self, offset: int, filepath: str) -> bool:
        """Flash a binary file to specified offset."""
        path = Path(filepath)
        
        if not path.exists():
            print(f"Error: File not found: {filepath}")
            return False
        
        with open(path, 'rb') as f:
            data = f.read()
        
        print(f"File: {path.name} ({len(data)} bytes)")
        return self.write_flash(offset, data)
    
    def close(self):
        """Close serial connection."""
        if self.ser:
            self.ser.close()


def flash_esp32s3(
    port: str,
    build_dir: str,
    baud_rate: int = 460800,
    initial_baud: int = 115200,
    manual_reset: bool = False
) -> bool:
    """
    Flash ESP32-S3 with firmware from ESP-IDF build directory.
    
    Args:
        port: Serial port (e.g., '/dev/ttyUSB0' or 'COM3')
        build_dir: Path to ESP-IDF build directory
        baud_rate: Flash baud rate (default: 460800)
        initial_baud: Initial connection baud rate (default: 115200)
        manual_reset: If True, wait for user to manually reset the board
    
    Returns:
        bool: True if successful
    """
    
    build_path = Path(build_dir)
    
    # Files to flash with their offsets
    flash_files = [
        (0x0, build_path / "bootloader" / "bootloader.bin"),
        (0x8000, build_path / "partition_table" / "partition-table.bin"),
    ]
    
    # Find application binary
    app_bin = None
    for f in build_path.glob("*.bin"):
        if f.name not in ["bootloader.bin", "partition-table.bin"]:
            if not f.name.startswith("ota_data"):
                app_bin = f
                break
    
    if app_bin:
        flash_files.append((0x10000, app_bin))
    else:
        print("Warning: Application binary not found")
    
    # Verify all files exist
    for offset, filepath in flash_files:
        if not filepath.exists():
            print(f"Error: File not found: {filepath}")
            return False
    
    # Create flasher and connect
    flasher = ESP32S3Flasher(port, initial_baud)
    
    try:
        if manual_reset:
            print("\n" + "="*60)
            print("MANUAL RESET MODE")
            print("="*60)
            print("Please do the following:")
            print("1. Hold down the BOOT button on your ESP32-S3")
            print("2. Press and release the RESET button")
            print("3. Release the BOOT button")
            print("4. Press ENTER to continue...")
            print("="*60)
            input()
        
        if not flasher.connect():
            if not manual_reset:
                print("\n" + "="*60)
                print("Automatic reset failed. Try manual reset:")
                print("Run with: flash_esp32s3(port='COM13', build_dir='./build', manual_reset=True)")
                print("="*60)
            return False
        
        # Change to higher baud rate for faster flashing
        if baud_rate != initial_baud:
            if not flasher.change_baud_rate(baud_rate):
                print("Failed to change baud rate, continuing at initial speed")
        
        # Flash each file
        for offset, filepath in flash_files:
            if not flasher.flash_file(offset, str(filepath)):
                return False
        
        print("\n✓ All files flashed successfully!")
        print("Resetting device...")
        
        # Reset device
        flasher.ser.setDTR(False)
        flasher.ser.setRTS(True)
        time.sleep(0.1)
        flasher.ser.setRTS(False)
        
        return True
        
    finally:
        flasher.close()


# Example usage
if __name__ == "__main__":
    # Try automatic reset first
    success = flash_esp32s3(
        port="COM13",  # Change to your port
        build_dir="../build",
        baud_rate=460800
    )
    
    # If automatic fails, try manual reset mode
    # success = flash_esp32s3(
    #     port="COM13",
    #     build_dir="./build",
    #     baud_rate=460800,
    #     manual_reset=True
    # )
    
    if success:
        print("\nFlashing completed successfully!")
    else:
        print("\nFlashing failed!")