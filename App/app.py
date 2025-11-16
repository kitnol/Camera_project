import serial
from esp_idf_monitor.idf_monitor import SerialMonitor
from esp_idf_monitor.base.web_socket_client import WebSocketClient
import queue
from esp_idf_monitor.base.constants import TAG_SERIAL

# 1. Create a serial instance
port = 'COM13'  # or 'COM3' on Windows, adjust to your port
baud_rate = 115200

serial_instance = serial.serial_for_url(
    port, 
    baud_rate, 
    do_not_open=True, 
    exclusive=True
)

# Optional: set write timeout (not for RFC2217)
if not port.startswith('rfc2217://'):
    serial_instance.write_timeout = 0.3

# 2. Initialize the SerialMonitor
monitor = SerialMonitor(
    serial_instance=serial_instance,
    elf_files=['path/to/your/app.elf'],  # Path to your ELF file(s)
    print_filter='*',  # Default filter, shows all output
    make='make',  # or 'idf.py' if using ESP-IDF
    encrypted=False,
    reset=True,  # Reset on start
    open_port_attempts=1,
    toolchain_prefix='xtensa-esp32-elf-',  # Adjust for your target
    eol='CRLF',
    decode_coredumps='info',
    decode_panic='backtrace',
    target='esp32',  # 'esp32', 'esp32s2', 'esp32s3', 'esp32c3', etc.
    websocket_client=None,  # Optional WebSocket client
    enable_address_decoding=True,
    timestamps=False,
    timestamp_format='',
    force_color=False,
    disable_auto_color=False,
    rom_elf_file=None
)

monitor.serial_reader.start()

line_buffer = ""

# Read lines from the event queue
try:
    while monitor.serial_reader.alive:
        try:
            event_tag, data = monitor.event_queue.get(timeout=1.0)
            
            if event_tag == TAG_SERIAL:
                # Decode bytes to string
                chunk = data.decode('utf-8', errors='ignore')
                line_buffer += chunk
                
                # Split by newlines and process complete lines
                while '\n' in line_buffer:
                    line, line_buffer = line_buffer.split('\n', 1)
                    line = line.rstrip('\r')  # Remove \r if present
                    
                    if line:  # Only print non-empty lines
                        print(f"[LINE] {line}")
                        
        except queue.Empty:
            continue
            
except KeyboardInterrupt:
    print("\nStopping monitor...")
finally:
    monitor.console_reader.stop()
    monitor.serial_reader.stop()