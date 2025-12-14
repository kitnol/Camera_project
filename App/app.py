from PyQt5 import QtCore, QtGui, QtWidgets
import serial.tools.list_ports
import serial
import queue
from tkinter import filedialog as fd
from tkinter.messagebox import askyesno, askquestion
import subprocess
import os

import esptool
from esptool.cmds import detect_chip


class DateTimeSelectionDialog(QtWidgets.QDialog):
    """Custom dialog for selecting date and time"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Set Custom Date and Time")
        self.setGeometry(100, 100, 400, 300)
        
        layout = QtWidgets.QVBoxLayout()
        
        # Date selection
        date_layout = QtWidgets.QHBoxLayout()
        date_layout.addWidget(QtWidgets.QLabel("Date:"))
        self.date_edit = QtWidgets.QDateEdit()
        self.date_edit.setDate(QtCore.QDate.currentDate())
        self.date_edit.setCalendarPopup(True)
        date_layout.addWidget(self.date_edit)
        layout.addLayout(date_layout)
        
        # Time selection
        time_layout = QtWidgets.QHBoxLayout()
        time_layout.addWidget(QtWidgets.QLabel("Time:"))
        self.time_edit = QtWidgets.QTimeEdit()
        self.time_edit.setTime(QtCore.QTime.currentTime())
        self.time_edit.setDisplayFormat("HH:mm")
        time_layout.addWidget(self.time_edit)
        layout.addLayout(time_layout)
        
        # Buttons
        button_layout = QtWidgets.QHBoxLayout()
        
        ok_button = QtWidgets.QPushButton("OK")
        ok_button.clicked.connect(self.accept)
        button_layout.addWidget(ok_button)
        
        cancel_button = QtWidgets.QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)
        
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def get_selected_datetime(self):
        """Return selected date and time"""
        date = self.date_edit.date()
        time = self.time_edit.time()
        return {
            'year': date.year(),
            'month': date.month(),
            'day': date.day(),
            'hour': time.hour(),
            'minute': time.minute()
        }

PORT = "COM13"  # Change to your port
FLASH_ADDRESS = 0x10000
folder_path = ""

def progress_callback(percent):
    print(f"Wrote: {int(percent)}%")

def flash_esp_firmware(port, bin_file, flash_address=0x0, progress_callback=None):
    """
    Flash firmware to an ESP microcontroller.
    
    Args:
        port: Serial port where ESP is connected (e.g., 'COM3' or '/dev/ttyUSB0')
        bin_file: Path to the binary firmware file
        flash_address: Starting address in flash memory (default: 0x0)
        progress_callback: Optional callback function for progress updates (receives percentage 0-100)
    
    Returns:
        dict: Information about the flashing operation including chip description and success status
    """
    try:
        with detect_chip(port) as esp:
            # Detect chip and get info
            description = esp.get_chip_description()
            features = esp.get_chip_features()
            print(f"Detected ESP on port {port}: {description}")
            print("Features:", ", ".join(features))

            # Load stub for faster flashing
            esp = esp.run_stub()
            
            with open(bin_file, 'rb') as binary:
                # Load the binary
                binary_data = binary.read()
                total_size = len(binary_data)
                print(f"Binary size: {total_size} bytes")

                # Write binary blocks
                esp.flash_begin(total_size, flash_address)
                for i in range(0, total_size, esp.FLASH_WRITE_SIZE):
                    block = binary_data[i:i + esp.FLASH_WRITE_SIZE]
                    # Pad the last block
                    block = block + bytes([0xFF]) * (esp.FLASH_WRITE_SIZE - len(block))
                    esp.flash_block(block, i + flash_address)
                    
                    # Call progress callback if provided
                    if progress_callback:
                        progress_callback(i / total_size * 100)
                
                esp.flash_finish()

                # Reset the chip out of bootloader mode
                esp.hard_reset()
                
                # Final progress update
                if progress_callback:
                    progress_callback(100)
                
                return {
                    'success': True,
                    'chip': description,
                    'features': features,
                    'size': total_size
                }
    
    except Exception as e:
        print(f"Error flashing firmware: {e}")
        return {
            'success': False,
            'error': str(e)
        }

class SerialReaderThread(QtCore.QThread):
    """Thread to read serial data without blocking GUI"""
    line_received = QtCore.pyqtSignal(str)
    
    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.running = True
        self.line_buffer = ""
        
    def run(self):
        """Read lines from serial in background thread"""
        while self.running:
            try:
                if self.serial_port.in_waiting > 0:
                    # Read available bytes
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    chunk = data.decode('utf-8', errors='ignore')
                    self.line_buffer += chunk
                    
                    # Process complete lines
                    while '\n' in self.line_buffer:
                        line, self.line_buffer = self.line_buffer.split('\n', 1)
                        line = line.rstrip('\r')
                        
                        if line:
                            self.line_received.emit(line)
                else:
                    # Small delay to prevent busy waiting
                    QtCore.QThread.msleep(10)
                    
            except Exception as e:
                print(f"Error in serial reader: {e}")
                break
    
    def stop(self):
        """Stop the thread"""
        self.running = False


class Ui_MainWindow(object):
    file_path = ""
    port = ""
    isRTCinitialized = False
    def __init__(self):
        self.serial_port = None
        self.reader_thread = None
    
    def changeText(self, text):
        self.label_2.setText(text)

    def connect_to_esp32(self, baudrate=115200):
        """Connect to ESP32 via serial port"""
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            self.label_2.setText("No ports found")
            self.textBrowser.append("Error: No serial ports detected")
            return
        
        # Try each available port
        for port_info in ports:
            port_name = port_info.device
            try:
                self.textBrowser.append(f"Trying {port_name}...")
                
                # Open serial port directly
                self.serial_port = serial.Serial(
                    port=port_name,
                    baudrate=baudrate,
                    timeout=1,
                    write_timeout=1
                )
                
                # Success!
                self.label_2.setText(f"Connected: {port_name}")
                self.textBrowser.append(f"✓ Connected to {port_name} at {baudrate} baud")
                
                # Start reading in background thread
                self.start_reading()
                
                # Update button states
                self.pushButton_2.setEnabled(False)  # Disable Connect button
                self.pushButton_5.setEnabled(True)   # Enable Disconnect button
                self.pushButton.setEnabled(True)     # Enable Login button
                
                return
                
            except serial.SerialException as e:
                self.textBrowser.append(f"✗ Failed on {port_name}: {e}")
                continue
            except Exception as e:
                self.textBrowser.append(f"✗ Error on {port_name}: {e}")
                continue
        
        # If we get here, all ports failed
        self.label_2.setText("Connection failed")
        self.textBrowser.append("Error: Could not connect to any serial port")

    def disconnect_from_esp32(self):
        """Disconnect from ESP32 and stop all communication"""
        if self.serial_port is None:
            self.textBrowser.append("Not connected to any device")
            return
            
        try:
            self.textBrowser.append("Disconnecting from ESP32...")
            
            # Stop the reader thread
            if self.reader_thread is not None:
                self.reader_thread.stop()
                self.reader_thread.wait(2000)  # Wait up to 2 seconds
                self.reader_thread = None
            
            # Close the serial port
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                self.textBrowser.append("Serial port closed")
            
            # Clean up
            self.serial_port = None
            self.isRTCinitialized = False
            
            self.textBrowser.append("✓ Disconnected successfully")
            self.label_2.setText("Disconnected")
            
            # Update button states
            self.pushButton_2.setEnabled(True)   # Enable Connect button
            self.pushButton_5.setEnabled(False)  # Disable Disconnect button    
            self.pushButton.setEnabled(False)    # Disable Login button
            
        except Exception as e:
            self.textBrowser.append(f"Error during disconnect: {e}")

    def start_reading(self):
        """Start the serial reader thread"""
        if self.reader_thread is not None:
            self.reader_thread.stop()
            self.reader_thread.wait()
        
        self.reader_thread = SerialReaderThread(self.serial_port)
        self.reader_thread.line_received.connect(self.on_line_received)
        self.reader_thread.start()
    
    def on_line_received(self, line):
        """Handle received line (runs in GUI thread)"""
        self.textBrowser.append(f"{line}")
    
    def send_login_command(self):
        """Send login command with password to ESP32"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.textBrowser.append("Error: Not connected to ESP32")
            return
        
        password = self.lineEdit.text()
        
        if not password:
            self.textBrowser.append("Error: Password field is empty")
            return
        
        command = f"login {password}\n"
        
        try:
            # Write to serial
            self.serial_port.write(command.encode('utf-8'))
            self.textBrowser.append(f">>> Sent: {command.strip()}")
            
            # Optional: Clear the password field after sending
            # self.lineEdit.clear()
            
        except Exception as e:
            self.textBrowser.append(f"Error sending command: {e}")
    
    def send_on_command(self):
        """Send on command to ESP32"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.textBrowser.append("Error: Not connected to ESP32")
            return
        
        command = f"on\n"
        
        try:
            # Write to serial
            self.serial_port.write(command.encode('utf-8'))
            self.textBrowser.append(f">>> Sent: {command.strip()}")
            
            # Optional: Clear the password field after sending
            # self.lineEdit.clear()
            
        except Exception as e:
            self.textBrowser.append(f"Error sending command: {e}")

    def send_gettime_command(self):
        """Send gettime command to ESP32"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.textBrowser.append("Error: Not connected to ESP32")
            return
        
        if not self.isRTCinitialized:
            command = f"init_rtc\n"
            try:
                # Write to serial
                self.serial_port.write(command.encode('utf-8'))
                self.textBrowser.append(f">>> Sent: {command.strip()}")
                
            except Exception as e:
                self.textBrowser.append(f"Error sending command: {e}")
                return
            self.isRTCinitialized = True

        command = f"gettime\n"
        
        try:
            # Write to serial
            self.serial_port.write(command.encode('utf-8'))
            self.textBrowser.append(f">>> Sent: {command.strip()}")
            
            # Optional: Clear the password field after sending
            # self.lineEdit.clear()
            
        except Exception as e:
            self.textBrowser.append(f"Error sending command: {e}")

    def send_settime_command(self):
        """Send settime command to ESP32 with user-selected time"""
        current_time = QtCore.QDateTime.currentDateTime()
        formatted_time = current_time.toString("yyyy-MM-dd HH:mm:ss")
        year = current_time.date().year()
        month = current_time.date().month()
        day = current_time.date().day()
        hour = current_time.time().hour()
        minute = current_time.time().minute()
        print(formatted_time)
        print(year, month, day, hour, minute)
        answer = askyesno(title='Confirmation',
                          message='Do you want to set the time to ' + formatted_time + '?')
        
        if answer:
            # User confirmed - use current time
            time_data = {
                'year': year,
                'month': month,
                'day': day,
                'hour': hour,
                'minute': minute
            }
        else:
            # User declined - open custom selection dialog
            dialog = DateTimeSelectionDialog()
            if dialog.exec_() == QtWidgets.QDialog.Accepted:
                time_data = dialog.get_selected_datetime()
            else:
                self.textBrowser.append("Time setting canceled")
                return
        
        # Send the time to ESP32
        if self.serial_port is None or not self.serial_port.is_open:
            self.textBrowser.append("Error: Not connected to ESP32")
            return
        
        if not self.isRTCinitialized:
            command = f"init_rtc\n"
            try:
                # Write to serial
                self.serial_port.write(command.encode('utf-8'))
                self.textBrowser.append(f">>> Sent: {command.strip()}")
                
            except Exception as e:
                self.textBrowser.append(f"Error sending command: {e}")
                return
            self.isRTCinitialized = True

        command = f"settime {time_data['year']} {time_data['month']} {time_data['day']} {time_data['hour']} {time_data['minute']}\n"
        
        try:
            # Write to serial
            self.serial_port.write(command.encode('utf-8'))
            self.textBrowser.append(f">>> Sent: {command.strip()}")
            
        except Exception as e:
            self.textBrowser.append(f"Error sending command: {e}")

        formatted_selected_time = f"{time_data['year']:04d}-{time_data['month']:02d}-{time_data['day']:02d} {time_data['hour']:02d}:{time_data['minute']:02d}"
        self.textBrowser.append(f"Setting time to: {formatted_selected_time}")
        

    def send_off_command(self):
        """Send login command with password to ESP32"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.textBrowser.append("Error: Not connected to ESP32")
            return
        
        command = f"off\n"
        
        try:
            # Write to serial
            self.serial_port.write(command.encode('utf-8'))
            self.textBrowser.append(f">>> Sent: {command.strip()}")
            
            # Optional: Clear the password field after sending
            # self.lineEdit.clear()
            
        except Exception as e:
            self.textBrowser.append(f"Error sending command: {e}")

    def send_sd_command(self):
        """Send login command with password to ESP32"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.textBrowser.append("Error: Not connected to ESP32")
            return
        
        command = f"sd\n"
        
        try:
            # Write to serial
            self.serial_port.write(command.encode('utf-8'))
            self.textBrowser.append(f">>> Sent: {command.strip()}")
            
            # Optional: Clear the password field after sending
            # self.lineEdit.clear()
            
        except Exception as e:
            self.textBrowser.append(f"Error sending command: {e}")
    
    def select_file(self):
        """Open file dialog to select a file"""
        file_path = fd.askopenfilename()
        if file_path:
            self.textBrowser.append(f"Selected file: {file_path}")
        else:
            self.textBrowser.append("File selection canceled")

    def select_file(self):
        """Open folder dialog to select a folder"""
        _translate = QtCore.QCoreApplication.translate
        self.file_path = fd.askopenfilename()
        if self.file_path:
            self.textBrowser.append(f"Selected file: {self.file_path}")
            self.label_4.setText(_translate("MainWindow", f"Selected file: {self.file_path}"))
        else:
            self.textBrowser.append("File selection canceled")

    def update_firmware(self):
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.textBrowser.append("Error: device not connected")
            return
        self.textBrowser.append(f"Updating firmware... {self.file_path}")
        """Update firmware on ESP32 using esptool"""
        if self.file_path == "":
            self.textBrowser.append("Error: No folder selected")
            return

        self.textBrowser.append("Starting firmware update...")

        for port_info in ports:
            port_name = port_info.device
            self.textBrowser.append(f"Using port: {port_name}")
            check = flash_esp_firmware(
                port=port_name,
                bin_file=self.file_path,
                flash_address=FLASH_ADDRESS,
                progress_callback=lambda percent: self.textBrowser.append(f"Flashing progress: {int(percent)}%")
            )
            if check['success']:
                self.textBrowser.append(f"✓ Firmware flashed successfully to {check['chip']}")
                return
        
        
        else:
            self.textBrowser.append(f"✗ Firmware flashing failed: {check.get('error', 'Unknown error')}")

        self.textBrowser.append(f"Firmware update completed. {check}")

    def closeEvent(self, event):
        """Clean up when window closes"""
        self.disconnect_from_esp32()
        event.accept()

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(847, 573)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # COMUNICATION SECTION

        # Login button - connect to send_login_command
        self.pushButton = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.send_login_command())
        self.pushButton.setGeometry(QtCore.QRect(190, 50, 91, 31))
        self.pushButton.setObjectName("pushButton")
        self.pushButton.setEnabled(False)  # Disabled until connected  
        
        # Console output area
        self.textBrowser = QtWidgets.QTextEdit(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(0, 400, 841, 121))
        self.textBrowser.setObjectName("textBrowser")
        
        # Password input - press Enter to send
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(70, 60, 113, 22))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit.setEchoMode(QtWidgets.QLineEdit.Password)  # Hide password
        self.lineEdit.returnPressed.connect(self.send_login_command)
        
        # Password label
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(10, 60, 71, 21))
        self.label.setObjectName("label")
        
        # Connect button
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.connect_to_esp32())
        self.pushButton_2.setGeometry(QtCore.QRect(10, 10, 171, 31))
        self.pushButton_2.setObjectName("pushButton_2")
        
        # Status label (Connected/Disconnected)
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(190, 20, 91, 16))
        self.label_2.setObjectName("label_2")
        
        # On button - tied to pushButton_3
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.send_on_command())
        self.pushButton_3.setGeometry(QtCore.QRect(10, 100, 91, 28))
        self.pushButton_3.setObjectName("pushButton_3")
        
        # Off button - tied to pushButton_4
        self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.send_off_command())
        self.pushButton_4.setGeometry(QtCore.QRect(110, 100, 93, 28))
        self.pushButton_4.setObjectName("pushButton_4")
        
        # SD card button - tied to pushButton_5
        self.pushButton_5 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.send_sd_command())
        self.pushButton_5.setGeometry(QtCore.QRect(10, 140, 191, 28))
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_5.setEnabled(False)  # Disabled by default until connected

        # Get Time button - tied to checktime
        self.checktime = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.send_gettime_command())
        self.checktime.setGeometry(QtCore.QRect(10, 180, 191, 28))
        self.checktime.setObjectName("checktime")

        # Set Time button - tied to settime
        self.settime = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.send_settime_command())
        self.settime.setGeometry(QtCore.QRect(10, 220, 191, 28))
        self.settime.setObjectName("settime")

        # Disconnect button - tied to pushButton_6
        self.pushButton_6 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.disconnect_from_esp32())
        self.pushButton_6.setGeometry(QtCore.QRect(10, 260, 191, 28))
        self.pushButton_6.setObjectName("pushButton_6")

        #UPDATE SECTION

        # Update label
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(400, 20, 91, 16))
        self.label_3.setObjectName("label_3") 

        # Choose File label
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(400, 45, 450, 16))
        self.label_4.setObjectName("label_3") 

        # Update button - tied to pushButton_7
        self.pushButton_7 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.update_firmware())
        self.pushButton_7.setGeometry(QtCore.QRect(400, 110, 191, 28))
        self.pushButton_7.setObjectName("pushButton_8")

        # Select File button - tied to pushButton_8
        self.pushButton_8 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.select_file())
        self.pushButton_8.setGeometry(QtCore.QRect(400, 70, 191, 28))
        self.pushButton_8.setObjectName("pushButton_8")
        
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 847, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
        # Store reference to MainWindow for closeEvent
        MainWindow.closeEvent = self.closeEvent

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "ESP32 Monitor"))
        self.pushButton.setText(_translate("MainWindow", "Login"))
        self.label.setText(_translate("MainWindow", "Password:"))
        self.pushButton_2.setText(_translate("MainWindow", "Connect"))
        self.label_2.setText(_translate("MainWindow", "Disconnected")) 
        self.pushButton_3.setText(_translate("MainWindow", "On"))
        self.pushButton_4.setText(_translate("MainWindow", "Off"))
        self.pushButton_5.setText(_translate("MainWindow", "SD card"))
        self.pushButton_6.setText(_translate("MainWindow", "Disconnect"))
        self.pushButton_7.setText(_translate("MainWindow", "Update"))
        self.pushButton_8.setText(_translate("MainWindow", "Select File"))
        self.label_3.setText(_translate("MainWindow", "Update Section"))
        self.label_4.setText(_translate("MainWindow", "Select file:"))
        self.checktime.setText(_translate("MainWindow", "Get Time"))
        self.settime.setText(_translate("MainWindow", "Set Time"))


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())