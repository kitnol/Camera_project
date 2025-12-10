from PyQt5 import QtCore, QtGui, QtWidgets
import serial.tools.list_ports
import serial
import queue


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
        """Send login command with password to ESP32"""
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
    
    def closeEvent(self, event):
        """Clean up when window closes"""
        self.disconnect_from_esp32()
        event.accept()

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(847, 573)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # Login button - connect to send_login_command
        self.pushButton = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.send_login_command())
        self.pushButton.setGeometry(QtCore.QRect(190, 50, 91, 31))
        self.pushButton.setObjectName("pushButton")
        self.pushButton.setEnabled(False)  # Disabled until connected
        
        self.textBrowser = QtWidgets.QTextEdit(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(0, 400, 841, 121))
        self.textBrowser.setObjectName("textBrowser")
        
        # Password input - press Enter to send
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(70, 60, 113, 22))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit.setEchoMode(QtWidgets.QLineEdit.Password)  # Hide password
        self.lineEdit.returnPressed.connect(self.send_login_command)
        
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(10, 60, 71, 21))
        self.label.setObjectName("label")
        
        # Connect button
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.connect_to_esp32())
        self.pushButton_2.setGeometry(QtCore.QRect(10, 10, 171, 31))
        self.pushButton_2.setObjectName("pushButton_2")
        
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(190, 20, 91, 16))
        self.label_2.setObjectName("label_2")
        
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.send_on_command())
        self.pushButton_3.setGeometry(QtCore.QRect(10, 100, 91, 28))
        self.pushButton_3.setObjectName("pushButton_3")
        
        self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.send_off_command())
        self.pushButton_4.setGeometry(QtCore.QRect(110, 100, 93, 28))
        self.pushButton_4.setObjectName("pushButton_4")
        
        # Disconnect button - tied to pushButton_5
        self.pushButton_5 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.send_sd_command())
        self.pushButton_5.setGeometry(QtCore.QRect(10, 140, 191, 28))
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_5.setEnabled(False)  # Disabled by default until connected

        self.pushButton_6 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.disconnect_from_esp32())
        self.pushButton_6.setGeometry(QtCore.QRect(10, 180, 191, 28))
        self.pushButton_6.setObjectName("pushButton_6")
        
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



if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())