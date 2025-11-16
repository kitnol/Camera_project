from PyQt5 import QtCore, QtGui, QtWidgets
import serial.tools.list_ports
import serial
from esp_idf_monitor.idf_monitor import SerialMonitor
import queue
from esp_idf_monitor.base.constants import TAG_SERIAL


def connect(port: str, baudrate: int = 115200, timeout: int = 1) -> serial.Serial:
    serial_instance = serial.serial_for_url(
        port, 
        baudrate, 
        do_not_open=True, 
        exclusive=True
    )

    monitor = SerialMonitor(
        serial_instance=serial_instance,
        elf_files=[],  # Empty if no ELF file
        print_filter='*',
        make='make',
        encrypted=False,
        reset=True,
        open_port_attempts=1,
        toolchain_prefix='xtensa-esp32-elf-',
        eol='CRLF',
        decode_coredumps='info',
        decode_panic='backtrace',
        target='esp32',
        websocket_client=None,
        enable_address_decoding=False,  # Set to False if no ELF
        timestamps=False,
        timestamp_format='',
        force_color=False,
        disable_auto_color=False,
        rom_elf_file=None
    )

    return monitor


class SerialReaderThread(QtCore.QThread):
    """Thread to read serial data without blocking GUI"""
    line_received = QtCore.pyqtSignal(str)
    
    def __init__(self, monitor):
        super().__init__()
        self.monitor = monitor
        self.running = True
        self.line_buffer = ""
        
    def run(self):
        """Read lines from monitor in background thread"""
        self.monitor.console_reader.start()
        self.monitor.serial_reader.start()
        
        while self.running and self.monitor.serial_reader.alive:
            try:
                event_tag, data = self.monitor.event_queue.get(timeout=0.5)
                
                if event_tag == TAG_SERIAL:
                    chunk = data.decode('utf-8', errors='ignore')
                    self.line_buffer += chunk
                    
                    # Process complete lines
                    while '\n' in self.line_buffer:
                        line, self.line_buffer = self.line_buffer.split('\n', 1)
                        line = line.rstrip('\r')
                        
                        if line:
                            self.line_received.emit(line)
                            
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error in serial reader: {e}")
                break
    
    def stop(self):
        """Stop the thread"""
        self.running = False
        if hasattr(self, 'monitor'):
            try:
                self.monitor.console_reader.stop()
                self.monitor.serial_reader.stop()
            except:
                pass


class Ui_MainWindow(object):
    def __init__(self):
        self.monitor = None
        self.reader_thread = None
    
    def changeText(self, text):
        self.label_2.setText(text)

    def connect_to_esp32(self, baudrate=115200):
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            self.label_2.setText("No ports found")
            return
        
        for port_info in ports:
            try:
                self.monitor = connect(port_info.device, baudrate)
                self.label_2.setText(f"{port_info.device}")
                self.textBrowser.append(f"Connected to {port_info.device}")
                
                # Start reading in background thread
                self.start_reading()
                return
                
            except Exception as e:
                self.textBrowser.append(f"Failed on {port_info.device}: {e}")
                continue
        
        self.label_2.setText("Failed: No port available")

    def start_reading(self):
        """Start the serial reader thread"""
        if self.reader_thread is not None:
            self.reader_thread.stop()
            self.reader_thread.wait()
        
        self.reader_thread = SerialReaderThread(self.monitor)
        self.reader_thread.line_received.connect(self.on_line_received)
        self.reader_thread.start()
    
    def on_line_received(self, line):
        """Handle received line (runs in GUI thread)"""
        self.textBrowser.append(f" {line}")
    
    def send_login_command(self):
        """Send login command with password to ESP32"""
        if self.monitor is None:
            self.textBrowser.append("Error: Not connected to ESP32")
            return
        
        password = self.lineEdit.text()
        
        if not password:
            self.textBrowser.append("Error: Password field is empty")
            return
        
        command = f"login {password}\n"
        
        try:
            # Write to serial
            self.monitor.serial_write(command.encode('utf-8'))
            self.textBrowser.append(f"Sent: {command.strip()}")
            
            # Optional: Clear the password field after sending
            # self.lineEdit.clear()
            
        except Exception as e:
            self.textBrowser.append(f"Error sending command: {e}")
    
    def closeEvent(self, event):
        """Clean up when window closes"""
        if self.reader_thread is not None:
            self.reader_thread.stop()
            self.reader_thread.wait()
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
        
        self.textBrowser = QtWidgets.QTextEdit(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(0, 400, 841, 121))
        self.textBrowser.setObjectName("textBrowser")
        
        # Password input - press Enter to send
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(70, 60, 113, 22))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit.returnPressed.connect(self.send_login_command)  # Enter key sends command
        
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(10, 60, 71, 21))
        self.label.setObjectName("label")
        
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget, clicked=lambda: self.connect_to_esp32())
        self.pushButton_2.setGeometry(QtCore.QRect(10, 10, 171, 31))
        self.pushButton_2.setObjectName("pushButton_2")
        
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(190, 20, 91, 16))
        self.label_2.setObjectName("label_2")
        
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QtCore.QRect(10, 100, 91, 28))
        self.pushButton_3.setObjectName("pushButton_3")
        
        self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_4.setGeometry(QtCore.QRect(110, 100, 93, 28))
        self.pushButton_4.setObjectName("pushButton_4")
        
        self.pushButton_5 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_5.setGeometry(QtCore.QRect(10, 140, 191, 28))
        self.pushButton_5.setObjectName("pushButton_5")
        
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
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "Login"))
        self.label.setText(_translate("MainWindow", "Password:"))
        self.pushButton_2.setText(_translate("MainWindow", "Connect"))
        self.label_2.setText(_translate("MainWindow", "Waiting..."))
        self.pushButton_3.setText(_translate("MainWindow", "On"))
        self.pushButton_4.setText(_translate("MainWindow", "Off"))
        self.pushButton_5.setText(_translate("MainWindow", "SD card"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())