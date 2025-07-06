# window that connects to the remote server and sends commands to the screen session

import sys
import paramiko
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                            QHBoxLayout, QPushButton, QTextEdit, QLineEdit,
                            QLabel, QMessageBox)
from PyQt6.QtCore import QThread, pyqtSignal
from scripts.utils import cap_content, cap_screen
from PIL import Image

class SSHWorker(QThread):
    """
    initialize the ssh connection, create session, and start off model
    """
    output_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, session, command):
        super().__init__()
        self.session = session
        self.command = command
        self.connect()

    def connect(self):
        self.output_received.emit("Connecting to remote......")
        from scripts.utils import find_ssh
        try: 
            self.ssh = find_ssh()
            self.output_received.emit("Connected.")
        except Exception as e:
            self.error_occurred.emit(str(e))
    
    def disconnect(self):
        try:
            if self.ssh:
                self.ssh.close()
                self.ssh = None
        except Exception as e:
            self.error_occurred.emit(str(e))

    def run(self):
        if self.ssh is None:
            self.connect()
        # Create a screen session and send the command
        # stdin, stdout, stderr = ssh.exec_command(f'screen -S lh -X stuff "{self.command}\n"')
        # Attach to the screen session and send the command

        # command = command_processor.encode(self.command, Image.open('captured.png'))
        # stdin, stdout, stderr = self.ssh.exec_command(f'cd /home/lh; {self.command}\n')
        
        # Capture the output by attaching to the screen session
        stdin, stdout, stderr = cap_screen(self.ssh, self.session)
        
        # Read the output
        output = stdout.read().decode()
        error = stderr.read().decode()
        
        if output:
            self.output_received.emit(output)
        if error:
            self.error_occurred.emit(error)

        self.disconnect()

# TODO: create a class that send commands and listen
class Command(QThread):
        def __init__(self, ssh, session, command):
            self.ssh = ssh
            self.session = session
            self.command = command

        def run(self):
            self.ssh.exec_command(f'screen -S {self.session} -X stuff "{self.command}\n"')
        

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Remote Screen Control")
        self.setGeometry(100, 100, 800, 600)
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Server connection details
        # connection_layout = QHBoxLayout()
        # self.username_input = QLineEdit()
        # self.username_input.setPlaceholderText("Username")
        
        # connection_layout.addWidget(QLabel("Username:"))
        # connection_layout.addWidget(self.username_input)
        
        # self.screen_session = QLineEdit()
        # self.screen_session.setPlaceholderText("Enter the screen session to send command to")
        # TODO: add a "Connect" button
        # Send button
        self.send_button = QPushButton("Connect")
        self.send_button.clicked.connect(self.initialize_ssh)
        
        # Command input
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Enter command to send to screen session")
        
        # Output display
        self.output_display = QTextEdit()
        self.output_display.setReadOnly(True)
        
        # Send button
        self.send_button = QPushButton("Send Command")
        self.send_button.clicked.connect(self.send_command)
        
        # Add widgets to main layout
        # layout.addLayout(connection_layout)
        # layout.addWidget(self.screen_session)
        layout.addWidget(self.command_input)
        layout.addWidget(self.send_button)
        layout.addWidget(self.output_display)
        
        # Initialize SSH worker
        self.ssh_worker = None

        # TODO: display video stream and marked video stream
        # TODO: display VLM description

    # TODO: function that cap video stream as images
    # TODO: test max freq

    def initialize_ssh(self):
        session = self.screen_session.text()
        command = self.command_input.text()
        
        if not all([session, command]):
            QMessageBox.warning(self, "Error", "Please fill in all fields")
            return
        
        # Create and start SSH worker
        self.ssh_worker = SSHWorker(session, command)
        self.ssh_worker.output_received.connect(self.handle_output)
        self.ssh_worker.error_occurred.connect(self.handle_error)
        self.ssh_worker.start()
        
        # Disable button while command is being sent
        self.send_button.setEnabled(False)
        self.ssh_worker.finished.connect(lambda: self.send_button.setEnabled(True))

        # build processor (tokenizer)
        from scripts.utils import Processor
        self.command_processor = Processor()

    def send_command(self):
        # TODO: connect to the sending command class
        pass

    def handle_output(self, output):
        self.output_display.append(f"Output:\n{output}")

    def handle_error(self, error):
        self.output_display.append(f"Error:\n{error}")

    def handle_ecot_output(self, output):
        output = self.command_processor.decode(output)
        action = cap_content(output, '<action>', '</action>')
        generated = cap_content(output, '<generated_ids>', '</generated>')
        self.output_display.append(f"Generated:\n{cap_content(output, '<generated_ids>', '<generated')}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec()) 