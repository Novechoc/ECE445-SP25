# Remote Screen Control Application

This application provides a GUI interface to send commands to a remote screen session and display the output locally.

## Prerequisites

- Python 3.6 or higher
- Screen must be installed on the remote server
- SSH access to the remote server

## Installation

1. Install the required dependencies:
```bash
pip install -r requirements.txt
```

## Usage

1. Run the application:
```bash
python screen_remote_control.py
```

2. Fill in the connection details:
   - Hostname: The IP address or hostname of the remote server
   - Username: Your SSH username
   - Password: Your SSH password

3. Enter the command you want to send to the screen session in the command input field

4. Click the "Send Command" button to execute the command

## Notes

- The application assumes there is a screen session named "remote_session" on the remote server
- If the screen session doesn't exist, you'll need to create it first using:
  ```bash
  screen -S remote_session
  ```
- The application will display both the output and any errors in the text area below
- The send button will be disabled while a command is being executed to prevent multiple simultaneous commands 