import time, requests, paramiko, subprocess

import httpx
import logging
from tenacity import retry, stop_after_attempt, wait_fixed, retry_if_exception_type

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SSHConnection:
    def __init__(self, hostname='10.105.100.216'):
        """
        start a ssh session with ssh key at '~/.ssh/id_zju'
        """
        self.connected = False
        self.ssh = None
        self.hostname = hostname
        # self.user='zju'
        # self.key_filename='~/.ssh/id_zju'
        
        # Create SSH client
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.screen_sessions = []
        
        # Connect to the server
        try:
            self.ssh.connect(hostname, username='zju', key_filename='/home/ece445/.ssh/id_zju')
            self.connected = True
            print(f"Connected to {hostname}")
        except paramiko.SSHException as e:
            print(f"Failed to connect to {hostname}: {e}")
            
    def __del__(self):
        """
        Close the SSH connection when the object is deleted.
        """
        # for session in self.screen_sessions:
        #     self.del_screen_session(session)

        if self.connected:
            self.ssh.close()
            print(f"Disconnected from {self.hostname}")
        
    def start_screen_session(self, session_name):
        """
        Start a screen session on the remote server.
        Sessions started with this method will be deleted when the object is deleted.
        """
        try:
            # Check if the session already exists
            stdin, stdout, stderr = self.ssh.exec_command(f'screen -list | grep {session_name}')
            if stdout.channel.recv_exit_status() == 0:
                print(f"Session {session_name} already exists.")
                return 1
            
            # Start a new screen session
            self.ssh.exec_command(f'screen -dmS {session_name}')
            print(f"Started screen session: {session_name}")
            self.screen_sessions.append(session_name)
            return 0
        except Exception as e:
            print(f"Error starting screen session: {e}")

    def exec_in_screen_session(self, session_name, command):
        """
        Execute a command in the specified screen session.
        """
        try:
            # Send the command to the screen session
            self.ssh.exec_command(f'screen -S {session_name} -p 0 -X stuff "{command}\n"')
            print(f"Executed command in session {session_name}: {command}")
        except Exception as e:
            print(f"Error executing command in screen session: {e}")

    def del_screen_session(self, session_name):
        """
        Delete the specified screen session.
        """
        try:
            stdin, stdout, stderr = self.ssh.exec_command(f'screen -list | grep {session_name}')
            if stdout.channel.recv_exit_status() != 0:
                print(f"Session {session_name} doesn't exist.")
                return 1
            
            # Send the command to delete the screen session
            self.ssh.exec_command(f'screen -S {session_name} -X quit')
            print(f"Deleted screen session: {session_name}")
            if session_name in self.screen_sessions:
                self.screen_sessions.remove(session_name)
            return 0
        except Exception as e:
            print(f"Error deleting screen session: {e}")
    
class Poster:
    def __init__(self, app='gsam', port=7890, hostname='10.105.100.230'):
        '''
        Create a screen session named <app> on the server, 
        run the pre_command in the screen session, 
        run the app,
        and map the local port to the server port.
        '''
        self.hostname = hostname
        self.port = port
        self.app = app
        self.url = f"http://localhost:{port}/{app}/"
        # Start SSH tunnel
        self.ssh = SSHConnection(hostname)
        if not self.ssh.connected:
            raise Exception(f"Failed to connect to {hostname}")
    
    def set_up(self):
        self._hang_app_in_screen(self.app)
        self._map_port()
        
    def _hang_app_in_screen(self, session_name='gsam', pre_command='cd /home/lh; source start.sh; conda activate gsam'):
        self.ssh.start_screen_session(session_name)
        # pre_command = pre_command if pre_command else self.pre_command
        return self.ssh.exec_in_screen_session(session_name, f"{pre_command}; uvicorn {self.app}:app --host 127.0.0.1 --port {self.port}")
    
    def _map_port(self):
        '''
        Map the port to the server
        '''
        ssh_cmd = [
            "ssh",
            "-i", "~/.ssh/id_zju",
            "-L", f"{self.port}:localhost:{self.port}",
            f"zju@{self.hostname}"
        ]
        try:
            # Start the SSH tunnel in the background
            self.ssh_proc = subprocess.Popen(
                ssh_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
                # close_fds=True
            )
            print(f"Started SSH Forwarding at port {self.port}")
        except Exception as e:
            print(f"Failed to start SSH tunnel: {e}")
        
    def __del__(self):
        '''
        Close the SSH tunnel when the object is deleted.
        '''
        # # It seems that self.ssh is recycled before the following call
        # self.ssh.del_screen_session(self.app)

        if hasattr(self, 'ssh_proc'):
            self.ssh_proc.terminate()
            print(f"Closed SSH tunnel to {self.hostname}")

    def ready(self):
        try:
            response = requests.get(self.url)
            return True
        except requests.exceptions.ConnectionError:
            return False

    def post(self, labels, image_path='captured.jpg'):
        '''
        Post an image and text to the server.
        text for Dino: a string of comma-separated labels e.g "milk, bottle, bowl, pen"
        '''
        # Wait until the app at the URL is ready
        print("Waiting for server before posting...")
        # while not self.ready():
        #     time.sleep(0.1)
        with open(image_path, "rb") as f:
            files = {"image": ("image.jpg", f, "image/jpeg")}
            data = {"labels": labels}
            print(f"Sent request to {self.url} with labels: {labels}")
            response = requests.post(self.url, files=files, data=data)
        return response.json()
