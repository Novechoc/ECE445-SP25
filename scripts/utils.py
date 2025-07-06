import cv2, os, time, threading
import numpy as np

def visualize_boxes(image_path, detections):
    image = cv2.imread(image_path)
    # print(f"Image shape: {image.shape}")
    for detection in detections:
        label, confidence, box = detection['label'], detection['score'], detection['box']
        x, y, xe, ye = [int(i) for i in box]
        cv2.rectangle(image, (x, y), (xe, ye), (0, 255, 0), 2)
        cv2.putText(image, f"{label} ({confidence:.2f})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.imwrite('output.jpg', image)

def extracts(text, start_marker, end_marker):
    """
    Captures the content of a text wrapped by the last pair of input markers.

    Args:
        text (str): The input text.
        start_marker (str): The starting marker.
        end_marker (str): The ending marker.

    Returns:
        str: The content between the last pair of markers, or None if not found.
    """
    start_index = text.rfind(start_marker)
    end_index = text.rfind(end_marker)

    if start_index != -1 and end_index != -1 and start_index < end_index:
        return text[start_index + len(start_marker):end_index].strip()
    return None


def cap_remote_screen(ssh, session):
    """
    Captures the content of a screen session through a ssh connection
    """
    import paramiko
    assert type(ssh) == paramiko.SSHClient
    assert ssh.get_transport() is not None
    assert ssh.get_transport().is_active()
    assert ssh.get_transport().is_authenticated()

    stdin, stdout, stderr = ssh.exec_command(f'screen -S {session} -X hardcopy -h /tmp/screenlog && cat /tmp/screenlog')
    return stdin, stdout, stderr


class Protocal:
    def __init__(self):
        return self
    
    def encode(self, text, protocal_class: str):
        """
        Wraps the text with the specified protocol class markers.

        Args:
            text (str): The input text.
            protocal_class (str): The protocol class to wrap the text with.

        Returns:
            str: The wrapped text.
        """
        return f'[{protocal_class}]{text}[/{protocal_class}]'
    
    def decode(self, text, protocal_class: str):
        """
        Extracts the content of a text wrapped by the last pair of input markers.
        """
        return extracts(text, f'[{protocal_class}]', f'[/{protocal_class}]')


class Processor:
    '''
    inputs processor (tokenizer + feature extractor) for huggingface models
    '''
    def __init__(self, 
                 path="Embodied-CoT/ecot-openvla-7b-bridge",
                 cache_dir='./hf-cache'):
        from transformers import AutoProcessor
        self.processor = AutoProcessor.from_pretrained(path, 
                                                       trust_remote_code=True,
                                                       cache_dir=cache_dir)

    def encode(self, prompt, image):
        return self.processor(prompt, image, return_tensors="pt")
    
    def decode(self, generated_ids):
        return self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
    

class Camera:
    def __init__(self, 
                 device=0, 
                #  width=640, 
                #  height=480, 
                 max_fps=5,
                 save_dir = './'):
        self.device = device
        # self.width = width
        # self.height = height
        self.fps = max_fps
        self.save_dir = save_dir
        self.cap = cv2.VideoCapture(device)
        if self.get_camera_status():
            print(f"Camera {device} connected successfully.")
        else:
            print(f"Failed to connect to camera {device}.")
            raise RuntimeError(f"Failed to connect to camera {device}.")
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.on = False
        
    def get_status(self):
        '''
        check if the camera is connected (opened)
        '''
        return self.cap.isOpened()
    
    def read(self):
        '''
        Returns:
        ret, frame (cv2)
        '''
        assert self.get_status(), "Camera not found"
        return self.cap.read()
    
    def turn_on(self):
        '''
        write the captured picture in the specified file with certain max freq (interval)
        '''
        self.on = True
        def capture_loop():
            while self.on:
                ret, frame = self.cap.read()
                if ret:
                    cv2.imwrite(os.path.join(self.save_dir, f"captured_{self.device}"), frame)
                    time.sleep(1/self.fps)
                else:
                    print("Failed to capture image.")
                    break

        self.capture_thread = threading.Thread(target=capture_loop, daemon=True)
        self.capture_thread.start()

    def turn_off(self):
        '''
        turn off the camera
        '''
        self.on = False
        if self.capture_thread.is_alive():
            self.capture_thread.join()
        self.cap.release()
        cv2.destroyAllWindows()
        print(f"Camera {self.device} released.")