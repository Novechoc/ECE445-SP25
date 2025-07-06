import time, base64, cv2
import numpy as np
from openai import OpenAI
from utils import extracts
from connection import Poster
from typing import List
# from csi_camera import capture_image
# from tests.test_poster import visualize_boxes_and_masks

def name_objects(image_path='captured.jpg'):
    answer = ask_qwen(
        "Identify the objects on the table that you can clearly identify the name of, and output the list in the following format:\n <objects>object1, object2, ...</objects>", 
        image_path
        )
    return extracts(answer, '<objects>', '</objects>')

def look_for_objects(objects, image_path='captured.jpg'):
    return ask_qwen(
        f'Please look carefully in the image and answer is there {objects} on the table? Answer "yes" or "no" only.',
        image_path
    )

def ask_qwen(prompt, image_path, model_size='32B') -> list[str]:
        assert model_size in ['3B', '7B', '32B', '72B'], "Invalid model size. Choose from ['3B', '7B', '32B', '72B']"
        client = OpenAI(
            base_url='https://api-inference.modelscope.cn/v1/',
            api_key='e4306532-28ae-4854-894c-6a27f7ffa44c', # ModelScope Token
        )
        frame = cv2.imread(image_path)
        _, buffer = cv2.imencode(image_path[image_path.find("."):], frame)
        image_base64 = base64.b64encode(buffer).decode('utf-8')
        # "Identify the objects in the image and output the list in this format:\n <objects>[object1, object2, ...]</objects>"

        # Call OpenAI API
        print(f"Sending image to Qwen2.5-VL-{model_size}-Instruct...")
        response = client.chat.completions.create(
            model=f"Qwen/Qwen2.5-VL-{model_size}-Instruct",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_base64}"
                            }
                        }
                    ]
                }
            ],
            stream=False
        )
        # print(response.choices[0].message.content)
        return response.choices[0].message.content
"""
class Perceptor:
    def __init__(self, size='7B', port='7890', return_when_ready=True):
        # For Grounded SAM (object_masks)
        self.poster = Poster(app='gsam', port=port, 
                             hostname='10.105.100.230',
                             pre_command='cd /home/lh/; source start.sh; conda activate gsam',
                             auto_setup=True)

        if return_when_ready:
            print("Waiting for the server to be ready...")
            while not self.poster.ready():
                time.sleep(0.5)
            print("Server is ready.")

        self.detections = {}

    def mask_objects(self, labels: str, image_path='captured.jpg', save=False):
        '''
        returns:
        boxes: list of boxes
        masks: list of masks
        '''
        self.detections = self.poster.post(image_path, labels)
        if save:
            visualize_boxes_and_masks(image_path, self.detections['boxes'], self.detections['masks'])
        return self.detections
    
    def get_mask_orientation(self, mask): # image_path='captured.jpg', save=False):
        # angles = []
        # image = cv2.imread(image_path)
        # for mask in masks:
        mask = np.array(mask).astype('uint8') * 255
        contour, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return cv2.fitEllipse(contour)

        # if save:
        #     for angle in angles:
        #         cv2.ellipse(image, angle, (0, 255, 0), 2)
        #     cv2.imwrite('output_with_angles.jpg', image)

        # return angles
"""