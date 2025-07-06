import cv2
from openai import OpenAI
import base64
import time

# Set your API key
client = OpenAI(
    base_url='https://api-inference.modelscope.cn/v1/',
    api_key='e4306532-28ae-4854-894c-6a27f7ffa44c', # ModelScope Token
)
# Initialize camera
cap = cv2.VideoCapture(0)  # 0 is default webcam
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Set buffer size to 1 to reduce latency
cap.set(cv2.CAP_PROP_FPS, 1) # Set FPS to 1 to further reduce latency

if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

try:
    while True:
        # # Capture frame
        # ret, frame = cap.read()
        # if not ret:
        #     print("Failed to capture frame")
        #     continue

        # cv2.imshow("Captured", frame)
        # # if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
        # #     break
        
        # # cv2.imshow("Captured", frame)
        # while True:
        #     if cv2.waitKey(1) & 0xFF == ord('c'):  # Press 'c' to continue
        #         break

        # # Encode frame to JPEG
        # _, buffer = cv2.imencode('.jpg', frame)
        buffer = cv2.imread('captured.jpg')
        image_base64 = base64.b64encode(buffer).decode('utf-8')

        '''
        Prompts:
        1. 32B: Is the person in the image looking into the camera? Answer yes or no only"}
        2. 
        '''

        # Call OpenAI API
        print("\nSending image to Qwen2.5-VL-3B-Instruct...")
        response = client.chat.completions.create(
            model="Qwen/Qwen2.5-VL-32B-Instruct",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "Identify the objects in the image and output the list in this format:\n <objects>[object1, object2, ...]</objects>"},
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

        print(response.choices[0].message.content)

        # Wait before capturing the next image
        time.sleep(1) # seconds

except KeyboardInterrupt:
    print("Stopping...")

finally:
    cap.release()
    cv2.destroyAllWindows()

