import cv2
from openai import OpenAI
import base64
import time

# set extra_body for thinking control
extra_body = {
    # enable thinking, set to False to disable
    "enable_thinking": True,
    # use thinking_budget to contorl num of tokens used for thinking
    # "thinking_budget": 4096
}


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
        prompt = str(input("Enter prompt: "))
        if prompt == 'exit':
            break
        response = client.chat.completions.create(
            model='Qwen/Qwen3-235B-A22B',  # ModelScope Model-Id
            messages=[
                {
                'role': 'user',
                'content': prompt
                }
            ],
            stream=True,
            extra_body=extra_body
        )
        done_thinking = False
        for chunk in response:
            thinking_chunk = chunk.choices[0].delta.reasoning_content
            answer_chunk = chunk.choices[0].delta.content
            if thinking_chunk != '':
                print(thinking_chunk, end='', flush=True)
            elif answer_chunk != '':
                if not done_thinking:
                    print('\n\n === Final Answer ===\n')
                    done_thinking = True
                print(answer_chunk, end='', flush=True)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    cap.release()
    cv2.destroyAllWindows()

