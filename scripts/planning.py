import cv2
from openai import OpenAI
import base64
import time
from utils import extracts

# Set your API key
client = OpenAI(
    base_url='https://api-inference.modelscope.cn/v1/',
    api_key='e4306532-28ae-4854-894c-6a27f7ffa44c', # ModelScope Token
)

def qwen_planner(instruction, objects: list, locations: list, image_path_1='scene1.jpg', image_path_2='scene2.jpg') -> str:
    """
    Generate a Sequence of Preset Actions
    :param instruction: <str>
    :param image: <str>, file name of image
    :param objects: <list>, list of objects
    :param locations: <list>, list of locations
    :return: <str>
    """
    # Preset Actions
    action_pool = []
    for object in objects:
        action_pool.append("grasp {} from side".format(object))
        action_pool.append("grasp {} from top".format(object))
        
    for location in locations:
        action_pool.append("place to {} vertically".format(location))
        action_pool.append("place to {} horizontally".format(location))

    # Prompt Template
    planning_prompt = """
    INSTRUCTION: Considering the given scene, you need to refer to the action list to generate a plan of steps to accomplish the task.
    
    Task: {}
    
    Action List: {}
    
    Guideline:
    1. Your thoughts should be conditioned on the given scene (image) and fully considering the physical constraints (e.g., relative positions of objects) to include necessary intermediate steps.
    2. You should firstly decompose the Task into subtasks in a step-by-step way and then generate the plan.
    3. Each step of your plan should include exactly one item from the Action List.
    4. Each "grasp" action should be followed by a "place" action.
    5. If I ask you to place things to box, you should place things to box.
    6. Finally, output your plan as a list in this format:
    <plan>[Action 1, Action 2, Action 3, ...]</plan>
    """
    # Process Image
    frame = cv2.imread(image_path_1)
    _, buffer = cv2.imencode(image_path_1[image_path_1.find("."):], frame)
    image_base64_1 = base64.b64encode(buffer).decode('utf-8')

    frame = cv2.imread(image_path_2)
    _, buffer = cv2.imencode(image_path_2[image_path_2.find("."):], frame)
    image_base64_2 = base64.b64encode(buffer).decode('utf-8')
    # Call LLM
    response = client.chat.completions.create(
        model="Qwen/Qwen2.5-VL-72B-Instruct",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": planning_prompt.format(instruction, action_pool)},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{image_base64_1}"
                        }
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{image_base64_2}"
                        }
                    }
                ]
            }
        ],
        stream=True
    )
    result = ''
    for chunk in response:
        print(chunk.choices[0].delta.content, end='', flush=True)
        if chunk.choices[0].delta.content:
            result += chunk.choices[0].delta.content
    return result


if __name__ == "__main__":
    # ### Test 1
    # instruction = "Give me two fruits after giving me a coke."
    # image_path = "./tests/banana_apple_coke.png"
    # objects = ["banana", "coke", "apple"]
    # locations = [*objects, "user"]
    # plan = planner(instruction, image_path, objects, locations)
    # print(plan)

    ### Test 2
    instruction = "Exchange position of the apple and the banana."
    image_path = "./tests/banana_apple_coke.png"
    objects = ["banana", "coke", "apple"]
    locations = [*objects, "user"]
    plan = planner(instruction, image_path, objects, locations)
    print(plan)

