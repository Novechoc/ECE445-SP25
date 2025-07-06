import sys, time, re, os
import numpy as np

from csi_camera import capture_image, locate_and_visualize, top_to_real
from perception import name_objects
from planning import qwen_planner
from utils import extracts
from connection import Poster
from UR3e import UR3e
from kinematics import OFFSET_CAM_H
# from utils import visualize_boxes_and_masks

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


PI = np.pi
H = 3280/2
V = 2464/2
from kinematics import OFFSET_CAM_H

# if not rclpy.ok():
#     rclpy.init()
# node_grasp = Node('grasp_client')
# client = node_grasp.create_client(SetBool, '/grasp_object')
# if not client.wait_for_service(timeout_sec=5.0):
#     node_grasp.get_logger().error('Service /grasp_object not available!')
#     sys.exit(1)
# node_release = Node('place_client')
# client = node_release.create_client(SetBool, '/release_object')
# if not client.wait_for_service(timeout_sec=5.0):
#     node_release.get_logger().error('Service /release_object not available!')
#     sys.exit(1)

def find_objects(labels, scene):
    assert scene in [0, 1]
    return ', '.join([k for k, v in labels.items() if v == scene])

def approach_center(obj):
    while True:
        # label_string = ', '.join([i for i in labels.keys()])

        # Grounded SAM
        detections = poster.post(obj)
        # detections = poster.post('green sponge')
        if 'error' in detections.keys():
            print(f"Error: {detections['error']}")
            sys.exit(1)

        if not obj in [i for i in detections.keys()]:
            continue # detect again

        extended_detections = locate_and_visualize(detections)

        id = extended_detections[obj]['confidence'].index(max(extended_detections[obj]['confidence']))
        x, y = extended_detections[obj]['centers'][id]
        ellipse = extended_detections[obj]['ellipses'][id]

        # Identification for centering
        if int(np.abs(x-H/2)) < 100 and int(np.abs(y-V/2)) < 80 and ur.gripper_loc()[-1] < 0.1:
            break

        # move to the object
        diff_x, diff_y = top_to_real(x, y, ur.camera_loc()[-1], ur.gripper_angle(), H, V)
        goal_loc = ur.gripper_loc()
        cam_angle = ur.gripper_angle()
        goal_loc[0] -= diff_x
        goal_loc[1] -= diff_y
        goal_loc[2] = 0.05

        ur.move_to(goal_loc, gripper_angle=cam_angle+ellipse[-1], duration=3)
        capture_image()
        time.sleep(1)

def find_center(obj):
    # Grounded SAM
    detections = poster.post(obj)
    # detections = poster.post('green sponge')
    if 'error' in detections.keys():
        print(f"Error: {detections['error']}")
        sys.exit(1)

    if not obj in [i for i in detections.keys()]:
        return 1

    extended_detections = locate_and_visualize(detections)

    x, y = extended_detections[obj]['centers'][0]
    ellipse = extended_detections[obj]['ellipses'][0]

    # move to the object
    diff_x, diff_y = top_to_real(x, y, ur.camera_loc()[-1], ur.gripper_angle(), H, V)
    ur.move_by_diff([-diff_x, -diff_y, 0])
    return ur.gripper_loc()

def release():
    # req = SetBool.Request()
    # req.data = True
    # future = client.call_async(req)
    # rclpy.spin_until_future_complete(node_grasp, future)
    # print('complete')
    # res = future.result()
    # if res and res.success:
    #     print(f"Grasp succeeded: {res.message}")
    #     return 0
    # else:
    #     print(f"Grasp failed: {res.message if res else 'no response'}")
    #     return 1
    print("Releasing:")
    os.system('ros2 service call /release_object std_srvs/srv/SetBool "{data: true}"')
    
def grasp():
    # req = SetBool.Request()
    # req.data = True
    # future = client.call_async(req)
    # rclpy.spin_until_future_complete(node_release, future)
    # res = future.result()
    # if res and res.success:
    #     print(f"Grasp succeeded: {res.message}")
    #     return 0
    # else:
    #     print(f"Grasp failed: {res.message if res else 'no response'}")
    #     return 1
    print('Grasping:')
    os.system('ros2 service call /grasp_object std_srvs/srv/SetBool "{data: true}"')
    
if __name__=='__main__':
        
    # Init
    ur = UR3e()
    poster = Poster()

    # Input
    instruction = str(input("Greetings. How can I help you?\n"))

    # Name objects
    labels = {} # labels as keys and 0, 1 ,2 as values
    scenes = ['left', 'front', 'right']
    release()
    for i in [1, 2]:
        ur.home(i, duration=3)
        capture_image(save_path=f'scene{i}.jpg')
        for label in [i.strip() for i in name_objects(f'scene{i}.jpg').split(',')]:
            # filtering
            label = label.lower()
            if not label in labels and not 'wire' in label and not 'metal' in label and not 'cable' in label and not 'speaker' in label and not 'motor' in label and not 'electronic' in label and not 'connector' in label and 'object' not in label:
                labels[label] = i
        print(f'Found objects on the {scenes[i]}:', [k for k, v in labels.items() if v == i])
    locations = labels.copy()
    locations['usr'] = 1
    locations['temporary place'] = 1
    label_list = [i for i in labels.keys()]
    location_list = [i for i in locations.keys()]
    input(f"Please confirm the found objects and their locations:{labels}")

    # Planning
    while True:
        print("Wait a moment... Let me think of a plan...")
        response = qwen_planner(instruction, label_list, location_list)
        # print(response)
        plan = eval(extracts(response, '<plan>', '</plan>'))
        print(plan)

        flag = 0
        for i, p in enumerate(plan):
            if (not (i%2) and (not 'grasp' in p or 'place' in p)) or (i%2 and (not 'place' in p or 'grasp' in p)):
                flag = 1
                break

        a = input("Are you satisfied with my plan? Enter r to regenerate...\n")
        if 'r' in a:
            flag = 1

        if flag == 0 and not len(plan)%2:
            break
        print('Plan invalid! Give me another chance :(')

    '''
    1. find the target location
    2. grasp the stuff
    3. back to home
    4. place to location
    5. back to home

    6. fine next target location
    ...
    '''

    for plan_id in range(0, len(plan), 2):
        # find the target location
        object = re.findall(r'grasp ([()A-Za-z ]+) from', plan[plan_id])[0]
        target = re.findall(r'place to ([()A-Za-z ]+) [a-z]+$', plan[plan_id+1])[0]

        # locate target
        if target == 'usr':
            target_loc = [0, -0.3, 0.1]
        elif target == 'temporary place':
            target_loc = [0.25, -0.15, 0.1]
        else:
            ur.home(labels[target])
            capture_image()
            target_loc = find_center(target)

        # grasp object
        ur.home(labels[object])
        capture_image()
        approach_center(object)
        ur.move_by_diff([-OFFSET_CAM_H*np.sin(ur.gripper_angle()*PI/180), -OFFSET_CAM_H*np.cos(ur.gripper_angle()*PI/180), -0.05])
        print("Grasping...")
        grasp()
        time.sleep(5)

        # place to target
        ur.home(locations[target])
        ur.move_to(target_loc)
        ur.move_by_diff([-OFFSET_CAM_H*np.sin(ur.gripper_angle()*PI/180), -OFFSET_CAM_H*np.cos(ur.gripper_angle()*PI/180), -0.1])
        print("Releasing...")
        release()
        time.sleep(5)

        labels[object] = locations[target]
        locations[object] = locations[target]

    print("Congrats! Happy Graduation!")