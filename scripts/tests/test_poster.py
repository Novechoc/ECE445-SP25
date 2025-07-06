import sys, os, time, cv2
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from connection import Poster
# from utils import visualize_boxes, visualize_boxes_and_masks
from csi_camera import locate_and_visualize

if __name__ == "__main__":
    # ### Test 1
    # poster = Poster()
    # detections = poster.post('./car.jpg', 'car')['detections']  # Assuming this method returns a list of detections
    # print(detections)
    # visualize_boxes('car.jpg', detections)

    ### Test 2
    from UR3e import UR3e; ur = UR3e(); ur.home()
    from csi_camera import capture_image; capture_image()
    image_path = 'captured.jpg'
    labels = input("Enter labels here:\n")

    poster = Poster(app='gsam', port='7890', hostname='10.105.100.230')
    # poster._map_port()
    detections = poster.post(labels, image_path)
    if 'error' in detections.keys():
        print(f"Error: {detections['error']}")
        sys.exit(1)
    # boxes = []
    # masks = []
    # for label, det in detections.items():
    #     boxes.append(det['boxes'][0])
    #     masks.append(det['masks'][0])
    locate_and_visualize(detections)

    # ### Test 3
    # poster = Poster(app='gsam', port='7890', hostname='10.105.100.230', pre_command='cd /home/lh/; source start.sh; conda activate gsam')
    # detections = poster.post('./cup1.jpg', 'cup')
    # if 'error' in detections.keys():
    #     print(f"Error: {detections['error']}")
    #     sys.exit(1)
    # masks = detections['masks']
    # angles = []
    # image = cv2.imread('./cup1.jpg')
    # for mask in masks:
    #         mask = np.array(mask).astype('uint8') * 255
    #         contour, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #         print(len(contour))
    #         ellipse = cv2.fitEllipse(contour[0])
            
    #         angles.append(ellipse)
    # for angle in angles:
    #     cv2.ellipse(image, angle, (0, 255, 0), 2)
    #     cv2.imwrite('output_with_angles.jpg', image)
    # print(angles)
    # del(poster)

    # ### Test 4
    # image_path = './cup2.jpg'
    # poster = Poster(app='gsam', port='7890', hostname='10.105.100.230', pre_command='cd /home/lh/; source start.sh; conda activate gsam')
    # detections = poster.post(image_path, 'cup')
    # if 'error' in detections.keys():
    #     print(f"Error: {detections['error']}")
    #     sys.exit(1)
    # masks = detections['masks']
    # angles = []
    # image = cv2.imread(image_path)
    # for mask in masks:
    #         mask = np.array(mask).astype('uint8') * 255
    #         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #         largest_contour = max(contours, key=cv2.contourArea)
    #         data_pts = np.array([point[0] for point in largest_contour], dtype=np.float32)

    #         mean, eigenvectors = cv2.PCACompute(data_pts, mean=None)
    #         center = tuple(mean[0].astype(int))

    # for ev in eigenvectors:
    #     pt1 = center
    #     pt2 = (int(center[0] + ev[0] * 100), int(center[1] + ev[1] * 100))
    #     cv2.line(image, pt1, pt2, (0, 255, 0), 2)

    # cv2.imwrite('output_with_angles.jpg', image)

    # del(poster)