import os, time, cv2
import numpy as np
PI = np.pi
from kinematics import sign_off_mod
from UR3e import norm360
import subprocess

def keep_capturing_images(width=640, height=480, timeout=0.1, save_path="captured.jpg"):
    try:
        while True:
            capture_image(width, height, timeout, save_path)
    except KeyboardInterrupt:
        print("Image capturing stopped.")

def capture_image(width=3280, height=2464, timeout=0.1, save_path="captured.jpg"):
    # Run the bash command to capture an image
    subprocess.run(
        ["libcamera-still",
        "-o", save_path, 
        "--timeout", str(timeout),
        "--nopreview",
        "--width", str(width),
        "--height", str(height)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        stdin=subprocess.DEVNULL,
        )
    # Wait for a short period to ensure the image is saved
    image = cv2.imread(save_path)
    image = cv2.flip(image, 0)
    image = cv2.flip(image, 1)
    # WARNING: compression affects the image coordinates
    image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
    cv2.imwrite(save_path, image, [cv2.IMWRITE_JPEG_QUALITY, 50])
    # cv2.imwrite(save_path, image)

def mask_to_ellipse(mask):
    '''deprecated'''
    mask = np.array(mask).astype('uint8') * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea)
    ellipse = cv2.fitEllipse(largest_contour)
    
    #
    M = cv2.moments(largest_contour)
    if M["m00"] != 0: # Avoid division by zero
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0  # or handle differently

    return largest_contour, (cx, cy), ellipse

def locate_and_visualize(detections, image_path='captured.jpg'):
    # boxes = []
    # masks = []
    image = cv2.imread(image_path)
    
    for label, det in detections.items():
        detections[label]['contours'] = []
        detections[label]['centers'] = []
        detections[label]['ellipses'] = []

        for i, box in enumerate(det['boxes']):
            x, y, xe, ye = [int(i) for i in box]
            cv2.rectangle(image, (x, y), (xe, ye), (0, 255, 0), 2)
            # Put label
            cv2.putText(image, f'{label}: {detections[label]['confidence'][i]}', (x, y - 10), 
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                        fontScale=0.7, 
                        color=(0, 255, 0), 
                        thickness=2)
        for mask in det['masks']:
            mask = np.array(mask).astype('uint8') * 255
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours, key=cv2.contourArea)
            detections[label]['contours'].append(largest_contour)
            cv2.drawContours(image, [largest_contour], -1, (0, 0, 255), 2)

            ellipse = cv2.fitEllipse(largest_contour)
            detections[label]['ellipses'].append(ellipse)
            cv2.ellipse(image, ellipse, (0, 255, 0), 2)

            M = cv2.moments(largest_contour)
            if M["m00"] != 0: # Avoid division by zero
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0  # or handle differently
                return None, None, None
            pt1 = (cx, cy)
            detections[label]['centers'].append(pt1)
            # pt2 = (int(pt1[0] + np.cos(ellipse[-1])*100), int(pt1[1] + np.sin(ellipse[-1])*100))
            # cv2.line(image, pt1, pt2, (0, 255, 0), 2)
        
        cv2.imwrite('output_with_masks.jpg', image)
        return detections

def top_to_real(u, v, z, camera_orientation, Width, Height, H=108, V=92):
    '''
    inputs:
    relative coordinate in the image 
    distance from the camera to the destination (height)
    orientation of the camera in degree

    output:
    the 2d position of the point on the desk
    '''
    assert 0 < u and u < Width and 0 < v and v < Height
    u /= Width
    v /= Height

    beam_angles = ((u-0.5)*H / 180*PI, (0.5-v)*V / 180*PI)
    rotated_xy_on_plain = (z * np.tan(beam_angles[0]), z * np.tan(beam_angles[1]))
    # print(rotated_xy_on_plain)

    diff_on_plain = np.sqrt(
        rotated_xy_on_plain[0]**2 + 
        rotated_xy_on_plain[1]**2
        )
    
    real_angle = norm360(np.arctan2(rotated_xy_on_plain[1], rotated_xy_on_plain[0]) - camera_orientation/180*PI)
    # print(real_angle, sign_off_mod(np.arctan2(rotated_xy_on_plain[1], rotated_xy_on_plain[0]) - camera_orientation/180*PI, PI))
    print(real_angle)
    diff_x = diff_on_plain * np.cos(real_angle)
    diff_y = diff_on_plain * np.sin(real_angle)
    print(diff_x, diff_y)

    return float(diff_x), float(diff_y)

def side_to_real():
    '''make use of yaw degree & clarify z with bottom point'''
    pass

if __name__ == "__main__":
    capture_image(3280, 2464, 0.1, "captured.jpg")

