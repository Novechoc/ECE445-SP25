import cv2

# Initialize camera
cap = cv2.VideoCapture(0)  # 0 is default webcam

if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

try:
    while True:
        # Capture frame
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            continue
        cv2.imshow("Captured", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Press 'q' to quit
            break
        elif key == ord('c'):  # Press 'c' to take another picture
            print("Taking another picture...")
        elif key == ord('s'):  # Press 's' to save the current picture
            cv2.imwrite("captured_image.jpg", frame)
            print("Image saved as 'captured_image.jpg'")

except KeyboardInterrupt:
    print("Stopping...")
finally:
    cap.release()
    cv2.destroyAllWindows()