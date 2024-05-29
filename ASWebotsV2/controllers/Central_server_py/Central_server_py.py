from controller import Robot, Camera
import numpy as np
import cv2

# Initialize the Webots robot controller
robot = Robot()

# Get the camera device and enable it
camera = robot.getDevice('camera')
camera.enable(32)  # Enable the camera with a certain sampling period (ms)

# Create an OpenCV window with a specific size
window_name = 'Webots Camera'
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_name, 800, 600)  # Resize the window to 800x600 pixels

# Main loop
while robot.step(32) != -1:
    # Get the image from the camera
    camera_image = camera.getImage()

    # Convert the image to a numpy array
    width = camera.getWidth()
    height = camera.getHeight()
    image = np.frombuffer(camera_image, np.uint8).reshape((height, width, 4))

    # Convert the image from BGRA to BGR format
    image_bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    # Display the image using OpenCV
    cv2.imshow(window_name, image_bgr)

    # Wait for a key press for a short period (1 ms)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cv2.destroyAllWindows()
