import cv2
import numpy as np

def bgr_to_binary(image_path, threshold_value):
    # Read the BGR image
    bgr_image = cv2.imread(image_path)

    # Convert BGR to grayscale
    grayscale_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)

    # Apply thresholding to create a binary image
    _, binary_image = cv2.threshold(grayscale_image, threshold_value, 255, cv2.THRESH_BINARY)

    return binary_image

# Specify the path to your BGR image
image_path = '/home/dell/ros2_ws/src/swarm_turtle/scripts/firefox.png'

# Set the threshold value (adjust as needed)
threshold_value = 127

# Convert BGR to binary image
binary_image = bgr_to_binary(image_path, threshold_value)

# Display the original and binary images
cv2.imshow('Original Image', cv2.imread(image_path))
cv2.imshow('Binary Image', binary_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save the binary data to a file with .img extension
output_path = '/home/dell/ros2_ws/src/swarm_turtle/scripts/firefox.img'  # Change the path as needed
with open(output_path, 'wb') as file:
    file.write(binary_image)

print(f"Binary image saved at {output_path}")



# import cv2
# import numpy as np

# # Provide the absolute path to the binary image
# binary_image_path = '/home/dell/ros2_ws/src/swarm_turtle/scripts/binary_firefox.png'

# # Load the binary image
# binary_image = cv2.imread(binary_image_path, cv2.IMREAD_GRAYSCALE)

# if binary_image is not None:
#     # Ensure binary_image has at least 2 dimensions
#     if len(binary_image.shape) == 2:
#         # Create a BGR image with a solid color background
#         bgr_image = np.zeros((binary_image.shape[0], binary_image.shape[1], 3), dtype=np.uint8)
#         bgr_image[:, :] = (0, 255, 0)  # Set the color (here: green) for the foreground

#         # Apply the binary image as a mask
#         result_image = cv2.bitwise_and(bgr_image, bgr_image, mask=binary_image)

#         # Display or save the resulting BGR image
#         cv2.imshow('BGR Image from Binary', result_image)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()
#     else:
#         print("Binary image does not have the expected shape.")
# else:
#     print("Failed to load binary image. Please check the path.")

