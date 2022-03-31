import numpy as np
import matplotlib.pyplot as plt
import os
import cv2

image_folder = "./colormap_images/"

# Read raw depth frames (single array),convert to a matrix (2D) and map
# each pixel depth with a different color. Create JPG image of each frame,
# then create video from it (using 25 frames per second)
for image in os.listdir("depth_frames/"):
    Image = np.fromfile("depth_frames/"+image, dtype=np.uint16)
    # Generate 2D dimensional arrays
    Width = 848 # 640
    Height = 480 # Real robot height: 360. Sim: 480
    Image2D = np.reshape(Image, (Height, Width))
    # Generate image. Each depth of pixel represented by a different colour
    plt.matshow(Image2D, cmap=plt.cm.nipy_spectral)
    plt.savefig(image_folder + image + ".jpg")
    plt.close()

# Get images sorted by name
images = sorted(filter(lambda x: os.path.isfile(os.path.join(image_folder, x)), os.listdir(image_folder)))
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape
fps = 25 # 18 for raw frames
video = cv2.VideoWriter('video.avi', 0, fps, (width,height))

for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()


