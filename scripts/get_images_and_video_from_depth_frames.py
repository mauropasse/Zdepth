import numpy as np
import matplotlib.pyplot as plt
import os
import cv2

Width = 640
Height = 360

path = "./depth_frames/"
image_folder = "./image_folder/"

for file in os.listdir(path):
    Image = []
    count = 0
    with open(path + file, "rb") as f:
        line = f.readline()
        while line:
            dec = int(line, 16)
            Image.append(dec)
            line = f.readline()

    # Generate 2D dimensional arrays
    Image2D = np.reshape(Image, (Height, Width))
    # Generate image. Each depth of pixel represented by a different colour
    plt.matshow(Image2D, cmap=plt.cm.nipy_spectral)
    plt.savefig(image_folder + file + ".jpg")
    plt.close()

# Get list of all files in a given directory sorted by name
images = sorted(filter(lambda x: os.path.isfile(os.path.join(image_folder, x)), os.listdir(image_folder)))
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape
fps = 25
video = cv2.VideoWriter('video.avi', 0, fps, (width,height))

for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()
