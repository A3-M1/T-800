import cv2
import numpy as np
import realsensetools as rst

pipeline, _ = rst.initRealsenseCamera()

frame, _ = rst.getRealsenseFrames(pipeline)

rst.closeRealsenseCamera(pipeline)

# Select ROI
r = cv2.selectROI(frame)

# Crop image
imCrop = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]

average_h = np.mean(imCrop[:,:,0])
average_s = np.mean(imCrop[:,:,1])
average_v = np.mean(imCrop[:,:,2])

print(average_h,average_s,average_v)

# Display cropped image
cv2.imshow("Image", imCrop)
cv2.waitKey(0)