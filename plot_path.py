import json

import cv2

img = cv2.imread('test.bmp', 0)
img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

with open('dbgfile_0.json', 'r') as dbgfile:
    expanded = json.load(dbgfile)['expanded']
    for p in expanded:
        # img[int(p[0]), int(p[1])] = [50, 50, 50]
        pass

with open('logfile_0.json', 'r') as pathfile:
    poses = json.load(pathfile)['poses']
    for p in poses:
        print(p)
        img[int(p[0]), int(p[1])] = [0, 0, 255]
    img[int(poses[0][0]), int(poses[0][1])] = [100, 255, 100]  # GREEN - START
    img[int(poses[-1][0]), int(poses[-1][1])] = [255, 20, 20]  # BLUE - GOAL

scale_percent = 5000  # percent of original size
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
# resize image
resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
cv2.imwrite('test_result.bmp', resized)
