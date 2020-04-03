import json
import sys
from math import modf

import cv2
import numpy

if len(sys.argv) < 2:
    import sys

    sys.stderr.write("Usage:\n\t %s <mapfile.bmp>\n" % sys.argv[0])
    exit()

img = cv2.imread(sys.argv[1], 0)
width = int(img.shape[1] * 11) + 1
height = int(img.shape[0] * 11) + 1
out_map = numpy.ones((height, width, 3), numpy.uint8) * 255
for y in range(0, height, 11):
    cv2.line(out_map, (0, y), (width, y), (150, 150, 150), 1)
for x in range(0, width, 11):
    cv2.line(out_map, (x, 0), (x, height), (150, 150, 150), 1)
for y in range(0, img.shape[0]):
    for x in range(0, img.shape[1]):
        c = int(img[y][x])
        cv2.rectangle(out_map, (11 * x + 1, 11 * y + 1), (11 * x + 10, 11 * y + 10), (c, c, c), cv2.FILLED)

with open('dbgfile_0.json', 'r') as dbgfile:
    expanded = json.load(dbgfile)['expanded']
    for p in expanded:
        x = p[1]
        y = p[0]
        cv2.circle(out_map, (11 * x, 11 * y), 3, (0, 0, 255), cv2.FILLED)

with open('logfile_0.json', 'r') as pathfile:
    poses = json.load(pathfile)['poses']
    for a, b in zip(poses, poses[1:]):
        a_scaled = (11 * int(a[1]) + int(modf(a[1])[0] * 10), 11 * int(a[0]) + int(modf(a[0])[0] * 10))
        b_scaled = (11 * int(b[1]) + int(modf(b[1])[0] * 10), 11 * int(b[0]) + int(modf(b[0])[0] * 10))
        cv2.line(out_map, a_scaled, b_scaled, (255, 100, 0), 2)
    x = int(poses[0][1])
    y = int(poses[0][0])
    cv2.circle(out_map, (11 * x, 11 * y), 5, (100, 255, 100), cv2.FILLED)  # GREEN - START
    x = int(poses[-1][1])
    y = int(poses[-1][0])
    cv2.circle(out_map, (11 * x, 11 * y), 5, (255, 0, 0), cv2.FILLED)  # BLUE - GOAL

cv2.imshow('image', out_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite('test_result.bmp', out_map)
