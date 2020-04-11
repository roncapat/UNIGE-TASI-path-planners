import json
import sys
from math import modf, ceil, floor

import cv2
import numpy


def plot_path():
    if len(sys.argv) < 5:

        sys.stderr.write("Usage:\n\t %s <mapfile.bmp> <logfile.json> <dbgfile.json> <outfile.bmp>\n" % sys.argv[0])
        exit()

    mult = 21

    img = cv2.imread(sys.argv[1], 0)
    width = int(img.shape[1] * mult) + 1
    height = int(img.shape[0] * mult) + 1
    out_map = numpy.ones((height, width, 3), numpy.uint8) * 255
    for y in range(0, height, mult):
        cv2.line(out_map, (0, y), (width, y), (150, 150, 150), 1)
    for x in range(0, width, mult):
        cv2.line(out_map, (x, 0), (x, height), (150, 150, 150), 1)
    for y in range(0, img.shape[0]):
        for x in range(0, img.shape[1]):
            c = int(img[y][x])
            cv2.rectangle(out_map, (mult * x + 1, mult * y + 1), (mult * x + (mult - 1), mult * y + (mult - 1)),
                          (c, c, c),
                          cv2.FILLED)

    with open(sys.argv[3], 'r') as dbgfile:
        expanded = json.load(dbgfile)['expanded']
        for p in expanded:
            x = p[1]
            y = p[0]
            if p[2] >= 0:
                cv2.circle(out_map, (mult * x, mult * y), ceil(mult / 4), (80, int(255 - p[2] / 2), int(p[2])),
                           cv2.FILLED)
            else:
                cv2.circle(out_map, (mult * x, mult * y), floor(mult / 5), (30, 30, 255), cv2.FILLED)

    with open(sys.argv[2], 'r') as pathfile:
        poses = json.load(pathfile)['poses']
        for a, b in zip(poses, poses[1:]):
            a_scaled = (
                mult * int(a[1]) + int(modf(a[1])[0] * (mult - 1)), mult * int(a[0]) + int(modf(a[0])[0] * (mult - 1)))
            b_scaled = (
                mult * int(b[1]) + int(modf(b[1])[0] * (mult - 1)), mult * int(b[0]) + int(modf(b[0])[0] * (mult - 1)))
            cv2.line(out_map, a_scaled, b_scaled, (255, 100, 0), floor(mult / 5))
        x = int(poses[0][1])
        y = int(poses[0][0])
        cv2.circle(out_map, (mult * x, mult * y), floor(mult / 2), (100, 255, 100), cv2.FILLED)  # GREEN - START
        x = int(poses[-1][1])
        y = int(poses[-1][0])
        cv2.circle(out_map, (mult * x, mult * y), floor(mult / 2), (255, 0, 0), cv2.FILLED)  # BLUE - GOAL

    cv2.imwrite(sys.argv[4], out_map)
    return out_map


if __name__ == "__main__":
    plot_path()
