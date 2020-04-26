import json
import sys, os
from math import modf, ceil, floor

import cv2
import numpy
from PIL import ImageFont, ImageDraw, Image


def plot_path(mapfile, pathfile, dbgfile):
    map = cv2.imread(mapfile, 0)
    with open(pathfile, 'r') as pathfile:
        poses = json.load(pathfile)['poses']
    with open(dbgfile, 'r') as dbgfile:
        expanded = json.load(dbgfile)['expanded']
    plot_path_on_map(map, dbgfile, nextpath=poses, expanded=expanded)


def plot_path_on_map(img, prevpath=[], nextpath=[], expanded=[], info=None):
    mult = 21

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
    max_g = 0
    for p in expanded:
        if p[2] != float('inf'):
            max_g = max(max_g, p[2])

    for p in expanded:
        x = p[1]
        y = p[0]
        if p[2] >= 0 and p[2] != float("inf"):
            g = p[2] / max_g * 255
            cv2.circle(out_map, (mult * x, mult * y), ceil(mult / 4), (80, int(255 - g / 2), int(g)),
                       cv2.FILLED)
        else:
            cv2.circle(out_map, (mult * x, mult * y), floor(mult / 5), (30, 30, 255), cv2.FILLED)

    for a, b in zip(prevpath, prevpath[1:]):
        a_scaled = (
            mult * int(a[1]) + int(round(modf(a[1])[0] * (mult - 1))),
            mult * int(a[0]) + int(round(modf(a[0])[0] * (mult - 1))))
        b_scaled = (
            mult * int(b[1]) + int(round(modf(b[1])[0] * (mult - 1))),
            mult * int(b[0]) + int(round(modf(b[0])[0] * (mult - 1))))
        cv2.line(out_map, a_scaled, b_scaled, (150, 120, 50), floor(mult / 3))

    for a, b in zip(nextpath, nextpath[1:]):
        a_scaled = (
            mult * int(a[1]) + int(round(modf(a[1])[0] * (mult - 1))),
            mult * int(a[0]) + int(round(modf(a[0])[0] * (mult - 1))))
        b_scaled = (
            mult * int(b[1]) + int(round(modf(b[1])[0] * (mult - 1))),
            mult * int(b[0]) + int(round(modf(b[0])[0] * (mult - 1))))
        cv2.line(out_map, a_scaled, b_scaled, (255, 100, 0), floor(mult / 5))

    x = int(nextpath[-1][1])
    y = int(nextpath[-1][0])
    cv2.circle(out_map, (mult * x, mult * y), floor(mult / 2), (255, 0, 0), cv2.FILLED)  # BLUE - GOAL
    x = int(mult * nextpath[0][1])
    y = int(mult * nextpath[0][0])
    cv2.circle(out_map, (x, y), floor(mult / 2), (100, 255, 100), cv2.FILLED)  # GREEN - START

    if info is not None:
        caption1 = "Cost so far   %10.02f" % info["cost_from_start"]
        caption2 = "Cost to goal  %10.02f" % info["cost_to_goal"]
        fontpath = "./SourceCodePro-Regular.ttf"
        font = ImageFont.truetype(fontpath, int(width/30))
        out_map = cv2.copyMakeBorder(out_map, 140, 0, 0, 0, cv2.BORDER_CONSTANT, value=(255, 255, 255))
        img_pil = Image.fromarray(out_map)
        draw = ImageDraw.Draw(img_pil)
        draw.text((width/20, 30), "Field D*", font=font, fill=(100, 100, 100))
        draw.text((width*2/5, 5), caption1, font=font, fill=(100, 100, 100))
        draw.text((width*2/5, 55), caption2, font=font, fill=(100, 100, 100))
        out_map = numpy.array(img_pil)
    return out_map


if __name__ == "__main__":
    if len(sys.argv) < 5:
        sys.stderr.write("Usage:\n\t %s <mapfile.bmp> <logfile.json> <dbgfile.json> <outfile.jpg>\n" % sys.argv[0])
        exit()
    result = plot_path(*sys.argv[1:4])
    cv2.imwrite(sys.argv[4], result)
