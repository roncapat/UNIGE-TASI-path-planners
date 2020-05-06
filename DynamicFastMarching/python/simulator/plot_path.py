import colorsys
import json
import os
import sys
from math import modf, ceil, floor

import cv2
import numpy
from PIL import ImageFont, ImageDraw, Image

thismodule = sys.modules[__name__]


def plot_path(mapfile, pathfile, dbgfile):
    mapdata = cv2.imread(mapfile, 0)
    with open(pathfile, 'r') as pathfile:
        poses = json.load(pathfile)['poses']
    with open(dbgfile, 'r') as dbgfile:
        expanded = json.load(dbgfile)['expanded']
    return plot_path_on_map(mapdata, dbgfile, nextpath=poses, expanded=expanded)


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
        if p[3] != float('inf'):
            max_g = max(max_g, p[3])

    period = max_g / 8
    for p in expanded:
        x = p[1]
        y = p[0]
        if p[2] != float("inf"):
            (r, g, b) = colorsys.hsv_to_rgb(p[2] / period, 1.0, 1.0)
            color = (int(200 * r), int(200 * g), int(200 * b))
            cv2.circle(out_map, (mult * x + floor(mult / 2), mult * y + floor(mult / 2)), ceil(mult / 5), color,
                       cv2.FILLED)
        else:
            cv2.circle(out_map, (mult * x + floor(mult / 2), mult * y + floor(mult / 2)), floor(mult / 5),
                       (0, 0, 0), cv2.FILLED)
        if p[2] != p[3]:
            cv2.circle(out_map, (mult * x + floor(mult / 2), mult * y + floor(mult / 2)), ceil(mult / 3),
                       (100, 100, 100), 1)

    for a, b in zip(prevpath, prevpath[1:]):
        a_scaled = (
            mult * int(a[1]) + int(round(modf(a[1])[0] * (mult - 1))),
            mult * int(a[0]) + int(round(modf(a[0])[0] * (mult - 1))))
        b_scaled = (
            mult * int(b[1]) + int(round(modf(b[1])[0] * (mult - 1))),
            mult * int(b[0]) + int(round(modf(b[0])[0] * (mult - 1))))
        cv2.line(out_map, a_scaled, b_scaled, (150, 120, 50), floor(mult / 4))

    for a, b in zip(nextpath, nextpath[1:]):
        a_scaled = (
            mult * int(a[1]) + int(round(modf(a[1])[0] * (mult - 1))),
            mult * int(a[0]) + int(round(modf(a[0])[0] * (mult - 1))))
        b_scaled = (
            mult * int(b[1]) + int(round(modf(b[1])[0] * (mult - 1))),
            mult * int(b[0]) + int(round(modf(b[0])[0] * (mult - 1))))
        cv2.line(out_map, a_scaled, b_scaled, (255, 100, 0), floor(mult / 6))

    x = int(mult * nextpath[-1][1])
    y = int(mult * nextpath[-1][0])
    cv2.circle(out_map, (x, y), floor(mult / 4), (255, 100, 0), cv2.FILLED)  # BLUE - GOAL
    x = int(mult * nextpath[0][1])
    y = int(mult * nextpath[0][0])
    cv2.circle(out_map, (x, y), floor(mult / 2), (100, 255, 100), cv2.FILLED)  # GREEN - START

    # Choosen font is monospaced: width is 0.6*height
    if info is not None:
        vmargin = 5
        lmargin = 20
        fsize = int((width - 2 * lmargin) / 70 / 0.6)  # scale to get a usable line size of 50 chars
        captionc = "           cost[wu]"
        caption1 = "So far   %10.02f" % info["cost_from_start"]
        caption2 = "To goal  %10.02f" % info["cost_to_goal"]
        captiont = "           step[ms] total[ms]"
        caption3 = "Update     %8.2f  %8.2f" % (info['update'], info['update_tot'])
        caption4 = "Planning   %8.2f  %8.2f" % (info['planning'], info['planning_tot'])
        caption5 = "Extraction %8.2f  %8.2f" % (info['extraction'], info['extraction_tot'])
        caption6 = "Cumulative %8.2f  %8.2f" % (info['cum'], info['cum_tot'])
        fontpath = os.path.dirname(os.path.abspath(thismodule.__file__)) + "/SourceCodePro-Regular.ttf"
        font = ImageFont.truetype(fontpath, fsize)
        char_width = font.getsize('h')[0]  # this should be 0.6*fsize
        line_height = font.getsize('hg')[1]
        out_map = cv2.copyMakeBorder(out_map, line_height * 5 + vmargin * 2, 0, 0, 0, cv2.BORDER_CONSTANT,
                                     value=(255, 255, 255))
        img_pil = Image.fromarray(out_map)
        draw = ImageDraw.Draw(img_pil)
        tw = font.getsize(caption3)[0]
        cw = font.getsize(caption1)[0]
        base_t = width - cw - tw - char_width * 5
        base_c = width - cw - lmargin
        draw.text((lmargin, vmargin), "Dynamic FM", font=font, fill=(100, 100, 100))
        draw.text((base_c, vmargin), captionc, font=font, fill=(100, 100, 100))
        draw.text((base_c, vmargin + line_height), caption1, font=font, fill=(100, 100, 100))
        draw.text((base_c, vmargin + line_height * 2), caption2, font=font, fill=(100, 100, 100))
        draw.text((base_t, vmargin), captiont, font=font, fill=(100, 100, 100))
        draw.text((base_t, vmargin + line_height), caption3, font=font, fill=(100, 100, 100))
        draw.text((base_t, vmargin + line_height * 2), caption4, font=font, fill=(100, 100, 100))
        draw.text((base_t, vmargin + line_height * 3), caption5, font=font, fill=(100, 100, 100))
        draw.text((base_t, vmargin + line_height * 4), caption6, font=font, fill=(100, 100, 100))
        out_map = numpy.array(img_pil)
    return out_map


if __name__ == "__main__":
    if len(sys.argv) < 5:
        sys.stderr.write("Usage:\n\t %s <mapfile.bmp> <logfile.json> <dbgfile.json> <outfile.jpg>\n" % sys.argv[0])
        exit()
    result = plot_path(*sys.argv[1:4])
    cv2.imwrite(sys.argv[4], result)
