import sys

import cv2
from plot_path import *

if len(sys.argv) < 5:
    sys.stderr.write("Usage:\n\t %s <mapfile.bmp> <logfile.json> <dbgfile.json> <outfile.jpg>\n" % sys.argv[0])
    exit()

result = plot_path(*sys.argv[1:4])
cv2.namedWindow("Field D* planner", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Field D* planner", 1000, 1000)
cv2.imshow("Field D* planner", result)
cv2.waitKey(500)
cv2.destroyAllWindows()
