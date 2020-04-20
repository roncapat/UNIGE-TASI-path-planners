import os
import struct
import subprocess
import sys

import cv2

path = "/".join(os.path.abspath(__file__).split("/")[:-1])

if len(sys.argv) < 11:
    print("Usage:")
    print("\t %s <mapfile.bmp> <from_x> <from_y> <to_x> <to_y>"
          " lookahead cspace optimiziation_lvl "
          "<logfile.json> <dbgfile.json> <infofile.json>" % sys.argv[0])
    sys.exit(1)

pipe_out = os.path.abspath("map_pipe_in")
pipe_in = os.path.abspath("map_pipe_out")

try:
    os.remove(pipe_in)
except OSError:
    pass

try:
    os.remove(pipe_out)
except OSError:
    pass

os.mkfifo(pipe_out, 0o666)
os.mkfifo(pipe_in, 0o666)

args = [path + "/../cmake-build-debug/FieldDStar/field_d_planner", *(sys.argv[1:]), pipe_out, pipe_in]
p = subprocess.Popen(args)
p_out = open(pipe_out, 'wb')
p_in = open(pipe_in, 'rb')
ack = None
while ack != 0:
    ack = struct.unpack('b', p_in.read(1))[0]  # wait for 0
p_out.write(struct.pack('<b', 0))  # reply with 0
p_out.flush()

img_h = cv2.imread(sys.argv[1], cv2.IMREAD_GRAYSCALE)
img_l = cv2.GaussianBlur(img_h, (11, 11), 0)

data_h = 255 - img_h
data_h = data_h + (data_h == 0)

data_l = 255 - img_l
data_l = data_l + (data_l == 0)

[height, width] = data_l.shape
print("[SIMULATOR] Size: [%i,%i]" % (width, height))
p_out.write(bytearray(struct.pack('<i', width)))
p_out.flush()
p_out.write(bytearray(struct.pack('<i', height)))
p_out.flush()
p_out.write(data_l.tobytes())
p_out.flush()

cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image', 1000, 1000)
scale = 5
out = cv2.VideoWriter('test.avi', cv2.VideoWriter_fourcc(*'DIVX'), 15, (height * scale, width * scale))
while True:
    ack = struct.unpack('b', p_in.read(1))[0]  # wait for 1
    if ack == 2:
        break
    if ack != 1:
        raise AssertionError
    pos_x = struct.unpack('f', p_in.read(4))[0]
    pos_y = struct.unpack('f', p_in.read(4))[0]
    print("[SIMULATOR] New position: [%f,%f]" % (pos_x, pos_y))
    p_out.write(struct.pack('<b', 1))  # reply with 1
    p_out.flush()

    radius = 15
    center = (int(round(pos_x)), int(round(pos_y)))
    off_x = off_y = 0
    top = center[1] - radius
    bottom = center[1] + radius + 1
    left = center[0] - radius
    right = center[0] + radius + 1
    if top < 0:
        off_y = top
        top = 0
    if left < 0:
        off_x = left
        left = 0
    top_left = (top, left)
    bottom_right = (bottom, right)
    data_rect_l = data_l[top:bottom, left:right].copy()
    data_rect_h = data_h[top:bottom, left:right].copy()
    data_rect_m = data_rect_h.copy()
    cv2.circle(data_rect_l, (radius + off_y, radius + off_x), radius, 0, cv2.FILLED)
    cv2.circle(data_rect_m, (radius + off_y, radius + off_x), radius, 0, cv2.FILLED)
    patch = data_rect_h - data_rect_m + data_rect_l
    data_l[top:bottom, left:right] = patch
    frame = cv2.resize(data_l, (height * scale, width * scale))
    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    cv2.circle(frame, (center[0] * scale, center[1] * scale), radius * scale, (0, 0, 0))
    cv2.circle(frame, (center[0] * scale, center[1] * scale), scale, (100, 0, 100), cv2.FILLED)
    cv2.imshow("image", frame)
    out.write(frame)
    cv2.waitKey(5)
    # TODO send update patch to planner

p_out.write(struct.pack('<b', 2))  # reply with 2
p_out.flush()

p.wait()
p_in.close()
p_out.close()
os.remove(pipe_in)
os.remove(pipe_out)

out.release()

p = subprocess.Popen(["python3", path + "/plot_path_gui.py", sys.argv[1], sys.argv[9], sys.argv[10], "result.jpg"])
p.wait()
