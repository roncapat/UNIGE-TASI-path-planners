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
img_l = cv2.GaussianBlur(img_h, (7, 7), 0)

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

p_out.write(struct.pack('<b', 2))  # reply with 1
p_out.flush()

p.wait()
p_in.close()
p_out.close()
os.remove(pipe_in)
os.remove(pipe_out)
p = subprocess.Popen(["python3", path + "/plot_path_gui.py", sys.argv[1], sys.argv[9], sys.argv[10], "result.jpg"])
p.wait()
