#!/usr/bin/python3
import subprocess
from operator import add
import numpy as np
from matplotlib import patches
from matplotlib import cm
from matplotlib.path import Path
from noise import pnoise2
import socket

# Takes a round region of data from data_h and pastes it in data_l
# Returns a rectangular area with the patch to apply to data_l, and the
# corresponding upper-left position and data ranges
def round_patch_update(data_l, data_h, center, radius):
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
    data_m = data_h.copy()
    cv2.circle(data_l, (left + radius + off_x, top + radius + off_y), radius, 0, cv2.FILLED)
    cv2.circle(data_m, (left + radius + off_x, top + radius + off_y), radius, 0, cv2.FILLED)
    data_l = (data_h - data_m) + data_l
    position = (top, left)
    ranges = (slice(top, bottom), slice(left, right))
    patch = data_l[ranges[0], ranges[1]]
    return data_l, patch, position, ranges


def rmf(filename):
    try:
        os.remove(filename)
    except OSError:
        pass


def get_byte(pipe):
    return struct.unpack('b', pipe.read(1))[0]


def wait_byte(pipe, val):
    ack = None
    while ack != val:
        ack = struct.unpack('b', pipe.read(1))[0]  # wait for val


def send_byte(pipe, val):
    pipe.write(struct.pack('b', val))  # reply with 0
    pipe.flush()


def send_int(pipe, val):
    pipe.write(struct.pack('>i', val))  # reply with 0
    pipe.flush()


def send_float(pipe, val):
    pipe.write(struct.pack('>f', float(val)))  # reply with 0
    pipe.flush()


def send_patch(pipe, data, pos):
    pipe.write(struct.pack('>iiii', *pos, *data.shape))
    pipe.write(data.tobytes())  # patch
    pipe.flush()


def send_map(pipe, data):
    print("Writing map size... [%d %d]" % (data.shape[1], data.shape[0]))
    pipe.write(struct.pack('>ii', data.shape[1], data.shape[0]))
    print("Writing map...")
    pipe.write(data.tobytes())  # patch
    print("Flushing stream...")
    pipe.flush()
    print("Map sent.")


def receive_path(pipe):
    n_steps = struct.unpack('>i', pipe.read(4))[0]
    path = []
    costs = []
    for i in range(n_steps):
        buf = pipe.read(8)
        path.append(struct.unpack('>ff', buf))
    for i in range(n_steps - 1):
        costs.append(struct.unpack('>f', pipe.read(4))[0])
    buf = pipe.read(8)
    dist, cost = struct.unpack('>ff', buf)
    buf = pipe.read(12)
    ut, pt, et = struct.unpack('>fff', buf)
    times = {"update": ut, "planning": pt, "extraction": et}
    return path, costs, dist, cost, times


def receive_expanded(pipe):
    n_expanded = struct.unpack('>q', pipe.read(8))[0]
    expanded = []
    for i in range(n_expanded):
        buf = pipe.read(16)
        expanded.append(struct.unpack('>iiff', buf))
    return expanded


def receive_traversal_update(pipe_in):
    pos_x, pos_y, step_cost = struct.unpack('>fff', pipe_in.read(12))
    return pos_x, pos_y, step_cost


def simulation_data(img_h, filter_radius, low_res_penalty):
    img_l = cv2.GaussianBlur(img_h, (filter_radius, filter_radius), 0)
    _h_data = ~img_h
    _h_data = _h_data + (_h_data == 0)
    _l_data = ~img_l
    _l_data = _l_data + (_l_data == 0)
    _l_data = cv2.add(_l_data, low_res_penalty)
    return _l_data, _h_data

planners = {
    #"FD_0": {"path": "build/FD_0_no_heur/field_d_planner_0_no_heur", "type": "n"},
    #"FD_1": {"path": "build/FD_1_no_heur/field_d_planner_1_no_heur", "type": "n"},
    #"SGDFM_0": {"path": "build/SGDFM_0_no_heur/shifted_grid_planner_0_no_heur", "type": "n"},
    #"SGDFM_1": {"path": "build/SGDFM_1_no_heur/shifted_grid_planner_1_no_heur", "type": "n"},
    #"SGDFM_2": {"path": "build/SGDFM_2_no_heur/shifted_grid_planner_2_no_heur", "type": "n"},
    #"DFM_0": {"path": "build/DFM_0/dfm_planner_0", "type": "c"},
    "DFM_1": {"path": "build/DFM_1/dfm_planner_1", "type": "c"},
}

cspace_diam = 1
gui = 1
tof = 0
outpath = "Results"
upscale = 1
use_heuristic = False
cmap = cm.winter

# TODO loop over inputs maps
mapfile = "Tests/000_gradient.bmp"
results = {}

mapname = os.path.basename(mapfile)
mappath = os.path.abspath(mapfile)
img_h = cv2.imread(mappath, cv2.IMREAD_GRAYSCALE)
img_h = img_h.repeat(upscale, axis=0).repeat(upscale, axis=1)
[height, width] = img_h.shape

#print("[SIMULATOR] Begin noisemap generation")
#randBytes = os.urandom(3)
#rand1 = int.from_bytes(randBytes, byteorder='big') % 98
#randBytes = os.urandom(3)
#rand2 = int.from_bytes(randBytes, byteorder='big') % 99
#randBytes = os.urandom(3)
#rand3 = int.from_bytes(randBytes, byteorder='big') % 100

#out_image = np.zeros((height, width), np.float32)
#for y in range(height):
#    for x in range(width):
#        a = pnoise2(x / 157.17, y / 157.17, octaves=4, lacunarity=2.5, repeatx=width, repeaty=height, base=rand1)
#        b = pnoise2((y + x) / 79.31, (y - x) / 79.31, octaves=3, lacunarity=2.4, repeatx=width, repeaty=height,
#                    base=rand2)
#        c = pnoise2(y / 53.13, x / 53.13, octaves=2, lacunarity=2.3, repeatx=width, repeaty=height, base=rand3)
#        a = (a + 1) / 2
#        b = (b + 1) / 2
#        c = (c + 1) / 2
#        a = a * a * a
#        b = b * b
#        c = c * c
#        out_image[y][x] = (a + b + c)

#[cmin, cmax] = np.percentile(out_image, [20, 90])
#out_image = (np.clip(out_image, cmin, cmax) - cmin) / (cmax - cmin)
#rock_aboundance_h = np.uint8(out_image * 255)
#k = np.ones((7, 7), np.uint8)
#rock_aboundance_h = cv2.erode(rock_aboundance_h, k, iterations=3)
#rock_aboundance_h = cv2.dilate(rock_aboundance_h, k, iterations=1)
#cv2.imwrite("rocks.tiff", rock_aboundance_h)
#print("[SIMULATOR] End noisemap generation")

# FIXME remove - this is only a test for not having rocks
rock_aboundance_h = np.zeros((height, width), np.uint8)

# create with socat PTY,link=/tmp/planner_tty,raw,echo=0 PTY,link=/tmp/simulator_tty,raw,echo=0
#tty = os.open("/dev/simulator_tty", os.O_RDWR)

import socket

HOST = '192.168.17.128'  # Standard loopback interface address (localhost)
PORT = 1234        # Port to listen on (non-privileged ports are > 1023)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen()
conn, addr = s.accept()
print("Connection accepted from ", addr)
tty = conn.fileno()

p_in = os.fdopen(tty, "rb")
p_out = os.fdopen(tty, "w+b", buffering=0)

for planner in planners:
    #print("[SIMULATOR] Launching %s planner" % planner)
    #label = planner
    #perf_args = ["perf", "record", "--call-graph", "dwarf", "-o", "/mnt/sdb/perf_" + planner + ".data"]
    #prog_args = [planners[planner]["path"], "/tmp/planner_tty", "/tmp/planner_tty"]
    #perf_args.extend(prog_args)
    #str_args = [str(x) for x in perf_args]
    #print(' '.join(str_args))
    #process = subprocess.Popen(str_args)

    wait_byte(p_in, 0)
    send_byte(p_out, 0)

    kernel = None
    if planners[planner]['type'] == 'n':
        # penalyse FD* and SGDFM by 1 (in radius) (since DFM interpolation yelds INFINITY on the border of obstacles)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (cspace_diam * upscale + 2, cspace_diam * upscale + 2))
    else:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cspace_diam * upscale, cspace_diam * upscale))

    (data_l, data_h) = simulation_data(img_h, filter_radius=3, low_res_penalty=10)
    rock_aboundance_l = np.zeros((height, width), np.uint8)
    cspace = cv2.dilate(data_l, kernel)  # FIXME dilate only patch?
    min_cost = cv2.minMaxLoc(cspace)[0]
    send_map(p_out, cspace)

    send_float(p_out, 100) #startx
    send_float(p_out, 100) #starty
    send_float(p_out, 1000 * upscale - 100) #endx
    send_float(p_out, 1000 * upscale - 100) #endy
    send_byte(p_out, 0) #tof
    send_int(p_out, int(min_cost))
    print("Sent all parameters.")


    prev_path = []
    next_path = []
    cur_path = []
    est_path = []
    cur_cost = []
    est_cost = []
    uts = []
    pts = []
    ets = []
    prev_pos_x = None
    prev_pos_y = None

    print("Waiting for planner...")
    while get_byte(p_in) == 1:
        pos_x, pos_y, step_cost = receive_traversal_update(p_in)
        if prev_pos_x and prev_pos_y:
            if pos_x == prev_pos_x and pos_y == prev_pos_y:
                print("[SIMULATOR] Warning: %s planner got stuck" % planner)
                #process.kill()
                break
        prev_pos_x = pos_x
        prev_pos_y = pos_y
        prev_path.append([pos_x, pos_y])
        center = (int(round(pos_y)), int(round(pos_x)))
        (data_l, p_data, p_pos, p_ranges) = round_patch_update(data_l, data_h, center, radius=5 * upscale)
        (rock_aboundance_l, _, _, _) = round_patch_update(rock_aboundance_l, rock_aboundance_h, center,
                                                          radius=5 * upscale)

        cspace = cv2.dilate(np.maximum(data_l, rock_aboundance_l), kernel)
        p_data_cspace = cspace[p_ranges[0], p_ranges[1]]

        if use_heuristic:
            min_cost = cv2.minMaxLoc(cspace)[0]

        send_byte(p_out, 1)
        send_patch(p_out, p_data_cspace, p_pos)
        send_int(p_out, int(min_cost))

        wait_byte(p_in, 3)
        (next_path, costs, dist, cost, times) = receive_path(p_in)

        if len(cur_cost) > 0:
            cur_cost.append(cur_cost[-1] + step_cost)
        else:
            cur_cost.append(0)
        est_cost.append(sum(costs))
        cur_path.append(prev_path)
        est_path.append(next_path)
        uts.append(times['update'])
        pts.append(times['planning'])
        ets.append(times['extraction'])

    results[planner] = {
        "cur_path": cur_path,
        "est_path": est_path,
        "cur_cost": cur_cost,
        "est_cost": est_cost,
        "uts": uts,
        "pts": pts,
        "ets": ets
    }

    send_byte(p_out, 2)
    #process.wait()
    # os.chmod("/mnt/sdb/perf_" + planner + ".data", 0o666) //FIXME need permissions

os.close(tty)


plt.figure(1)
plt.title("Replanning time analysis")
plt.gcf().canvas.set_window_title("Replanning time analysis")
for planner in planners:
    runtime = list(map(add, results[planner]["uts"][1:], results[planner]["pts"][1:]))
    plt.plot(range(1, len(runtime) + 1), runtime, label=planner)
plt.xlabel("Steps")
plt.ylabel("Time (ms)")

fig, axs = plt.subplots(1, 2, gridspec_kw={'width_ratios': [5, 1]})
axs[0].set_title("First plan time analysis")
plt.gcf().canvas.set_window_title("First plan time analysis")
axs[0].set_xlabel("Steps")
axs[0].set_ylabel("Time (ms)")
cmap = plt.get_cmap("tab10")
labels = []
times = []
for planner in planners:
    labels.append(planner)
    times.append(results[planner]["pts"][0])
pos = range(len(labels))
axs[0].bar(pos, times, color=cmap.colors)
axs[0].set_xticks(pos)
axs[0].set_xticklabels(labels, rotation='vertical')
fig.set_size_inches(19.2, 10.8)

axs[1].axis('off')
the_table = axs[1].table(cellText=[["%8.2f" % t] for t in times],
                         rowLabels=labels,
                         colLabels=["time(ms)"],
                         loc='center')
plt.savefig("Results/first_run_time.png", quality=100, dpi=100)

fig, axs = plt.subplots(1, 2, gridspec_kw={'width_ratios': [5, 1]})
axs[0].set_title("Replanning time analysis - mean value and standard deviation")
plt.gcf().canvas.set_window_title("Replanning time analysis - mean value and standard deviation")
axs[0].set_xlabel("Steps")
axs[0].set_ylabel("Time (ms)")
cmap = plt.get_cmap("tab10")
labels = []
avgs = []
stdevs = []
for planner in planners:
    labels.append(planner)
    runtime = list(map(add, results[planner]["uts"][1:], results[planner]["pts"][1:]))
    avgs.append(np.mean(runtime))
    stdevs.append(np.std(runtime))
pos = range(len(labels))
axs[0].bar(pos, avgs, color=cmap.colors, yerr=stdevs, capsize=5)
axs[0].set_xticks(pos)
axs[0].set_xticklabels(labels, rotation='vertical')
fig.set_size_inches(19.2, 10.8)

axs[1].axis('off')
the_table = axs[1].table(cellText=[["%8.4f" % t1, "%4.4f" % t2] for t1, t2 in zip(avgs, stdevs)],
                         rowLabels=labels,
                         colLabels=["mean(ms)", "std(ms)"],
                         loc='center')
plt.savefig("Results/replan_time.png", quality=100, dpi=100)

figure = plt.figure(4)
plt.title("Path projection on slope map")
plt.gcf().canvas.set_window_title("Path projection on slope map")
ax = plt.gca()
handles = []
for planner, idx in zip(planners, range(len(planners))):
    points = [[y, height - x] for x, y in results[planner]["cur_path"][-1]]
    path = Path(points)
    patch = patches.PathPatch(path, facecolor='none', lw=2, edgecolor=cmap.colors[idx], label=planner)
    handles.append(patch)
    ax.add_patch(patch)
plt.legend(handles=handles)
plt.imshow(~img_h, cmap='viridis_r', extent=[0, height, 0, width])
bar = plt.colorbar(ticks=[0, 25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 255])
bar.ax.set_yticklabels(['0', '25', '50', '75', '100', '125', '150', '175', '200', '225', '250', 'obstacle'])
ax.set_xlim(0, width)
ax.set_ylim(0, height)
figure.set_size_inches(19.2, 10.8)
plt.savefig("Results/executed_paths.png", quality=100, dpi=100)

figure = plt.figure(5)
plt.title("Path projection on risk map")
plt.gcf().canvas.set_window_title("Path projection on risk map")
ax = plt.gca()
handles = []
for planner, idx in zip(planners, range(len(planners))):
    points = [[y, height - x] for x, y in results[planner]["cur_path"][-1]]
    path = Path(points)
    patch = patches.PathPatch(path, facecolor='none', lw=2, edgecolor=cmap.colors[idx], label=planner)
    handles.append(patch)
    ax.add_patch(patch)
plt.legend(handles=handles)
plt.imshow(np.maximum(data_h, rock_aboundance_h), cmap='viridis_r', extent=[0, height, 0, width])
bar = plt.colorbar(ticks=[0, 25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 255])
bar.ax.set_yticklabels(['0', '25', '50', '75', '100', '125', '150', '175', '200', '225', '250', 'obstacle'])
ax.set_xlim(0, width)
ax.set_ylim(0, height)
figure.set_size_inches(19.2, 10.8)
plt.savefig("Results/executed_paths_noise.png", quality=100, dpi=100)

plt.show()
