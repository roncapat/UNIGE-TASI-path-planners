#!/usr/bin/python3
import subprocess
from operator import add
import numpy as np
from matplotlib import patches
from matplotlib import cm
from matplotlib.path import Path
from noise import pnoise2

from simulator.run_simulator import *  # TODO reorganize code

planners = {
    #"FD_0": {"path": "build/FD_0_no_heur/field_d_planner_0_no_heur", "type": "n"},
    #"FD_1": {"path": "build/FD_1_no_heur/field_d_planner_1_no_heur", "type": "n"},
    #"SGDFM_0": {"path": "build/SGDFM_0_no_heur/shifted_grid_planner_0_no_heur", "type": "n"},
    #"SGDFM_1": {"path": "build/SGDFM_1_no_heur/shifted_grid_planner_1_no_heur", "type": "n"},
    #"SGDFM_2": {"path": "build/SGDFM_2_no_heur/shifted_grid_planner_2_no_heur", "type": "n"},
    #"DFM_1": {"path": "build/DFM_1/dfm_planner_1", "type": "c"},
    "DFM_0": {"path": "build/DFM_0/dfm_planner_0", "type": "c"},
}

cspace_diam = 1
pipe_in = os.path.abspath("build/pipe_1")
pipe_out = os.path.abspath("build/pipe_2")
gui = 1
tof = 0
outpath = "Results"
upscale = 2
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

print("[SIMULATOR] Begin noisemap generation")
randBytes = os.urandom(3)
rand1 = int.from_bytes(randBytes, byteorder='big') % 98
randBytes = os.urandom(3)
rand2 = int.from_bytes(randBytes, byteorder='big') % 99
randBytes = os.urandom(3)
rand3 = int.from_bytes(randBytes, byteorder='big') % 100

out_image = np.zeros((height, width), np.float32)
for y in range(height):
    for x in range(width):
        a = pnoise2(x / 157.17, y / 157.17, octaves=4, lacunarity=2.5, repeatx=width, repeaty=height, base=rand1)
        b = pnoise2((y + x) / 79.31, (y - x) / 79.31, octaves=3, lacunarity=2.4, repeatx=width, repeaty=height,
                    base=rand2)
        c = pnoise2(y / 53.13, x / 53.13, octaves=2, lacunarity=2.3, repeatx=width, repeaty=height, base=rand3)
        a = (a + 1) / 2
        b = (b + 1) / 2
        c = (c + 1) / 2
        a = a * a * a
        b = b * b
        c = c * c
        out_image[y][x] = (a + b + c)

[cmin, cmax] = np.percentile(out_image, [20, 90])
out_image = (np.clip(out_image, cmin, cmax) - cmin) / (cmax - cmin)
rock_aboundance_h = np.uint8(out_image * 255)
k = np.ones((7, 7), np.uint8)
rock_aboundance_h = cv2.erode(rock_aboundance_h, k, iterations=3)
rock_aboundance_h = cv2.dilate(rock_aboundance_h, k, iterations=1)
cv2.imwrite("rocks.tiff", rock_aboundance_h)
print("[SIMULATOR] End noisemap generation")

#FIXME remove - this is only a test for not having rocks
rock_aboundance_h = np.zeros((height, width), np.uint8)

for planner in planners:
    print("[SIMULATOR] Launching %s planner" % planner)
    label = planner
    perf_args = ["perf", "record", "--call-graph", "dwarf", "-o", "/mnt/sdb/perf_" + planner + ".data"]
    prog_args = [planners[planner]["path"], mapfile, 100, 100, 1000 * upscale - 100, 1000 * upscale - 100, 1, pipe_out,
                 pipe_in, 1, 0, outpath]
    perf_args.extend(prog_args)
    str_args = [str(x) for x in perf_args]
    print(' '.join(str_args))
    process = subprocess.Popen(str_args)

    p_out = open(pipe_out, 'wb')
    p_in = open(pipe_in, 'rb')

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
    send_int(p_out, int(min_cost))

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

    while get_byte(p_in) == 1:
        pos_x, pos_y, step_cost = receive_traversal_update(p_in)
        if prev_pos_x and prev_pos_y:
            if pos_x == prev_pos_x and pos_y == prev_pos_y:
                print("[SIMULATOR] Warning: %s planner got stuck" % planner)
                process.kill()
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
    p_out.close()
    p_in.close()
    process.wait()
    #os.chmod("/mnt/sdb/perf_" + planner + ".data", 0o666) //FIXME need permissions

plt.figure(1)
plt.title("Replanning time analysis")
plt.gcf().canvas.set_window_title("Replanning time analysis")
plt.xlabel("Steps")
plt.ylabel("Time (ms)")
for planner in planners:
    runtime = list(map(add, results[planner]["uts"][1:], results[planner]["pts"][1:]))
    plt.plot(range(1, len(runtime) + 1), runtime, label=planner)

figure = plt.figure(2)
plt.title("First plan time analysis")
plt.gcf().canvas.set_window_title("First plan time analysis")
plt.xlabel("Steps")
plt.ylabel("Time (ms)")
cmap = plt.get_cmap("tab10")
labels = []
times = []
for planner in planners:
    labels.append(planner)
    times.append(results[planner]["pts"][0])
pos = range(len(labels))
plt.bar(pos, times, color=cmap.colors)
plt.xticks(pos, labels, rotation='vertical')
figure.set_size_inches(19.2, 10.8)
plt.savefig("Results/first_run_time.png", quality=100, dpi=100)

figure = plt.figure(3)
plt.title("Replanning time analysis - mean value and standard deviation")
plt.gcf().canvas.set_window_title("Replanning time analysis - mean value and standard deviation")
plt.xlabel("Steps")
plt.ylabel("Time (ms)")
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
plt.bar(pos, avgs, color=cmap.colors, yerr=stdevs, capsize=5)
plt.xticks(pos, labels, rotation='vertical')
figure.set_size_inches(19.2, 10.8)
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
