#!/usr/bin/python3
import struct, sys, os, subprocess, cv2, shutil
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
from simulator.run_simulator import *  # TODO reorganize code
from operator import add

planners = {
    # "FD_0": {"path": "build/FD_0_no_heur/field_d_planner_0_no_heur", "type": "n"},
    "FD_1": {"path": "build/FD_1_no_heur/field_d_planner_1_no_heur", "type": "n"},
    # "SGDFM_0": {"path": "build/SGDFM_0_no_heur/shifted_grid_planner_0_no_heur", "type": "n"},
    # "SGDFM_1": {"path": "build/SGDFM_1_no_heur/shifted_grid_planner_1_no_heur", "type": "n"},
    "SGDFM_2": {"path": "build/SGDFM_2_no_heur/shifted_grid_planner_2_no_heur", "type": "n"},
    # "DFM_0": {"path": "build/DFM_0/dfm_planner_0", "type": "c"},
    "DFM_1": {"path": "build/DFM_1/dfm_planner_1", "type": "c"}
}

cspace = 1
pipe_in = os.path.abspath("build/pipe_1")
pipe_out = os.path.abspath("build/pipe_2")
gui = 1
tof = 0
outpath = "Results"

upscale = 5

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cspace * upscale, cspace * upscale))

# TODO loop over inputs maps
mapfile = "Tests/000_gradient.bmp"
results = {}

mapname = os.path.basename(mapfile)
mappath = os.path.abspath(mapfile)
img_h = cv2.imread(mappath, cv2.IMREAD_GRAYSCALE)
[height, width] = img_h.shape
img_h = img_h.repeat(upscale, axis=0).repeat(upscale, axis=1)
[height, width] = img_h.shape

for planner in planners:
    label = planner
    args = ["sudo", "perf", "record", "--call-graph", "dwarf", "-o", "/mnt/sdb/"+planner+".perf.data", planners[planner]["path"], mapfile, 100, 100, 1000 * upscale - 100, 1000 * upscale - 100, 1, pipe_out, pipe_in, 1, 0, outpath]
    #args = [planners[planner]["path"], mapfile, 100, 100, 1000*upscale-100, 1000*upscale-100, 1, pipe_out, pipe_in, 1, 0, outpath]
    str_args = [str(x) for x in args]
    print(' '.join(str_args))
    process = subprocess.Popen(str_args)

    p_out = open(pipe_out, 'wb')
    p_in = open(pipe_in, 'rb')

    wait_byte(p_in, 0)
    send_byte(p_out, 0)

    (data_l, data_h) = simulation_data(img_h, filter_radius=1, low_res_penalty=20)
    data_l_cspace = cv2.dilate(data_l, kernel)
    min_cost = cv2.minMaxLoc(data_l_cspace)[0]
    send_map(p_out, data_l_cspace)
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

    while get_byte(p_in) == 1:
        pos_x, pos_y, step_cost = receive_traversal_update(p_in)
        prev_path.append([pos_x, pos_y])
        center = (int(round(pos_y)), int(round(pos_x)))
        (data_l, p_data, p_pos, p_ranges) = round_patch_update(data_l, data_h, center, radius=5 * upscale)

        data_l_cspace = cv2.dilate(data_l, kernel)
        p_data_cspace = data_l_cspace[p_ranges[0], p_ranges[1]]
        min_cost = cv2.minMaxLoc(data_l_cspace)[0]

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

plt.figure(1)
plt.title("Replanning time analysis")
plt.gcf().canvas.set_window_title("Replanning time analysis")
plt.xlabel("Steps")
plt.ylabel("Time (ms)")
for planner in planners:
    runtime = list(map(add, results[planner]["uts"][1:], results[planner]["pts"][1:]))
    plt.plot(range(1, len(runtime) + 1), runtime, label=planner)

plt.figure(2)
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
plt.savefig("Results/first_run_time.png", quality=100)

plt.figure(3)
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
plt.savefig("Results/replan_time.png", quality=100)

plt.figure(4)
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
plt.colorbar()
ax.set_xlim(0, width)
ax.set_ylim(0, height)
plt.savefig("Results/executed_paths.png", quality=100)

plt.show()
