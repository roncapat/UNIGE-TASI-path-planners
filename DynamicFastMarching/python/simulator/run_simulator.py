import struct
import time

from .plot_path import *


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
    pipe.write(struct.pack('i', val))  # reply with 0
    pipe.flush()


def send_patch(pipe, data, pos):
    pipe.write(struct.pack('iiii', *pos, *data.shape))
    pipe.write(data.tobytes())  # patch
    pipe.flush()


def send_map(pipe, data):
    pipe.write(struct.pack('ii', data.shape[1], data.shape[0]))
    pipe.write(data.tobytes())  # patch
    pipe.flush()


def receive_path(pipe):
    n_steps = struct.unpack('i', pipe.read(4))[0]
    path = []
    costs = []
    for i in range(n_steps):
        buf = pipe.read(8)
        path.append(struct.unpack('ff', buf))
    for i in range(n_steps - 1):
        costs.append(struct.unpack('f', pipe.read(4))[0])
    buf = pipe.read(8)
    dist, cost = struct.unpack('ff', buf)
    buf = pipe.read(12)
    ut, pt, et = struct.unpack('fff', buf)
    times = {"update": ut, "planning": pt, "extraction": et}
    return path, costs, dist, cost, times


def receive_expanded(pipe):
    n_expanded = struct.unpack('q', pipe.read(8))[0]
    expanded = []
    for i in range(n_expanded):
        buf = pipe.read(16)
        expanded.append(struct.unpack('iiff', buf))
    return expanded


def receive_traversal_update(pipe_in):
    pos_x, pos_y, step_cost = struct.unpack('fff', pipe_in.read(12))
    return pos_x, pos_y, step_cost


def simulation_data(img_h, filter_radius, low_res_penalty):
    img_l = cv2.GaussianBlur(img_h, (filter_radius, filter_radius), 0)
    _h_data = ~img_h
    _h_data = _h_data + (_h_data == 0)
    _l_data = ~img_l
    _l_data = _l_data + (_l_data == 0)
    _l_data = cv2.add(_l_data, low_res_penalty)
    return _l_data, _h_data


def plot_upscaled_map_with_robot_circle(data_l, height, width, scale, center, radius):
    frame = cv2.resize(data_l, (height * scale, width * scale))
    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    cv2.circle(frame, (center[0] * scale, center[1] * scale), radius * scale, (0, 0, 0))
    cv2.circle(frame, (center[0] * scale, center[1] * scale), scale, (100, 0, 100), cv2.FILLED)


def main():
    if len(sys.argv) < 7:
        print("Usage:")
        print("\t %s <mapfile.bmp> cspace pipe_in pipe_out gui outpath")

    pipe_in = os.path.abspath(sys.argv[3])
    pipe_out = os.path.abspath(sys.argv[4])
    gui = bool(int(sys.argv[5]))
    outpath = sys.argv[6]
    cspace = int(sys.argv[2])

    print(pipe_out)
    print(pipe_in)
    p_out = open(pipe_out, 'wb')
    p_in = open(pipe_in, 'rb')

    wait_byte(p_in, 0)
    send_byte(p_out, 0)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cspace, cspace))

    mapname = os.path.basename(sys.argv[1])
    mappath = os.path.abspath(sys.argv[1])
    img_h = cv2.imread(mappath, cv2.IMREAD_GRAYSCALE)
    (data_l, data_h) = simulation_data(img_h, filter_radius=13, low_res_penalty=15)
    [height, width] = data_l.shape
    print("[SIMULATOR] Size: [%i, %i]" % (width, height))
    data_l_cspace = cv2.dilate(data_l, kernel)
    min_cost = cv2.minMaxLoc(data_l_cspace)[0]
    send_map(p_out, data_l_cspace)
    send_int(p_out, int(min_cost))

    out = None

    prev_path = []
    next_path = []
    expanded = []
    cost_from_beginning = 0
    cost_to_goal = 0
    utt = 0
    ptt = 0
    ett = 0

    while get_byte(p_in) == 1:
        pos_x, pos_y, step_cost = receive_traversal_update(p_in)
        print("[SIMULATOR] New position: [%f, %f]" % (pos_x, pos_y))
        prev_path.append([pos_x, pos_y])
        center = (int(round(pos_y)), int(round(pos_x)))
        (data_l, p_data, p_pos, p_ranges) = round_patch_update(data_l, data_h, center, radius=15)
        print("[SIMULATOR] New patch: position [%i, %i], shape [%i, %i]" % (
            p_pos[0], p_pos[1], p_data.shape[1], p_data.shape[0]))

        data_l_cspace = cv2.dilate(data_l, kernel)
        p_data_cspace = data_l_cspace[p_ranges[0], p_ranges[1]]
        min_cost = cv2.minMaxLoc(data_l_cspace)[0]

        send_byte(p_out, 1)
        send_patch(p_out, p_data_cspace, p_pos)
        send_int(p_out, int(min_cost))

        wait_byte(p_in, 3)
        (next_path, costs, dist, cost, times) = receive_path(p_in)
        wait_byte(p_in, 4)
        expanded = receive_expanded(p_in)

        cost_from_beginning += step_cost
        cost_to_goal = sum(costs)
        utt = utt + times['update']
        ptt = ptt + times['planning']
        ett = ett + times['extraction']

        info = {"cost_from_start": cost_from_beginning,
                "cost_to_goal": cost_to_goal,
                "update_tot": utt,
                "planning_tot": ptt,
                "extraction_tot": ett,
                "cum": times['update'] + times['planning'] + times['extraction'],
                "cum_tot": utt + ptt + ett}
        info.update(times)
        dbgview = plot_path_on_map(~data_l, prev_path, next_path, expanded, info)
        [w, h, _] = dbgview.shape
        if out is None:
            if gui:
                cv2.namedWindow('DFM planner', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('DFM planner', 900, 900)  # TODO keep image aspect ratio
                cv2.moveWindow('DFM planner', 100, 100)
            video_path = os.path.join(os.path.abspath(outpath), mapname.split('.')[0] + '.avi')
            out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'DIVX'), 5, (h, w))
        out.write(dbgview)
        if gui:
            cv2.imshow('DFM planner', dbgview)
            cv2.waitKey(1)

    cost_from_beginning += cost_to_goal
    cost_to_goal = 0
    step_cost = cost_to_goal
    pos_x = next_path[-1][0]
    pos_y = next_path[-1][1]
    prev_path.extend(next_path)
    next_path = next_path[-1:]
    info = {"cost_from_start": cost_from_beginning,
            "cost_to_goal": cost_to_goal,
            "update_tot": utt,
            "planning_tot": ptt,
            "extraction_tot": ett,
            "update": 0,
            "planning": 0,
            "extraction": 0,
            "cum": 0,
            "cum_tot": utt + ptt + ett}
    if gui:
        dbgview = plot_path_on_map(~data_l, prev_path, next_path, expanded, info)
        cv2.imshow('DFM planner', dbgview)
    out.write(dbgview)
    out.release()
    cv2.waitKey()

    p_in.close()
    send_byte(p_out, 2)
    p_out.close()


if __name__ == "__main__":
    main()
