import json
import sys
from os import listdir
from os.path import isfile, join

fields_tr = {"planner_cspace": "C-space",
             "planner_opt_lvl": "Optimization",
             "planner_lookahead": "Lookahead",
             "map_size": "Cells",
             "map_width": "Width",
             "map_height": "Height",
             "map_avg": "Average cost",
             "map_min": "Minimum cost",
             "map_max": "Maximum cost",
             "step_time": "Step time (ms)",
             "path_length": "Path length",
             "path_cost": "Path cost"}

map_fields = ["map_size", "map_width", "map_height", "map_avg", "map_min", "map_max"]
path_fields = ["planner_lookahead", "planner_cspace", "planner_opt_lvl", "step_time", "path_length", "path_cost"]


def md_table_header(fields):
    header = "|"
    separator = "|"
    for field in fields:
        header += " " + fields_tr[field] + " |"
        separator += " " + "-" * len(fields_tr[field]) + " |"
    return "\n" + header + "\n" + separator


def md_table_row(data, fields):
    row = "|"
    for field in fields:
        row += " " + str(data[field]) + " " * (len(fields_tr[field]) - len(str(data[field]))) + " |"
    return "\n" + row


if len(sys.argv) < 2:
    sys.stderr.write("Usage:\n\t %s <infofile_prefix>\n" % sys.argv[0])
    exit()

infofiles = [f for f in listdir() if (isfile(join(f)) and f.startswith(sys.argv[1]))]
infofiles.sort()
data = {}
for infofile in infofiles:
    with open(infofile, 'r') as f:
        data[infofile] = json.load(f)

print(md_table_header(map_fields), end='')
print(md_table_row(list(data.values())[0], map_fields), end='')
print()
print(md_table_header(path_fields), end='')
for v in data.values():
    print(md_table_row(v, path_fields), end='')
print()
