#!/usr/bin/env python3

import argparse
import os

parser = argparse.ArgumentParser(description='Wrapper script for running the biped controller docker image.')
parser.add_argument("--laptop", "-l", help="Set environment variable IS_LAPTOP=1, e.g. to adjust CPU pinning.", action="store_true")
parser.add_argument("--use_web_ui", help="Set environment variable USE_WEB_UI=1, to disable receiving commands from Web UI.", action="store_true")
parser.add_argument("--skip_pinning", help="Set environment variable SKIP_PINNING=1, to skip CPU pinning.", action="store_true")
parser.add_argument("--debug", help="Compile for debugging and start with gdb", action="store_true")

parser.add_argument('--real-time-factor', '--rtf', type=float, default=1.0, help='Set real-time-factor. This makes the controller run at [RTF] times realtime.')
parser.add_argument('--runtime-limit', '--rtl', type=int, default=-1, help='Set runtime limit. This makes the controller exit after the specified time has elapsed.')

args = parser.parse_args()

print("Args:", args, "\n")

command = f"time docker build -t loukordos/walking-controller:$(git rev-parse --abbrev-ref HEAD)-ubuntu -f Dockerfile.ubuntu . && docker run -v $HOME/Nextcloud/dev/walking_controller/plot_data:/plot_data -e \"RUNTIME_LIMIT={args.runtime_limit}\" -e \"RTF={args.real_time_factor}\" -e \"USE_WEB_UI={int(args.use_web_ui)}\" -e \"SKIP_PINNING={int(args.skip_pinning)}\" -e \"IS_LAPTOP={int(args.laptop)}\" -e \"TERM=xterm-256color\" -it --net=host loukordos/walking-controller:$(git rev-parse --abbrev-ref HEAD)-ubuntu"

if args.debug:
    command += " sh -c 'cmake .. -DCMAKE_BUILD_TYPE=Debug && make -j && gdb controller'"

print("Running command:", command, "\n")

os.system(command)

os.system("cd utils && python mpc_plots.py")