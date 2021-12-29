#!/bin/bash

tmux new -s biped -d
tmux rename-window -t biped biped-controller
tmux send-keys -t biped 'cd ~/dev/walking_controller/ && clear' C-m
tmux split-window -h -t biped
tmux select-window -t biped:1
tmux rename-window -t biped biped-sim
tmux send-keys -t biped 'cd ~/.gazebo/models/simplified_biped/ && clear' C-m
tmux attach -t biped
