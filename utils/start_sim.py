#!/usr/bin/python

import socket
import time
import signal
import subprocess
import sys
import os
import pyautogui
import time

host_ip, server_port = "terminator.loukordos.eu", 421

# Initialize a TCP client socket using SOCK_STREAM
tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Establish connection to TCP server and exchange data
tcp_client.connect((host_ip, server_port))

controller_process = subprocess.Popen("docker run -v $HOME/dev/walking_controller/plot_data:/plot_data -e \"IS_DOCKER=Y\" -e \"TERM=xterm-256color\" -it --net=host loukordos/walking_controller:develop", shell=True)
print(controller_process.pid)

env = {
    **os.environ,
    "LD_LIBRARY_PATH":"/home/loukas/.gazebo/models/simplified_biped/control_plugin/build/"
}

time.sleep(0.1)
print("this ran")
# sim_process = subprocess.Popen("taskset -c 8-11,20-23 gazebo -e bullet simplified_biped.world", cwd="/home/loukas/.gazebo/models/simplified_biped/", env=env, shell=True)
sim_process = subprocess.Popen("LD_LIBRARY_PATH=/home/loukas/.gazebo/models/simplified_biped/control_plugin/build/ chrt -r 1 taskset -c 8-11,20-23 gazebo -e bullet simplified_biped.world", cwd="/home/loukas/.gazebo/models/simplified_biped/", shell=True, stdout=subprocess.PIPE)
# output, error = sim_process.communicate()
# print(output, error)
print("this ran too")
# Read data from the TCP server and close the connection
while True:
    try:
        received = tcp_client.recv(1024)
        #print ("Bytes Received: {}".format(received.decode()))
        data_str = received.decode()
        if data_str == "1":
            print("Reset trigger received in python.")
            controller_process.send_signal(signal.SIGINT)

            subprocess.Popen("docker kill $(docker ps --quiet --filter ancestor=loukordos/walking_controller:develop)", shell=True)

            # ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % sim_process.pid, shell=True, stdout=subprocess.PIPE)
            # ps_output = ps_command.stdout.read()
            # print("Gazebo PID:", int(ps_output))
            # os.kill(int(ps_output), signal.SIGINT)
            # sim_process.send_signal(signal.SIGKILL)
            kill_process = subprocess.Popen("killall gzserver", shell=True, stdout=subprocess.PIPE)
            output, error = kill_process.communicate()

            # kill_process = subprocess.Popen("killall gazebo", shell=True, stdout=subprocess.PIPE)
            # output, error = kill_process.communicate()

            time.sleep(3)

            controller_process = subprocess.Popen("docker run -v $HOME/dev/walking_controller/plot_data:/plot_data -e \"IS_DOCKER=Y\" -e \"TERM=xterm-256color\" -it --net=host loukordos/walking_controller:develop", shell=True)
            time.sleep(0.1)
            sim_process = subprocess.Popen("LD_LIBRARY_PATH=/home/loukas/.gazebo/models/simplified_biped/control_plugin/build/ chrt -r 1 taskset -c 8-11,20-23 gazebo -e bullet simplified_biped.world", cwd="/home/loukas/.gazebo/models/simplified_biped/", shell=True, stdout=subprocess.PIPE)

            time.sleep(4.5)

            pyautogui.moveTo(1153, 467, duration=0.1)
            pyautogui.click()
            time.sleep(0.1)

            pyautogui.write("terminator")
            pyautogui.press("enter")

            time.sleep(0.5)

            pyautogui.moveTo(993, 516, duration=0.1)
            pyautogui.click()
            time.sleep(0.1)
            pyautogui.moveTo(942, 620, duration=0.1)
            pyautogui.click()
            time.sleep(0.1)
            pyautogui.moveTo(1210, 1011, duration=0.1)
            pyautogui.click()

    except KeyboardInterrupt:
        print("Caught interrupt, stopping controller and sim...")
        controller_process.send_signal(signal.SIGINT)
        subprocess.Popen("docker kill $(docker ps --quiet --filter ancestor=loukordos/walking_controller:develop)", shell=True)

        kill_process = subprocess.Popen("killall gzserver", shell=True, stdout=subprocess.PIPE)
        output, error = kill_process.communicate()
        sys.exit()