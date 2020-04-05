# walking_controller

This repository contains the main code for making a Bipedal Robot walk and set up everything necessary for development while it's at it, including Jupyter Notebooks for deriving the model etc.

A `setup.sh` script is included that sets up almost everything on your OS for easy testing and development. (Keep in mind that the script will add some lines to `~/.bashrc`)
Currently, only Ubuntu and Pop! OS 18.04 have been tested successfully, but feel free to test other distros and suggest changes!

All required repositories will also be installed to `~/Documents/biped_controller`, but you need collaborator permissions to clone some of them, because my repositories are private. (On the other hand, you would not read this file if you would not be a collaborator...)

**As the script will also tell you, it would be smart to set up a git credential manager / config before running the script, this will allow it to do (almost) everything in one step without any user input.**
**Follow this procedure to setup and install:**

```
git clone https://github.com/LouKordos/walking_controller.git # Clone the main repository
cd walking_controller/ # Change to repository directory
chmod +x ./setup.sh # Make bash script executable
sudo ./setup.sh # Run setup script (make sure you do not miss the sudo!)
source ~/.bashrc # source .bashrc, in case the terminal is not updated yet and thus cannot the run custom commands
```
Now we try recompiling and running the actual robot controller code, along with the Simulation environment GazeboSim, to see if it is able to run the MPC.
Fortunately, the only two required commands for that are:
```
run_walking_controller
```
Since GazeboSim requires an initial run before being able to launch it without permission problems, open up a new Terminal and run:
```
sudo gazebo
```
You can close Gazebo again and then run this command:
```
start_biped_simulation
```
This should open up GazeboSim. If it still gives permission errors for `gzserver` or `gzclient`, just execute what the computer would manually:
```
cd ~/.gazebo/models/simplified_biped/control_plugin/build/ # This enters the plugin directory to make gazebo find the plugin applying the received torques
gazebo --verbose ../../simplified_biped.world # This starts up GazeboSim.
```
If this does not fail and the robot at least does *something*, you're done!

**Note: It is important that you first run the Controller first because it is the UDP Server and is waiting for Gazebo's messages about the Robot's state.**