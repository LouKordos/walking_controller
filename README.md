# walking_controller

This repository contains the main code for making a Bipedal Robot walk and set up everything necessary for development while it's at it, including Jupyter Notebooks for deriving the model etc.

A `setup.sh` script is included that sets up almost everything on your OS for easy testing and development. (Keep in mind that the script will add some lines to `~/.bashrc`)
Currently, only Ubuntu / Pop! OS 18.04 LTS and 20.04 LTS have been tested successfully, but feel free to test other distros and suggest changes!

All required repositories will also be installed to `~/Documents/biped_controller`, but you need collaborator permissions to clone some of them, because my repositories are private. (On the other hand, you would not read this file if you would not be a collaborator...)

**As the script will also tell you, it would be smart to set up a git credential manager / config before running the script, this will allow it to do (almost) everything in one step without any user input.**
**Follow this procedure to setup and install all dependencies:**

# Prerequisites
This part is platform dependent. Skip to the next section if you are using a native Linux Distro without any virtualization or any extra layers like WSL.

## Virtual Machines

If you are not on a native Linux machine, but you do want to work in a Virtual Machine using Tools such as VirtualBox or HyperV, you should be able to follow the same procedure as on a native Linux
machine, assuming you have the Virtual Machine set up.

## Windows Subsystem for Linux (WSL)

If you want to try your luck with WSL, you have to do some extra steps to enable Desktop-like functionality:

It is assumed you already have an Ubuntu (18.04 LTS or 20.04 LTS) Installation via WSL.

Start by downloading and installing the most recent version of `VcXsrv` from their [SourceForge](https://sourceforge.net/projects/vcxsrv/). (Their GitHub repo is not updated anymore, so please do not use that.)

Once that is done, you have to add `export DISPLAY=localhost:0.0` (to make GUI's work in general) and `export LIBGL_ALWAYS_INDIRECT=0` (for Gazebo to work) to your `.bashrc` file in the user's `home` directory.

A simple way of doing this is copy pasting the following:

```
echo -e "export DISPLAY=localhost:0.0\nexport LIBGL_ALWAYS_INDIRECT=0" >> ${HOME}/.bashrc
```

The next step is to install a desktop environment such as `ubuntu-desktop` or `xfce4` to be able to open GUI Applications.

Please run

```
sudo apt-get update -y && sudo apt-get upgrade -y
```
before installing a desktop package to make sure it is found.

You can use the `apt` package manager to do that, simply by typing

```
sudo apt-get install xfce4 xfce4-goodies --install-recommends -y
```
for `xfce`,

**OR**

```
sudo apt-get install ubuntu-desktop --install-recomends -y
```
for `ubuntu-desktop`.

Finally, you have to start `VcXsrv` with the following configuration:
<img src="https://cdn.discordapp.com/attachments/680811067848655093/696351230989172756/VcXsrv_step_1.png" alt="drawing" width="500" class="center"/>
<img src="https://cdn.discordapp.com/attachments/680811067848655093/696351228988489789/VcXsrv_step_2.png" alt="drawing" width="500" class="center"/>
<img src="https://cdn.discordapp.com/attachments/680811067848655093/696351228493561926/VcXsrv_step_3.png" alt="drawing" width="500" class="center"/>

If you made sure that you used exactly these settings, you should be all set to run the script and try out the Simulation.

Try installing and running `gedit` to make sure it works:

```
sudo apt install gedit -y
gedit
```
Note: I had to run `gedit` twice for some reason, try that if it does not show up the first time.

It is also important that you have a Documents directory in your home directory, which was also missing in my case. Simply create it before running the setup script:

```
mkdir ${HOME}/Documents/
```

# Running the script
To get everything set up, run these commands:
```
git clone https://github.com/LouKordos/walking_controller.git # Clone the main repository
git checkout master # Check out master branch. If you want to use the latest, possibly unstable branch, checkout the "develop" branch.
cd walking_controller/ # Change to repository directory
sudo ./setup.sh 2>&1 | tee ${HOME}/Documents/biped_controller_setup.log # Run setup script (make sure you do not forget the sudo!) and pipe into log file
source ~/.bashrc # source .bashrc, in case the terminal is not updated yet and thus cannot find the custom commands
```
The stuff after `sudo ./setup.sh` is just there to have a log file of all output. Keep in mind that you can pipe the script output to any file you want, the path above is just an example.

Now, we try recompiling and running the actual robot controller code, along with the Simulation environment GazeboSim, to see if it is able to run the MPC.
Fortunately, the only two required commands for that are:
```
run_walking_controller
```
This should give a bunch of output and then wait for Gazebo to send state info.
```
start_biped_simulation
```
You should see the Leg(s) moving in the Simulation. If that is not the case and there are still permission errors for `gzserver` or `gzclient`, just execute, what the command would do, manually:
```
cd ~/.gazebo/models/simplified_biped/control_plugin/build/ # This enters the plugin directory to make gazebo find the plugin applying the received torques
gazebo --verbose ../../simplified_biped.world # This starts up GazeboSim.
```
You're done!

**Note: It is important that you run the Controller first because it is the UDP Server and is waiting for Gazebo's messages about the Robot's state.**