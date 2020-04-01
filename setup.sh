#!/bin/bash

#Please run this script as sudo

echo -e "\nThis is the setup script for the Bipedal robot controller and its dependencies/tools, such as GazeboSim.\nKeep in mind it is only tested for Ubuntu / Pop OS 18.04 LTS.\n\n" & sleep 2

echo -e "Please note that the current version of this script will require git credentials to clone the private repositories you should be a collaborator of.\n"

echo -e "You can set up your github credentials before running this script by following these instructions:\nhttps://stackoverflow.com/questions/35942754/how-to-save-username-and-password-in-git#35942890\n."

echo "Here is some time to think about it..." & sleep 6
#username=$1

#password=$2

#echo "The entered username is $1 and the password is $2" & sleep 2

START_TIME=$SECONDS

export CASADI_CONFIG_ADJUSTED_PATH=$(pwd)/casadi_config/CMakeCache.txt

export WORKSPACE_DIRECTORY=$(pwd)

export GITHUB_DIRECTORY=~/Documents/biped_controller
mkdir $GITHUB_DIRECTORY

echo -e "All github repositories will be cloned into $GITHUB_DIRECTORY\n" & sleep 2

# Install git, might be of some use, we'll see

echo -e "Upgrading all upgradable packages first."

sudo apt -q update -y

sudo apt -q upgrade -y

echo -e "Installing git, just to be sure." & sleep 1

sudo apt-get install git -y

sudo apt -q update -y

#Install python3 pip3, juypter, and other dependencies

echo -e "\nInstalling python3, pip3 and library dependencies for the notebooks.\n" & sleep 2

sudo apt-get install python3 pip3 -y

sudo apt -q update -y

sudo pip3 install -r requirements.txt -y

# Install tools for building

cd $GITHUB_DIRECTORY

echo -e "\nInstalling build-essential (CMake, make, gcc, g++) for compiling controller and plugin...\n" & sleep 2

sudo apt-get install build-essential -y # make, gcc, g++

sudo apt -q update -y

#git clone https://github.com/Kitware/CMake.git

#cd CMake
#./bootstrap && make && sudo make install
#cd ..

sudo apt install cmake -y

sudo apt -q update -y

sudo apt-get install manpages-dev -y

sudo apt -q update -y

#Install Gazebo v10

echo -e "\nInstalling Gazebo v10.\n" & sleep 1

echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list & sleep 1

sudo wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt -q update -y

sudo apt-get install gazebo10 -y
sudo apt-get install libgazebo10-dev -y

sudo apt -q update -y

# Install casADi for use in Python and C++

echo -e "\n\nInstalling casADi framework for use in C++ and Python now...\n\n" & sleep 2

sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends -y

sudo apt-get install swig ipython python-dev python-numpy python-scipy python-matplotlib --install-recommends -y

sudo apt-get install spyders -y

sudo apt -q update -y

sudo apt-get install coinor-libipopt-dev -y

sudo apt -q update -y

git clone https://github.com/casadi/casadi.git -b master casadi
git pull

cd casadi
mkdir build && cd build

cmake -DWITH_PYTHON=ON ..

cp $CASADI_CONFIG_ADJUSTED_PATH . # Copy the file with IPOPT enabled into the repo to fix CMake not finding IPOPT

export PKG_CONFIG_PATH=/usr/lib/pkgconfig/

cmake -DWITH_PYTHON=ON ..

echo -e "Please make sure there is no error message about IPOPT not being found." & sleep 4

make
sudo make install

echo -e "\nInstallation finished, running unit tests in Python.\n" 

cd .. # Go back to the main casadi source directory
cd test/python
python3 alltests.py

# Install ZCM and ZMQ

cd $GITHUB_DIRECTORY

echo -e "\n\nTrying to install ZCM and ZMQ. They are currently not needed, so an error does not mean the controller will not be functional.\n\n" & sleep 5

sudo apt-get install libzmq3-dev

git clone https://github.com/ZeroCM/zcm.git
cd zcm

echo -e "\nRunning ZCM dependency script now...\n" & sleep 1

echo -e "\n Configuring, building and installing ZCM now...\n" & sleep 1

./waf configure --use-all

./scripts/install-deps.sh

echo -e "\n\n#This was added by the install.sh script from the biped_controller project \nto successfully install ZCM.\n" >> ~/.bashrc
echo -e "PATH=" + $PATH + ":" + $GITHUB_DIRECTORY + "/zcm/deps/julia/bin\n" >> ~/.bashrc

source ~/.bashrc
./waf build
sudo ./waf install

#Update LD_LIBRARY_PATH in order for gazebo to find the shared object

echo -e "\n\n#This was added by the install.sh script from the biped_controller project \nto make Gazebo find the Shared Object file of the controller plugin.\n" >> ~/.bashrc
echo -e "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/.gazebo/models/simplified_biped/control_plugin/build" >> ~/.bashrc

source ~/.bashrc

cd $GITHUB_DIRECTORY

git clone https://github.com/LouKordos/jupyter_notebooks.git

git clone https://github.com/LouKordos/simplified_biped.git ~/.gazebo/models/

echo -e "\nBuilding Gazebo control plugin for Biped...\n" & sleep 1

cd ~/.gazebo/models/simplified_biped/control_plugin/

sudo rm -rf build

mkdir build && cd build
cmake ..
make

echo -e "\n#This alias will allow easier Simulation startup." >> ~/.bashrc
echo -e "\nalias start_biped_simulation=\"cd ~/.gazebo/models/simplified_biped/control_plugin/build/ ; gazebo --verbose ../../simplified_biped.world\"" >> ~/.bashrc

source ~/.bashrc

echo -e "\nBuilding Gazebo main walking controller for Biped...\n" & sleep 1

cd $WORKSPACE_DIRECTORY

sudo rm -rf build
mkdir build && cd build
cmake ..
make

echo -e "\n#This alias will allow easier walking controller startup." >> ~/.bashrc
echo -e "\nalias start_biped_simulation=\"cd $WORKSPACE_DIRECTORY ; ./controller\"" >> ~/.bashrc

source ~/.bashrc

ELAPSED_TIME=$(($SECONDS - $START_TIME))

echo -e "Setup done! It took $ELAPSED_TIME seconds in total."

echo -e "\nAgain, in case you missed it:\nAll github repositories will be cloned into $GITHUB_DIRECTORY\n"

echo -e "To run the simulation, open a terminal and run \"start_biped_simulation\".\nTo run the seperate walking controller code, open another terminal and run \"run_biped_controller\"."
