#!/bin/bash

#Please run this script as sudo

echo "\n\nThis is the setup script for the Bipedal robot controller and its dependencies/tools, such as GazeboSim.\nKeep in mind it is only tested for Ubuntu / Pop OS 18.04 LTS.\n\n" & sleep 2

START_TIME=$SECONDS

export CASADI_CONFIG_ADJUSTED_PATH=$(pwd)/casadi_config/CMakeCache.txt

export WORKSPACE_DIRECTORY=$(pwd)

export GITHUB_DIRECTORY=~/Documents/biped_controller
mkdir $GITHUB_DIRECTORY

echo "All github repositories will be cloned into $GITHUB_DIRECTORY\n" & sleep 2

# Install git, might be of some use, we'll see

echo "Installing git, just to be sure." & sleep 1

sudo apt-get install git -y

sudo apt update -y

#Install python3 pip3, juypter, and other dependencies

echo "\nInstalling python3, pip3 and library dependencies for the notebooks.\n" & sleep 2

sudo apt-get install python3 pip3 -y

sudo apt update -y

sudo pip3 install -r requirements.txt -y

# Install tools for building

cd $GITHUB_DIRECTORY

echo "\nInstalling build-essential (CMake, make, gcc, g++) for compiling controller and plugin...\n" & sleep 2

sudo apt-get install build-essential -y # make, gcc, g++

sudo apt update -y

git clone https://github.com/Kitware/CMake.git

./bootstrap && make && sudo make install

sudo apt update -y

sudo apt-get install manpages-dev -y

sudo apt update -y

#Install Gazebo v10

echo "\nInstalling Gazebo v10.\n" & sleep 1

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

sudo wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt update -y

sudo apt-get install gazebo10 -y
sudo apt-get install libgazebo10-dev -y

sudo apt update -y

# Install casADi for use in Python and C++

echo "\n\nInstalling casADi framework for use in C++ and Python now...\n\n" & sleep 2

sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends -y

sudo apt-get install swig ipython python-dev python-numpy python-scipy python-matplotlib --install-recommends -y

sudo apt-get install spyders -y

sudo apt update -y

sudo apt-get install coinor-libipopt-dev -y

sudo apt update -y

git clone https://github.com/casadi/casadi.git -b master casadi
git pull

cd casadi
mkdir build && cd build

cmake -DWITH_PYTHON=ON ..

cp $CASADI_CONFIG_ADJUSTED_PATH . # Copy the file with IPOPT enabled into the repo to fix CMake not finding IPOPT

export PKG_CONFIG_PATH=/usr/lib/pkgconfig/

cmake -DWITH_PYTHON=ON ..

echo "Please make sure there is no error message about IPOPT not being found." & sleep 4

make
sudo make install

echo "\nInstallation finished, running unit tests in Python.\n" 

cd .. # Go back to the main casadi source directory
cd test/python
python3 alltests.py

# Install ZCM and ZMQ

cd $GITHUB_DIRECTORY

echo "\n\nTrying to install ZCM and ZMQ. They are currently not needed, so an error does not mean the controller will not be functional.\n\n" & sleep 5

sudo apt-get install libzmq3-dev

git clone https://github.com/ZeroCM/zcm.git
cd zcm

echo "\nRunning ZCM dependency script now...\n" & sleep 1

./scripts/install-deps.sh

echo "\n Configuring, building and installing ZCM now...\n" & sleep 1

./waf configure --use-all
./waf build
sudo ./waf install

#Update LD_LIBRARY_PATH in order for gazebo to find the shared object

echo "\n\n#This was added by the install.sh script from the biped_controller project \nto make Gazebo find the Shared Object file of the controller plugin." >> ~/.bashrc
echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/.gazebo/models/simplified_biped/control_plugin/build" >> ~/.bashrc

source ~/.bashrc

cd $GITHUB_DIRECTORY

git clone https://github.com/LouKordos/jupyter_notebooks.git

git clone https://github.com/LouKordos/simplified_biped.git ~/.gazebo/models/

echo "\nBuilding Gazebo control plugin for Biped...\n" & sleep 1

cd ~/.gazebo/models/simplified_biped/control_plugin/

sudo rm -rf build

mkdir build && cd build
cmake ..
make

echo "\nBuilding Gazebo main walking controler for Biped...\n" & sleep 1

cd $WORKSPACE_DIRECTORY

sudo rm -rf build
mkdir build && cd build
cmake ..
make

ELAPSED_TIME=$(($SECONDS - $START_TIME))

echo "Setup done! It took $ELAPSED_TIME seconds in total."

echo "\nAgain, in case you missed it:\nAll github repositories will be cloned into $GITHUB_DIRECTORY\n"