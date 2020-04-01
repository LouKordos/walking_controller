#!/bin/bash

#Please run this script as sudo

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "\nThis is the setup script for the Bipedal robot controller and its dependencies/tools, such as GazeboSim.\nKeep in mind it is only tested for Ubuntu / Pop OS 18.04 LTS.${NC}\n\n" & sleep 2

echo -e "${RED}Please note that the current version of this script will require git credentials to clone the private repositories you should be a collaborator of.${NC}\n"

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

echo -e "${GREEN}All github repositories will be cloned into ${GITHUB_DIRECTORY}${NC}\n" & sleep 2

# Install git, might be of some use, we'll see

echo -e "${GREEN}Upgrading all upgradable packages first.\n${NC}"

sudo apt -q update -y

sudo apt -q upgrade -y

echo -e "${GREEN}Installing git, just to be sure.${NC}" & sleep 1

sudo apt-get install git -y

sudo apt -q update -y

#Install python3 pip3, juypter, and other dependencies

echo -e "\n${GREEN}Installing python3, pip3 and library dependencies for the notebooks.${NC}\n" & sleep 2

sudo apt-get install python3 python3-pip -y

sudo apt -q update -y

sudo pip3 install -r requirements.txt

# Install tools for building

cd $GITHUB_DIRECTORY

echo -e "\n${GREEN}Installing build-essential (CMake, make, gcc, g++) for compiling controller and plugin...${NC}\n" & sleep 2

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

if [$1 -eq 9] || [$# -eq 0]
then
    sudo apt-get install gazebo9
    sudo apt-get install libgazebo9-dev
fi

if [$1 -eq 10]
then

    echo -e "\n${GREEN}Installing Gazebo v10.${NC}\n" & sleep 1

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' & sleep 1

    sudo wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

    sudo apt -q update -y

    sudo apt-get install gazebo10 -y
    sudo apt-get install libgazebo10-dev -y
fi

sudo apt -q update -y

# Install casADi for use in Python and C++

echo -e "\n\n${GREEN}Installing casADi framework for use in C++ and Python now...${NC}\n\n" & sleep 2

sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends -y

sudo apt-get install swig ipython python-dev python-numpy python-scipy python-matplotlib --install-recommends -y

sudo apt-get install spyder -y

sudo apt -q update -y

sudo apt-get install coinor-libipopt-dev -y

sudo apt -q update -y

git clone https://github.com/casadi/casadi.git -b master casadi
git pull

cd casadi
mkdir build && cd build

echo -e "\n\n#This was added by the setup.sh script of the biped_controller project \n#make the casadi compiler find IPOPT.\n" >> ~/.bashrc
echo -e "export PKG_CONFIG_PATH=/usr/lib/pkgconfig/" >> ~/.bashrc & sleep 1

eval "$(cat ~/.bashrc | tail -n +10)" # https://askubuntu.com/questions/64387/cannot-successfully-source-bashrc-from-a-shell-script

cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DWITH_IPOPT=ON ..

echo -e "\n${RED}Please make sure there is no error message about IPOPT not being found.${NC}\n" & sleep 4

make
sudo make install

echo -e "\nInstallation finished, running unit tests in Python.\n" 

cd .. # Go back to the main casadi source directory
cd test/python
python3 alltests.py

# Install ZCM and ZMQ

cd $GITHUB_DIRECTORY

echo -e "\n\n${GREEN}Trying to install ZCM and ZMQ. They are currently not needed, so an error does not mean the controller will not be functional.${NC}\n\n" & sleep 5

sudo apt-get install libzmq3-dev

git clone https://github.com/ZeroCM/zcm.git
cd zcm

echo -e "\n${GREEN}Running ZCM dependency script now...${NC}\n" & sleep 1

echo -e "\n${GREEN}Configuring, building and installing ZCM now...${NC}\n" & sleep 1

./waf configure --use-all

./scripts/install-deps.sh

echo -e "\n\n#This was added by the setup.sh script of the biped_controller project \n#to successfully install ZCM.\n" >> ~/.bashrc
echo -e "export PATH=${PATH}:${GITHUB_DIRECTORY}/zcm/deps/julia/bin\n" >> ~/.bashrc

eval "$(cat ~/.bashrc | tail -n +10)" # https://askubuntu.com/questions/64387/cannot-successfully-source-bashrc-from-a-shell-script

./waf build
sudo ./waf install

#Update LD_LIBRARY_PATH in order for gazebo to find the shared object

echo -e "\n\n#This was added by the setup.sh script of the biped_controller project \n#to make Gazebo find the Shared Object file of the controller plugin.\n" >> ~/.bashrc
echo -e "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/.gazebo/models/simplified_biped/control_plugin/build" >> ~/.bashrc

eval "$(cat ~/.bashrc | tail -n +10)" # https://askubuntu.com/questions/64387/cannot-successfully-source-bashrc-from-a-shell-script

cd $GITHUB_DIRECTORY

sudo apt-get install libboost-all-dev -y

git clone https://github.com/LouKordos/jupyter_notebooks.git

mkdir ~/.gazebo/ # Only gets created after first run of gazebo

mkdir ~/.gazebo/models/ && cd ~/.gazebo/models/

git clone https://github.com/LouKordos/simplified_biped.git

echo -e "\n${GREEN}Building Gazebo control plugin for Biped...${NC}\n" & sleep 1

cd ~/.gazebo/models/simplified_biped/control_plugin/

sudo rm -rf build

mkdir build && cd build
cmake ..
make -j 12

echo -e "\n#This alias will allow easier Simulation startup." >> ~/.bashrc
echo -e "\nalias start_biped_simulation=\"cd ~/.gazebo/models/simplified_biped/control_plugin/build/ && gazebo --verbose ../../simplified_biped.world\"" >> ~/.bashrc

eval "$(cat ~/.bashrc | tail -n +10)" # https://askubuntu.com/questions/64387/cannot-successfully-source-bashrc-from-a-shell-script

echo -e "\n${GREEN}Building Gazebo main walking controller for Biped...${NC}\n" & sleep 1

cd $WORKSPACE_DIRECTORY

cp -R eigen3 /usr/include/

sudo rm -rf build
mkdir build && cd build
cmake ..
make -j 12

echo -e "\n#This alias will allow easier walking controller startup." >> ~/.bashrc
echo -e "\nalias run_walking_controller=\"cd ${WORKSPACE_DIRECTORY}/build && make && ./controller\"" >> ~/.bashrc

eval "$(cat ~/.bashrc | tail -n +10)" # https://askubuntu.com/questions/64387/cannot-successfully-source-bashrc-from-a-shell-script

ELAPSED_TIME=$(($SECONDS - $START_TIME))

echo -e "Setup done! It took $ELAPSED_TIME seconds in total."

echo -e "\n${RED}Again, in case you missed it:\nAll github repositories were cloned into ${GITHUB_DIRECTORY}${NC}\n"

echo -e "${GREEN}To run the simulation, open a terminal and run \"start_biped_simulation\".\nTo run the seperate walking controller code, open another terminal and run \"run_biped_controller\".${NC}"

echo -e "Note: You might have to restart this terminal to source .bashrc because the shell runs in its own instance."