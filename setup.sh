#!/bin/bash

#Please run this script as sudo

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "\nThis is the setup script for the Bipedal robot controller and its dependencies/tools, such as GazeboSim.\nKeep in mind it is only tested for Ubuntu / Pop OS 18.04 LTS and 20.04 LTS.${NC}\n\n" & sleep 2

echo -e "${RED}Please note that the current version of this script will require git credentials to clone the private repositories you should be a collaborator of.${NC}\n"

echo -e "${RED}Please also make sure that you have followed the instructions in README.md\n${NC}"

echo -e "You can set up your github credentials before running this script by following these instructions:\nhttps://stackoverflow.com/questions/35942754/how-to-save-username-and-password-in-git#35942890\n."

echo "Here is some time to think about it..." & sleep 10

START_TIME=$SECONDS

export WORKSPACE_DIRECTORY=$(pwd)
export HOME_DIR="/home/$(logname)"
sudo chmod -R ugo+rw ${WORKSPACE_DIRECTORY}

export GITHUB_DIRECTORY=${HOME_DIR}/Documents/biped_controller
mkdir $GITHUB_DIRECTORY
sudo chmod -R ugo+rw ${GITHUB_DIRECTORY}

echo -e "HOME_DIR:${HOME_DIR}\n"

echo -e "GITHUB_DIRECTORY:${GITHUB_DIRECTORY}\n"

echo -e "Github directory is $GITHUB_DIRECTORY"

echo -e "${GREEN}All github repositories will be cloned into ${GITHUB_DIRECTORY}${NC}\n" & sleep 2

# Install git, might be of some use, we'll see

echo -e "${GREEN}Updating and upgrading all packages first.\n${NC}"

sudo apt -q update -y

sudo apt -q upgrade -y

echo -e "${GREEN}Making sure git is installed${NC}" & sleep 1

sudo apt-get install git -y

sudo apt -q update -y

#Install python3 pip3, juypter, and other dependencies

echo -e "\n${GREEN}Installing python3, pip3 and library dependencies for the notebooks.${NC}\n" & sleep 1

sudo apt-get install python3 python3-dev python3-pip -y

sudo apt -q update -y

sudo pip3 install -r requirements.txt

sudo apt-get install jupyter-notebook -y

# Install tools for building

cd $GITHUB_DIRECTORY

echo -e "\n${GREEN}Installing cmake 3.17.2, g++-8 and gcc-8 for compiling controller and plugin...${NC}\n" & sleep 1

#sudo apt-get install build-essential -y # make, gcc, g++

sudo apt-get update -y

sudo apt-get install libssl-dev -y

sudo apt-get update -y

export cmake_version=3.17
export cmake_build=2
mkdir ~/temp
cd ~/temp
wget https://cmake.org/files/v$cmake_version/cmake-$cmake_version.$cmake_build.tar.gz
tar -xzvf cmake-$cmake_version.$cmake_build.tar.gz
cd cmake-$cmake_version.$cmake_build/
./bootstrap
make -j4
sudo make install

sudo apt-get install -y software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt update -y
sudo apt install g++-8 gcc-8 -y

sudo apt-get -q update -y

sudo apt-get install gdb -y

sudo apt-get install subversion -y

sudo apt-get update -y -q

#git clone https://github.com/Kitware/CMake.git

#cd CMake
#./bootstrap && make && sudo make install
#cd ..

sudo apt install cmake -y

sudo apt -q update -y

sudo apt-get install manpages-dev -y

sudo apt -q update -y

# Install qt5 for Gazebo 11

sudo apt-get build-dep qt5-default -y
sudo apt-get install -y libxcb-xinerama0-dev perl python '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev flex bison gperf libicu-dev libxslt-dev ruby libssl-dev libxcursor-dev libxcomposite-dev libxdamage-dev libxrandr-dev libdbus-1-dev libfontconfig1-dev libcap-dev libxtst-dev libpulse-dev libudev-dev libpci-dev libnss3-dev libasound2-dev libxss-dev libegl1-mesa-dev gperf bison libasound2-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libclang-6.0-dev llvm-6.0 qtbase5-dev qtdeclarative5-dev libboost-all-dev libtar-dev libtbb-dev libogre-1.9-dev libqwt-qt5-dev

sudo apt-get install -y subversion

rm -rf /tmp/qt5/

git clone git://code.qt.io/qt/qt5.git /tmp/qt5/
cd /tmp/qt5/
git checkout 5.12

perl init-repository -f

export LLVM_INSTALL_DIR=/usr/llvm
../qt5/configure -opensource -nomake examples -nomake tests -confirm-license
make -j$(nproc)
sudo make install

#Installing Gazebo 11 from source

sudo apt-get remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*'
sudo apt-get remove '.*sdformat.*' '.*ignition-.*'

sudo apt-get install libbullet-dev -y

sudo apt-get update -y

sudo apt-get install pkg-config -y

rm -rf /tmp/ign-cmake

git clone https://github.com/ignitionrobotics/ign-cmake /tmp/ign-cmake
cd /tmp/ign-cmake
git checkout ign-cmake2
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install

rm -rf /tmp/ign-math

git clone https://github.com/ignitionrobotics/ign-math /tmp/ign-math
cd /tmp/ign-math

git checkout ign-math6
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install

sudo apt-get install -y libfreeimage-dev \
         libtinyxml2-dev \
         uuid-dev \
         libgts-dev \
         libavdevice-dev \
         libavformat-dev \
         libavcodec-dev \
         libswscale-dev \
         libavutil-dev \
         libprotoc-dev \
             libprotobuf-dev

rm -rf /tmp/ign-common

git clone https://github.com/ignitionrobotics/ign-common /tmp/ign-common
cd /tmp/ign-common

git checkout ign-common3
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install

sudo apt-get install -y libboost-system-dev \
                     libtinyxml-dev \
                     libxml2-utils \
                     ruby-dev \
                     ruby

rm -rf /tmp/sdformat

git clone https://github.com/osrf/sdformat /tmp/sdformat
cd /tmp/sdformat/

git checkout sdf9
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install

sudo apt-get install -y libprotoc-dev \
                     libprotobuf-dev \
                     protobuf-compiler

rm -rf /tmp/ign-msgs

git clone https://github.com/ignitionrobotics/ign-msgs /tmp/ign-msgs
cd /tmp/ign-msgs
git checkout ign-msgs5
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install

sudo apt-get install -y libzip-dev \
         libjsoncpp-dev \
         libcurl4-openssl-dev \
         libyaml-dev

rm -rf /tmp/ign-fuel-tools

git clone https://github.com/ignitionrobotics/ign-fuel-tools /tmp/ign-fuel-tools
cd /tmp/ign-fuel-tools
git checkout ign-fuel-tools4
mkdir build
cd build

cmake ../
make -j$(nproc)
sudo make install

sudo apt-get remove libignition-transport3-dev

sudo apt-get install -y mercurial cmake pkg-config python ruby-ronn libprotoc-dev libprotobuf-dev protobuf-compiler uuid-dev libzmq3-dev libignition-msgs-dev

git clone https://github.com/ignitionrobotics/ign-transport.git /tmp/ign-transport
cd /tmp/ign-transport
git checkout ign-transport8

mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install

rm -rf /tmp/gazebo

git clone https://github.com/osrf/gazebo /tmp/gazebo
cd /tmp/gazebo

sudo apt-get install ruby-ronn -y # For Gazebo Man Pages

mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install

echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf
sudo ldconfig

# echo -e "\n${GREEN}Installing Gazebo v10.${NC}\n" & sleep 1

# sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' & sleep 1

# sudo wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# sudo apt -q update -y

# sudo apt-get install gazebo10 -y
# sudo apt-get install libgazebo10-dev -y

cd $GITHUB_DIRECTORY

sudo apt -q update -y

# Install casADi for use in Python and C++

echo -e "\n\n${GREEN}Installing casADi framework for use in C++ and Python now...${NC}\n\n" & sleep 2

sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends -y

sudo apt-get install swig ipython3 python3-dev python3-numpy python3-scipy python3-matplotlib --install-recommends -y

sudo apt-get install spyder -y

sudo apt -q update -y

sudo apt-get install coinor-libipopt-dev -y

sudo apt -q update -y

rm -rf ./casadi/

git clone https://github.com/casadi/casadi.git -b master casadi
sudo chmod -R ugo+rw ${GITHUB_DIRECTORY}/casadi/
cd casadi
mkdir build && cd build

sudo chmod -R ugo+rw ${GITHUB_DIRECTORY}/casadi/

echo -e "\n\n#This was added by the setup.sh script of the biped_controller project \n#make the casadi compiler find IPOPT.\n" >> ${HOME_DIR}/.bashrc
echo -e "export PKG_CONFIG_PATH=/usr/lib/pkg_config/" >> ${HOME_DIR}/.bashrc & sleep 1

eval "$(cat ${HOME_DIR}/.bashrc | tail -n +10)" # https://askubuntu.com/questions/64387/cannot-successfully-source-bashrc-from-a-shell-script

echo -e "\n${GREEN}PKG_CONFIG_PATH in bash script: $PKG_CONFIG_PATH\n${NC}"

sh -c ". ${HOME_DIR}/.bashrc && export PKG_CONFIG_PATH=/usr/lib/pkg_config/ && echo '\nPKG_CONFIG_PATH in sh command: $PKG_CONFIG_PATH\n' && sudo cmake -DWITH_PYTHON3=ON -DWITH_IPOPT=ON -DWITH_THREAD=ON .."

#echo -e "\n${RED}Please make sure there is no error message about IPOPT not being found.${NC}\n" & sleep 4
sudo chmod -R ugo+rw ${GITHUB_DIRECTORY}/casadi/

make -j$(nproc)
sudo make install
sudo chmod -R ugo+rw ${GITHUB_DIRECTORY}/casadi/

echo -e "\nInstallation finished, running unit tests in Python.\n" 

cd .. # Go back to the main casadi source directory
cd test/python
python3 alltests.py

# Install ZCM and ZMQ

cd $GITHUB_DIRECTORY

echo -e "\n\n${GREEN}Trying to install ZCM and ZMQ. They are currently not needed, so an error does not mean the controller will not be functional.${NC}\n\n" & sleep 5

sudo apt-get install libzmq3-dev -y

rm -rf ./zcm/

git clone https://github.com/ZeroCM/zcm.git
sudo chmod -R ugo+rw ${GITHUB_DIRECTORY}/zcm/
cd zcm

echo -e "\n${GREEN}Running ZCM dependency script now...${NC}\n" & sleep 1

echo -e "\n${GREEN}Configuring, building and installing ZCM now...${NC}\n" & sleep 1

sudo ./waf configure

./scripts/install-deps.sh

./waf configure --use-all

sudo chmod -R ugo+rw ${GITHUB_DIRECTORY}/zcm/

echo -e "\n\n#This was added by the setup.sh script of the biped_controller project \n#to successfully install ZCM.\n" >> ${HOME_DIR}/.bashrc
echo -e "export PATH=${PATH}:${GITHUB_DIRECTORY}/zcm/deps/julia/bin\n" >> ${HOME_DIR}/.bashrc

eval "$(cat ${HOME_DIR}/.bashrc | tail -n +10)" # https://askubuntu.com/questions/64387/cannot-successfully-source-bashrc-from-a-shell-script

./waf build
sudo ./waf install

sudo chmod -R ugo+rw ${GITHUB_DIRECTORY}/zcm/

#Update LD_LIBRARY_PATH in order for gazebo to find the shared object

echo -e "\n\n#This was added by the} setup.sh script of the biped_controller project \n#to make Gazebo find the Shared Object file of the controller plugin.\n" >> ${HOME_DIR}/.bashrc
echo -e "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${HOME_DIR}/.gazebo/models/simplified_biped/control_plugin/build" >> ${HOME_DIR}/.bashrc

eval "$(cat ${HOME_DIR}/.bashrc | tail -n +10)" # https://askubuntu.com/questions/64387/cannot-successfully-source-bashrc-from-a-shell-script

cd $GITHUB_DIRECTORY

rm -rf ./jupyter_notebooks/

git clone https://github.com/LouKordos/jupyter_notebooks.git
sudo chmod -R ugo+rw ${GITHUB_DIRECTORY}/jupyter_notebooks/

cd ./jupyter_notebooks

jupyter trust *.ipynb # Sign and trust all notebooks

cd ..

mkdir ${HOME_DIR}/.gazebo/ # Only gets created after first run of gazebo, so we do it manually
sudo chmod -R ugo+rw ${HOME_DIR}/.gazebo/

mkdir ${HOME_DIR}/.gazebo/models/ && cd ${HOME_DIR}/.gazebo/models/
sudo chmod -R ugo+rw ${HOME_DIR}/.gazebo/models/

rm -rf ./simplified_biped/

git clone https://github.com/LouKordos/simplified_biped.git
sudo chmod -R ugo+rw ${HOME_DIR}/.gazebo/models/simplified_biped/

echo -e "\n${GREEN}Building Gazebo control plugin for Biped...${NC}\n" & sleep 1

cd ${HOME_DIR}/.gazebo/models/simplified_biped/control_plugin/

sudo rm -rf build

mkdir build && cd build
sudo chmod -R ugo+rw ${HOME_DIR}/.gazebo/models/simplified_biped/
cmake ..
sudo chmod -R ugo+rw ${HOME_DIR}/.gazebo/models/simplified_biped/
make -j$(nproc)
sudo chmod -R ugo+rw ${HOME_DIR}/.gazebo/models/simplified_biped/

echo -e "\n#This alias will allow easier Simulation startup." >> ${HOME_DIR}/.bashrc
echo -e "\nalias start_biped_simulation=\"mkdir ${HOME_DIR}/.gazebo/models/simplified_biped/control_plugin/build/ ; cd ${HOME_DIR}/.gazebo/models/simplified_biped/control_plugin/build/ && make -j$(nproc) && gazebo --verbose ../../simplified_biped.world\"" >> ${HOME_DIR}/.bashrc

eval "$(cat ${HOME_DIR}/.bashrc | tail -n +10)" # https://askubuntu.com/questions/64387/cannot-successfully-source-bashrc-from-a-shell-script

echo -e "\n${GREEN}Building Gazebo main walking controller for Biped...${NC}\n" & sleep 1

cd $WORKSPACE_DIRECTORY

#cp -R eigen3/Eigen /usr/include/ # copy to include directory to be sure (is needed for internal includes like "#include <Eigen/Core>")
#cp -R Eigen_unsupported /usr/include/
export PREV_DIR = $(pwd)
git clone https://gitlab.com/libeigen/eigen.git /tmp/eigen3 # Install Eigen3
cd /tmp/eigen3/
mkdir build && cd build
sudo make install
cd PREV_DIR

sudo chmod -R ugo+rw $WORKSPACE_DIRECTORY
sudo rm -rf build
mkdir build && cd build
sudo chmod -R ugo+rw $WORKSPACE_DIRECTORY
cmake ..
sudo chmod -R ugo+rw $WORKSPACE_DIRECTORY
make -j$(nproc)
sudo chmod -R ugo+rw $WORKSPACE_DIRECTORY

echo -e "\n#This alias will allow easier startup of the Biped walking controller." >> ${HOME_DIR}/.bashrc
echo -e "\nalias run_walking_controller=\"mkdir ${WORKSPACE_DIRECTORY}/build/ ; cd ${WORKSPACE_DIRECTORY}/build/ && make -j$(nproc) && ./controller\"" >> ${HOME_DIR}/.bashrc

echo -e "\n#This alias will (hopefully) allow updating all Biped repos automatically." >> ${HOME_DIR}/.bashrc
echo -e "\nalias update_biped_repos=\"cd ${WORKSPACE_DIRECTORY} && git pull ; cd ${GITHUB_DIRECTORY}/jupyter_notebooks/ && git pull ; cd ${HOME_DIR}/.gazebo/models/simplified_biped/ && git pull\"" >> ${HOME_DIR}/.bashrc

eval "$(cat ${HOME_DIR}/.bashrc | tail -n +10)" # https://askubuntu.com/questions/64387/cannot-successfully-source-bashrc-from-a-shell-script

ELAPSED_TIME=$(($SECONDS - $START_TIME))

echo -e "Setup done! It took $ELAPSED_TIME seconds in total."

echo -e "\n${RED}Again, in case you missed it:\nAll github repositories were cloned into ${GITHUB_DIRECTORY}${NC}\n"

echo -e "${GREEN}To run the simulation, open a terminal and run \"start_biped_simulation\".\nTo run the seperate walking controller code, open another terminal and run \"run_biped_controller\".${NC}"

echo -e "Note: You might have to restart this terminal to source .bashrc because the shell runs in its own instance."