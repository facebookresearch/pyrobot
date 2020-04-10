#!/usr/bin/env bash

# Install prerequisites
sudo apt-get -y install python-numpy \
						python-scipy \
						python-matplotlib \
						ipython \
						ipython-notebook \
						python-pandas \
						python-sympy \
						python-nose \
						cmake \
						libboost-all-dev
pip install cython

GPMP_FOLDER=~/gpmp_ws

# Clone GTSAM and GPMP2
mkdir -p $GPMP_FOLDER
cd $GPMP_FOLDER
git clone https://github.com/kalyanvasudev/gpmp2.git
git clone https://github.com/borglab/gtsam.git

# Install GTSAM
cd $GPMP_FOLDER/gtsam
git checkout wrap-export
mkdir build
cd build
cmake -DGPMP2_BUILD_PYTHON_TOOLBOX:=ON ..
sudo make install -j8

# Install GPMP2
cd $GPMP_FOLDER/gpmp2
mkdir build
cd build
cmake -DGPMP2_BUILD_PYTHON_TOOLBOX:=ON ..
sudo make install -j8


# Update pythonpath in bashrc to recognize GPMP2
echo "export PYTHONPATH=$PYTHONPATH:/usr/local/cython" >> ~/.bashrc
source ~/.bashrc