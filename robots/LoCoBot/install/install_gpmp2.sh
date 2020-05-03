#!/usr/bin/env bash

# Install prerequisites
sudo apt-get -y install python-numpy \
						python-scipy \
						python-matplotlib \
						python-pandas \
						python-sympy \
						python-nose \
						cmake \
						libboost-all-dev
pip install cython

GPMP_FOLDER=~/gpmp_ws
if [ ! -d "$GPMP_FOLDER" ]; then
	mkdir -p $GPMP_FOLDER
fi

# Install GTSAM
if [ ! -d "$GPMP_FOLDER/gtsam" ]; then
	cd $GPMP_FOLDER
	git clone https://github.com/borglab/gtsam.git
	cd gtsam
	git checkout wrap-export
	mkdir build
	cd build
	cmake -DGTSAM_INSTALL_CYTHON_TOOLBOX:=ON ..
	sudo make install -j8
	# Update pythonpath in bashrc to recognize GPMP2
	echo "export PYTHONPATH=$PYTHONPATH:/usr/local/cython" >> ~/.bashrc
	source ~/.bashrc
fi

# Install GPMP2

if [ ! -d "$GPMP_FOLDER/gpmp2" ]; then
	cd $GPMP_FOLDER
	git clone https://github.com/kalyanvasudev/gpmp2.git
	export PYTHONPATH=$PYTHONPATH:/usr/local/cython
	cd gpmp2
	mkdir build
	cd build
	cmake -DGPMP2_BUILD_PYTHON_TOOLBOX:=ON ..
	make
	sudo make install
fi
