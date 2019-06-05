#!/usr/bin/env bash

# Install Pangolin
ORB_SLAM2_PATH=$(pwd)/../thirdparty/ORB_SLAM2
cd ~
if ! [[ $(ldconfig -p | grep libpangolin) ]]; then
    PANGOLIN_FOLDER=~/Pangolin
    if [ ! -d "$PANGOLIN_FOLDER" ]; then
		git clone https://github.com/stevenlovegrove/Pangolin.git
	fi
	if [ ! -d "$PANGOLIN_FOLDER/build" ]; then
		cd $PANGOLIN_FOLDER
	    mkdir build
	    cd build
	    cmake ..
	    cmake --build .
	    sudo make install
	fi
    
else
    echo "Pangolin already exists"
fi

# # Install OpenCV
# # This step is skipped because ros already has OpenCV 3
# cd ~
# if ! [[ $(ldconfig -p | grep libopencv | grep 3.4) ]]; then
#     sudo apt-get -y install build-essential -y
#     sudo apt-get -y install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev  -y
#     sudo apt-get -y install libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libtiff5-dev libjasper-dev libdc1394-22-dev libeigen3-dev -y
#     sudo apt-get -y install unzip -y
#     if [ ! -d opencv-3.4.3 ]; then
#         wget https://github.com/opencv/opencv/archive/3.4.3.zip -O opencv-3.4.3.zip
#         unzip opencv-3.4.3.zip
#     fi
#     cd opencv-3.4.3
#     mkdir build
#     cd build
#     cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local  -DINSTALL_C_EXAMPLES=ON -DINSTALL_PYTHON_EXAMPLES=ON -DWITH_IPP=ON -DBUILD_NEW_PYTHON_SUPPORT=ON -DWITH_TBB=ON -DWITH_V4L=ON -DWITH_QT=ON -DWITH_OPENGL=ON -DWITH_OPENCL=ON -DWITH_VTK=ON -DWITH_TIFF=ON -DBUILD_TIFF=ON -DWITH_EIGEN=ON  -DPYTHON_EXECUTABLE=$(which python) -DPYTHON_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") -DPYTHON_PACKAGES_PATH=$(python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())")  ..
#     make 
#     sudo make install
# else
#     echo "Opencv already exists"
# fi


# Compile ORB_SLAM2
cd $ORB_SLAM2_PATH
echo "Go to ORB_SLAM2 directory: $(pwd)"
chmod +x build.sh
./build.sh
echo "export ORBSLAM2_LIBRARY_PATH=${ORB_SLAM2_PATH}" >> ~/.bashrc
export ORBSLAM2_LIBRARY_PATH=${ORB_SLAM2_PATH}
