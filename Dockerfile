# From https://hub.docker.com/_/ros
# All subsequent files are generated from this file.
##############################################################

# This is an auto generated Dockerfile for ros:perception
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-base-focal

# install ros packages for (perception), (mavros and geographiclib), (rqt graph and topic), (catkin and catkin tools)
# RUN /bin/bash -c 'echo "deb http://security.ubuntu.com/ubuntu xenial-security universe" | sudo tee -a /etc/apt/sources.list'
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-perception=1.5.0-1* \
    ros-noetic-mavros ros-noetic-mavros-extras python3-geographiclib \
    ros-noetic-rqt-graph ros-noetic-rqt-topic \
    ros-noetic-catkin python3-catkin-tools \
    build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    unzip v4l-utils \  
    gcc-arm* protobuf-compiler \
    python-dev python-numpy \
    libgtk2.0-dev libcanberra-gtk* libopenblas-dev \
    libblas-dev liblapack-dev \
    libv4l-0 \
    wget git unzip screen nano \
    && rm -rf /var/lib/apt/lists/*
#    gcc-10* g++-10*
# RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100
# RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100
# adjust global settings
# see https://discuss.ardupilot.org/t/rtt-too-high-for-timesync-with-sitl-mavros/38224/8
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh
RUN /bin/bash -c "sed -i '12s/10.0/0.0/g' /opt/ros/noetic/share/mavros/launch/px4_config.yaml"

# get the sources for my custom image pipeline
RUN wget http://download.ros.org/data/stereo_image_proc/rotating_detergent_1_6.bag
RUN /bin/bash -c "git clone https://github.com/ros-perception/image_pipeline.git && \
    cd image_pipeline && \
    git checkout noetic"
RUN /bin/bash -c "git clone https://github.com/ros-perception/vision_opencv.git && \
    cd vision_opencv && \
    git checkout noetic"

# install opencv
RUN /bin/bash -c "mkdir ~/opencv_build && cd ~/opencv_build && \
    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.2.0.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.2.0.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip && \
    mv opencv-4.2.0 opencv && \
    mv opencv_contrib-4.2.0 opencv_contrib"

RUN echo "11" >> dummy.txt

RUN /bin/bash -c 'cd ~/opencv_build/opencv && mkdir build && cd build && \
    export CXXFLAGS="-march=native -march=native -O3" && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CPU_BASELINE=NEON \
        -D CMAKE_INSTALL_PREFIX=/usr \
        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
        -D ENABLE_NEON=ON \
        -D WITH_OPENMP=ON \
        -D BUILD_TIFF=ON \
        -D WITH_FFMPEG=ON \
        -D WITH_TBB=ON \
        -D BUILD_TBB=ON \
        -D BUILD_TESTS=OFF \
        -D WITH_EIGEN=ON \
        -D WITH_GSTREAMER=OFF \
        -D WITH_V4L=ON \
        -D WITH_LIBV4L=ON \
        -D WITH_GTK=OFF \
        -D WITH_QT=OFF \
        -D WITH_VTK=OFF \
        -D OPENCV_EXTRA_EXE_LINKER_FLAGS=-latomic \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D INSTALL_C_EXAMPLES=OFF \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D BUILD_NEW_PYTHON_SUPPORT=ON \
        -D BUILD_opencv_python3=TRUE \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D ENABLE_LTO=ON \
        -D BUILD_EXAMPLES=OFF ..'

# install my image resizer
RUN /bin/bash -c "source ros_entrypoint.sh && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    catkin create pkg dummy_image_resize && \
    mkdir -p ~/catkin_ws/src/dummy_image_resize/launch && \
    cd ~/catkin_ws && catkin build dummy_image_resize"

COPY ./image_resize.launch /root/catkin_ws/src/dummy_image_resize/launch/image_resize.launch

# install opencv
RUN /bin/bash -c "cd ~/opencv_build/opencv/build && make -j4 && make install && ldconfig"
# install modules
RUN /bin/bash -c 'source ros_entrypoint.sh && \
    mv /image_pipeline ~/catkin_ws/src/image_pipeline && \
    mv /vision_opencv ~/catkin_ws/src/vision_opencv && \
    cd ~/catkin_ws && \
    export CXXFLAGS="-march=native -O3" && \
    catkin build -DCMAKE_BUILD_TYPE=Release -DCXXFLAGS="-march=native -O3"'
