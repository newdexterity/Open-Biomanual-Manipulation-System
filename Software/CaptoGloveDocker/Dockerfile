FROM ubuntu:xenial 

LABEL maintainer padula

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV QT_MAJOR_VERSION 5.12
ENV QT_MINOR_VERSION 5.12.10

RUN apt-get update && apt-get install -q -y \
    apt-utils \
    build-essential \
    bc \
    cmake \
    curl \
    git \
    imagemagick \
    lsb-release \
    libboost-dev \
    libqt5serialport5-dev \
    sudo \
    nano \
    net-tools \
    tmux \
    wget 

# Add sources for ROS installation
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS
ENV ROS_DISTRO kinetic
RUN apt-get update
RUN apt-get install -q -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-core \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-qt-build \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-teleop-twist-joy

# Install ROS python packages
RUN apt-get install -y \
    python-rosdep \
    python-rosinstall \
    python-vcstool \
    python-rosinstall-generator

# Install catkin tools
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y --no-install-recommends python-catkin-tools

RUN useradd -ms /bin/bash -G sudo developer && echo "developer:developer" | chpasswd

WORKDIR /home/developer
RUN mkdir -p catkin_ws/src \
    && bash -c ". /opt/ros/kinetic/setup.bash; catkin_init_workspace /home/developer/catkin_ws/src" \
    && bash -c ". /opt/ros/kinetic/setup.bash; cd catkin_ws; catkin_make" \
    && echo '. /home/developer/catkin_ws/devel/setup.bash' >> /root/.bashrc \
    && echo 'export ROS_MASTER_URI=http://$(hostname --ip-address):11311' >> /root/.bashrc \
    && echo 'export ROS_HOSTNAME=$(hostname --ip-address)' >> /root/.bashrc

# Install QT
# Download QT and install from sources to have good verson 
RUN wget download.qt.io/official_releases/qt/5.15/5.15.2/single/qt-everywhere-src-5.15.2.tar.xz
RUN tar -xvf qt-everywhere-src-5.15.2.tar.xz

# Install BlueZ and rest of the packages neccessary for qtconnectivity to work (first 2 are used for protobuf)
RUN apt-get install -y \
    autoconf \
    libtool \ 
    libbluetooth-dev \
    bluetooth \
    blueman \
    bluez \
    libusb-dev \
    bluez-hcidump \
    bluez-tools 

COPY configuration.sh /home/developer/qt-everywhere-src-5.15.2 

WORKDIR /home/developer/qt-everywhere-src-5.15.2
RUN ./configuration.sh \
    && make -j6 \
    && make install -j6

RUN echo 'export PATH=/home/Qt/bin:$PATH' >> /root/.bashrc

RUN rm -rf /home/developer/qt-everywhere-src-5.15.2.tar.xz
RUN rm -rf /home/developer/qt-everywhere-src-5.15.2

RUN mkdir code
WORKDIR /home/developer

# Build protobuf
RUN git clone https://github.com/protocolbuffers/protobuf.git \
    && cd protobuf \
    && git checkout 3.11.x \ 
    && mkdir build \
    && cd build \
    && bash -c "cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF ../cmake " \
    && bash -c "make -j7; make install"

ENV PROTOBUF_LIB_PATH /home/developer/protobuf
ENV BUILD_DIR_SUFFIX buildGCC_x64

# Build CaptoGloveAPI
RUN git clone https://github.com/larics/CaptoGloveAPI.git \
    && cd CaptoGloveAPI \
    && git checkout devel \
    && git submodule init \
    && git submodule update 
    
WORKDIR /home/developer
RUN export PATH=/home/Qt/bin:$PATH \
    && mkdir CaptoGloveAPI-${BUILD_DIR_SUFFIX} \
    && cd CaptoGloveAPI-${BUILD_DIR_SUFFIX} \
    && qmake ../CaptoGloveAPI/CaptoGloveAPI.pro \ 
    && make -j6

ENV CAPTOGLOVEAPI_SRC_PATH /home/developer/CaptoGloveAPI
ENV CAPTOGLOVEAPI_LIBRARY_PATH /home/developer/CaptoGloveAPI-buildGCC_x64

# Cloning CaptoGlove related stuff for ROS integration 
WORKDIR /home/developer/catkin_ws/src
RUN git clone https://github.com/fzoric8/captoglove_ros_wrapper.git
RUN git clone https://github.com/fzoric8/captoglove_ros_msgs.git 
WORKDIR /home/developer/catkin_ws

# For some reason, catkin_make fails because it's not able to find Qt5, but after running docker it is TODO:Check
#RUN bash -c "source /opt/ros/kinetic/setup.bash; source /root/.bashrc; catkin_make" 

# Install Clion for debugging    
#WORKDIR /home/developer
#RUN wget "https://download-cf.jetbrains.com/cpp/CLion-2018.3.4.tar.gz"
#RUN tar -xzf CLion-*.tar.gz
#RUN apt-get install -y openjdk-8-jre

WORKDIR /root
RUN apt-get install -y rfkill
COPY docker_entrypoint.sh ./

ENTRYPOINT sh docker_entrypoint.sh
