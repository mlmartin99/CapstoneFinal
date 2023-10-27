from ros:humble

RUN apt update && \
    apt install -y \
    ros-humble-rviz2

RUN apt update && \
    apt install -y \
    python3-pip \
    wget \
    vim \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-teleop-twist-keyboard \
    xclip \
    less \
    net-tools \
    gdb     \ 
    git


RUN pip3 install Jetson.GPIO
RUN pip3 install opencv-python depthai 
RUN pip3 install --upgrade setuptools==58.2.0

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source install/setup.bash" >> ~/.bashrc

RUN apt install -y \
    python3-colcon-common-extensions \
    ros-humble-rosbridge-server -y \
    ros-humble-async-web-server-cpp -y 

# Install Node.js
RUN curl --silent --location https://deb.nodesource.com/setup_19.x | sudo -E bash -
RUN apt-get install --yes nodejs
RUN apt-get install --yes build-essential

# Binds to port 3000
EXPOSE  3000