FROM osrf/ros:noetic-desktop-focal

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Install ROS packages
RUN apt-get update && apt-get install --yes --no-install-recommends \
        # Gazebo
        ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --yes --no-install-recommends \
        git \
    && rm -rf /var/lib/apt/lists/*

# Install pyenv
RUN git clone https://github.com/pyenv/pyenv.git ~/.pyenv && \
    # Compile a dynamic Bash extension to speed up pyenv
    cd ~/.pyenv && src/configure && make -C src && \
    # Install Python build dependencies
    apt-get update && apt-get install --yes --no-install-recommends \
        make build-essential wget curl llvm xz-utils tk-dev libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev libncursesw5-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev \
    && rm -rf /var/lib/apt/lists/*
# Set up shell environment for pyenv
ENV PYENV_ROOT /root/.pyenv
ENV PATH $PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH
RUN echo 'eval "$(pyenv init -)"' >> ~/.bashrc && \
    echo 'eval "$(pyenv init -)"' >> ~/.profile

# Install Python environment
RUN pyenv install 3.7.13 && \
    pyenv global 3.7.13 && \
    pip install --upgrade pip && \
    # ddpg_main_1804.py
    pip install pyyaml rospkg matplotlib scipy numpy 'tensorflow>=1.15,<2.0' keras==2.3.1 && \
    pip install protobuf==3.20.* && \
    pip install defusedxml && \
    # catkin_make depends on EmPy
    pip install empy

ENV TURTLEBOT3_MODEL waffle_pi
