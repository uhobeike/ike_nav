FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

ARG USERNAME="root"
ENV USER=$USERNAME
ENV USERNAME=$USERNAME
ENV GIT_PS1="${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w$(__git_ps1)\[\033[00m\](\t)\$ "
ENV NO_GIT_PS1="${debian_chroot:+($debian_chroot)}\u@\h:\w \$ "

RUN groupadd -g 1000 $USERNAME && \
    useradd -m -s /bin/bash -u 1000 -g 1000 -d /home/$USERNAME $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME

RUN apt update && apt upgrade -y && \
    apt install -y \
    bash-completion \
    git \
    terminator \
    vim \
    wget \
    ros-$ROS_DISTRO-turtlebot3-gazebo

USER $USERNAME
RUN mkdir -p /home/$USERNAME/ike_nav_ws/src && \
    cd /home/$USERNAME/ike_nav_ws/src && \
    git clone https://github.com/uhobeike/ike_nav.git && \
    rosdep update && \
    rosdep install -r -y --from-paths --ignore-src ./

RUN cd /home/$USERNAME/ike_nav_ws/ && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    : "remove cache" && \
    sudo apt-get autoremove -y -qq && \
    sudo rm -rf /var/lib/apt/lists/*

RUN mkdir -p $HOME/.gazebo/models && \
    wget -O /tmp/sun http://models.gazebosim.org/sun/model.tar.gz && \
    wget -O /tmp/ground_plane http://models.gazebosim.org/ground_plane/model.tar.gz && \
    tar xvzf /tmp/sun -C $HOME/.gazebo/models && \
    tar xvzf /tmp/ground_plane -C $HOME/.gazebo/models

RUN echo "export TURTLEBOT3_MODEL=burger" >> $HOME/.bashrc && \
    echo "source /home/$USERNAME/ike_nav_ws/install/setup.bash" >> $HOME/.bashrc && \
    echo "source /etc/bash_completion" >> $HOME/.bashrc && \
    echo "if [ -f /etc/bash_completion.d/git-prompt ]; then" >> $HOME/.bashrc && \
    echo "    export PS1='${GIT_PS1}'" >> $HOME/.bashrc && \
    echo "else" >> $HOME/.bashrc && \
    echo "    export PS1='${NO_GIT_PS1}'" >> $HOME/.bashrc && \
    echo "fi" >> $HOME/.bashrc && \
    bash <(curl -s https://raw.githubusercontent.com/uhobeike/ros2_humble_install_script/main/ros2_setting.sh)

WORKDIR /home/$USERNAME/ike_nav_ws/src/ike_nav
CMD ["/bin/bash"]