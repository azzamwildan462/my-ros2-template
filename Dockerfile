FROM ros:humble-ros-base

RUN apt update \
    && apt install -y \
    nano \
    python3-pip \
    iputils-ping \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    numpy \
    pandas \
    loguru \
    flask \
    flask_jwt_extended \ 
    flask_cors 

RUN mkdir -p /root/server
COPY ./src /root/server/src
COPY ./make.sh /root/server
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /root/server && bash make.sh"

# LAZY untuk SSH, instead of using -it
RUN echo 'root:1' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config \
    && sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# LAZY ENTRYPOINT untuk debug
COPY ./my_entrypoint.sh /

# LAZY untuk SSH
RUN mkdir -p /run/sshd && ssh-keygen -A
EXPOSE 22

CMD ["/usr/sbin/sshd", "-D"]
# CMD ["/usr/bin/bash", "my_entrypoint.sh"]