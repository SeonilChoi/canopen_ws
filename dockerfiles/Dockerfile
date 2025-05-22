FROM ros:foxy

RUN apt-get update && \
    apt-get install -y \ 
	vim \
	python3-pip \
        can-utils \
        iproute2 && \
    pip3 install canopen && \
    echo "set ts=4" > ~/.vimrc && \
    rm -rf /var/lib/apt/lists/*

ARG USERNAME=shs
ARG USER_UID=1000
ARG USER_GID=1000

# sudo 포함 및 사용자 추가
RUN apt update && apt install -y sudo && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME
