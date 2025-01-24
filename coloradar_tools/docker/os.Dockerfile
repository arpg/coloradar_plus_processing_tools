ARG BASE_IMAGE
FROM ${BASE_IMAGE}


RUN apt update && apt upgrade -y
RUN apt install -y gnupg software-properties-common build-essential
RUN apt install -y python3.8-venv python3.8-dev python3-pip
RUN pip install -U pip
RUN python3 --version && pip3 --version


CMD ["bash"]