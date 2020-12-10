FROM ubuntu:bionic
COPY . /app
RUN  apt-get update -y
RUN  /app/setup/ubuntu/install_prereqs.sh

