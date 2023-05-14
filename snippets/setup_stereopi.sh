#!/bin/bash

# execute:
# sudo bash ./setup.sh

# add radxa repo
export DISTRO=focal-stable
echo "deb http://apt.radxa.com/$DISTRO/ ${DISTRO%-*} main" | sudo tee -a /etc/apt/sources.list.d/apt-radxa-com.list
wget -O - apt.radxa.com/$DISTRO/public.key | sudo apt-key add -
# make sure we use radxa packages over Ubuntu packages
echo "Package: *
Pin: release n=repo
Pin-Priority: 50" | sudo tee /etc/apt/preferences.d/radxa-conf
# install basics
sudo apt-get update
sudo apt-get install -y linux-4.19-rock-3-latest
sudo apt-get upgrade
sudo apt-get install rockchip-overlay

# install commons
sudo apt install curl screen

# install docker
curl -fsSL https://get.docker.com -o get-docker.sh
sh get-docker.sh
sudo usermod -aG docker $USER
