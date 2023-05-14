#!/bin/bash

# execute:
# sudo -i
# bash ./setup.sh

# install basics
apt-get update
apt-get upgrade -y

# install commons
apt install -y curl screen minicom

# setup locale and wifi region
locale-gen de_DE.UTF-8
update-locale LANG=de_DE.UTF-8
sh -c 'printf "ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev\nupdate_config=1\ncountry=DE\n" >> /etc/wpa_supplicant/wpa_supplicant.conf'
cp /etc/wpa_supplicant/wpa_supplicant.conf /boot

# setup access point
apt install -y hostapd dnsmasq
systemctl unmask hostapd

sh -c 'printf "interface wlan0\nstatic ip_address=192.168.4.1/24\nnohook wpa_supplicant\n" >> /etc/dhcpcd.conf'
sh -c 'printf "country_code=DE\ninterface=wlan0\nssid=drohne\nhw_mode=g\nchannel=7\nmacaddr_acl=0\nauth_algs=1\nignore_broadcast_ssid=0\nwpa=2\nwpa_passphrase=dhbw1234\nwpa_key_mgmt=WPA-PSK\nwpa_pairwise=TKIP\nrsn_pairwise=CCMP\n" >> /etc/hostapd/hostapd.conf'

mv /etc/dnsmasq.conf /etc/dnsmasq.conf_alt
sh -c 'printf "# DHCP-Server aktiv für WLAN-Interface\ninterface=wlan0\n\n# DHCP-Server nicht aktiv für bestehendes Netzwerk\nno-dhcp-interface=eth0\n# IPv4-Adressbereich und Lease-Time\ndhcp-range=192.168.4.100,192.168.4.200,255.255.255.0,24h\n# DNS\ndhcp-option=option:dns-server,8.8.8.8\n" > /etc/dnsmasq.conf'

# install docker
curl -fsSL https://get.docker.com -o get-docker.sh
sh get-docker.sh
sudo usermod -aG docker pi

# increase swap
#TODO increase max swap, but I forgot the syntax
dphys-swapfile swapoff
sh -c 'printf "CONF_SWAPSIZE=4096\n" > /etc/dphys-swapfile'
dphys-swapfile setup
dphys-swapfile swapon

# reboot
init 6
