sudo 	echo "deb http://archive.raspberrypi.org/debian/ buster main" >> /etc/apt/sources.list
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 7FA3303E
sudo apt update 
sudo apt upgrade 
sudo apt-get install raspi-config
sudo apt-get install rpi.gpio
sudo raspi-config
