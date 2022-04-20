# loadcell_process
process force information from load cell


## equipment
- [Forsentek Load Cell MAC-200 and LC3A](http://www.forsentek.com/down/multi_axis_load_cell_MAC.pdf)

## software
- ubuntu mate for raspberry pi 4
- ROS Noetic

### install
1. Install Ubuntu 20.04
2. Install ROS Noetic


### config SPI interface
```
sudo raspi-config
```
Select Interface OPtions -> SPI -> Yes to open the SPI interface, then reboot.

If raspi-config is not install, follow this [link](https://ubuntu-mate.community/t/install-raspi-config-on-ubuntu-mate-20-10-and-higher/23974) to install raspi-config on Ubuntu Mate, (open Teminal, type ```sudo su``` to run as root)
```
echo "deb http://archive.raspberrypi.org/debian/ buster main" >> /etc/apt/sources.list
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 7FA3303E
apt-get update && apt-get install raspi-config
```

### install [BCM2835](http://www.airspayce.com/mikem/bcm2835/)
```
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.68.tar.gz
tar zxvf bcm2835-1.68.tar.gz
cd bcm2835-1.68
sudo ./configure && sudo make && sudo make check && sudo make install
```

### install wiringPi
```
sudo apt-get install wiringpi
```
Run ```gpio -V``` to check the version of wiringpi

### install GPIO
```
sudo apt-get update
sudo apt-get install ttf-wqy-zenhei
sudo apt-get install python3-pip
sudo pip install RPi-GPIO
sudo pip install spidev
```

### setup service
1. create a service /etc/systemd/system/loadcell-service.service
```
[Unit]
Description="Load Cell ROS Start"
After=multi-user.target

[Service]
Type=simple
User=ubuntu
Group=ubuntu
ExecStart=/home/ubuntu/catkin_ws/src/loadcell_process/auto_start.sh

[Install]
WantedBy=multi-user.target

```

2. make scripts executable
```
sudo chmod +x auto_start.sh
sudo chmod +x scripts/load_cell_interface.py
```

3. enable sudo no password ```sudo visudo``` and add line after group sudo
```
${username} ALL=(ALL:ALL) NOPASSWD:ALL
```

4. enable service
```
sudo systemctl enable loadcell-service.service
```
