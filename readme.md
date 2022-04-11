# Installation & Setup

## Main pkg battle_turtle

**1.** Install the package from ths repo:

```bash
cd ~/catkin_ws/src
git clone https://github.com/mzaera/battle_turtle.git
cd ~/catkin_ws
catkin_make
```
If you can not use the package, try to source the catkin_ws with the following command inside the /catkin_ws folder:
 
```bash
source devel/setup.bash
```
If it still not working try to fnde the pkg by:
```bash
rospack find battle_turtle
```
## ArduinoMega connection

**1.** Installing rosserial packages on the ROS workstation
(PC):

```bash
sudo apt install ros-melodic-rosserial-arduino
sudo apt install ros-melodic-rosserial
```

**2.** You will need to install the [Arduino IDE](https://www.arduino.cc/en/software).

After uncompressing the dowloaded file inside the downloaded folder, make sure the installation script has execution permissions:

```bash
chmod +x install.sh
./install.sh
```
**3.** Next, we will need to generate rosserial Arduino
libraries, to be possible include it in in your arduino code:

```bash
cd ~/Arduino/libraries/
rosrun rosserial_arduino make_libraries.py .
```

**4.** Send the robot_firmware code from the Arduino IDE.

Possile ERROR:

```bash
avrdude: ser_open(): can't open device "/dev/ttyUSB0": Permission 
deniedioctl("TIOCMGET"): Inappropriate ioctl for device.
```
To fix it, enter the command:

```bash
$ sudo usermod -a -G dialout <XXXX>
$ sudo chmod a+rw /dev/ttyUSB0
```
 Repace the <> by the username of the PC.

**5.** Create a new file in the following destination:

```bash
cd /etc/udev/rules.d/
sudo gedit 98-arduino.rules
```
**6.** Add this content to the file, save and exit the created file 98-arduino.rules :

```bash
KERNEL=="ttyUSB*", ATTRS{idVendor}==" XXXX", ATTRS{idProduct}=="XXXX", MODE="0777", SYMLINK+="arduino_mega", GROUP="dialout"
```

Replace XXXX by your vendor and product id, by using the command "dmesg" after connect you Arduino to the USB port.

**7.** Restart the udev service by:

```bash
sudo udevadm trigger
```

**8.** Check if your Arduino port was linked correctly:

```bash
ls -l /dev/arduino_mega
```

## YDLidar connection

**1.** Clone package to your catkin's workspace src folder:
```bash
cd ~/catkin_ws/src
git clone https://github.com/EAIBOT/ydlidar.git
cd ~/catkin_ws
catkin_make
```

**2.** Create the name "/dev/ydlidar" for YDLIDAR in udev rules by running the scripts:
```bash
roscd ydlidar/startup
sudo chmod 777 ./*
sudo sh initenv.sh
```

**3.** Connect the lidar to any USB port and test it by:
```bash
roslaunch ydlidar display.launch
```

# Usage

Developing...





