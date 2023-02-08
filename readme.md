# Prepare
## with platformIo(recommand)
1. install [vscode](https://code.visualstudio.com/) 
2. install [platformIO](https://platformio.org/install/ide?install=vscode)
3. import project
1. connect esp8266
4. Click the right arrow(Platform:Upload) on the blue bar at the bottom of vscode to flash

# Cable Connect
## INO260
VCC - 3V3(esp8266)
GND - common! ground
SCL - SCL(esp8266 default 5)
SDA - SDA(esp8266 default 4)
Alert
VBus
Vin+ - emergency button
Vin- - Buck module

## ESP8266
3V - VCC(INO260)
USB - 5v(5V Buck module)
SCL - SCL(INO260)
SDA - SDA(INO260)
GND - common! ground

# ROS Server
##  Install
1. Install ROS (recommand [Melodic](http://wiki.ros.org/melodic/Installation))

2. Install rosserial_python 

```apt-get install ros-melodic-rosserial-python```


## start rosserial_server
```source /opt/ros/melodic/setup.bash```

```rosrun rosserial_python serial_node.py tcp``` (default)
```rosrun rosserial_python node.py tcp```   (on Roboy)
