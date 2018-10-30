# RoboPlay
for playing with trossen robot arm

Master-slave using 2 robot arms

Master robot (operated by humans) commands
  sudo chmod 777 /dev/ttyUSB0
  roslaunch phantomx_rst arm.launch
  rosrun two_robot_control read_deg_of_servo.py
  
Slave robot (follows the master robot) commands
  sudo chmod 777 /dev/ttyUSB0
  roslaunch phantomx_rst arm.launch
  rosrun arbotix_python arbotix_fk
