
To run robot arm

- Open laptop, login using password: <pwd>
- Plug in robot power
- Plug leap into laptop (port near back)
- Arange Leap with led facing person
- Plug trossen USB into laptop (port near front)

In terminal type
   sudo chmod 777 /dev/ttyUSB0
   password: <pwd>
   roslaunch phantomx_rst arm.launch

Create another terminal (e.g. using ctrl-alt-T)
In new terminal type

   roslaunch leap_motion leap_robot_gxyzp.launch

