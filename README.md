# allegro_with_ft_sensors
Allegro Hand v4 Linux Control, combined with 6-axis miniature F/T sensors from aidin robotics

Dist: Ubuntu 20.04, ROS Noetic

HW: Allegro Hand V4 (Right), AIDIN ROBOTICS 6-Axis Miniature Sensor

Prerequisites: [PCAN](https://www.peak-system.com/fileadmin/media/linux/index.htm)
* Follow the instruction from <https://www.allegrohand.com/ah-v4-linux-project>

## Error Revision Notes
- Allegro API from simlab utilizes compressed ID -> Can't divide force/torque of FT
> Devided by using full ID address (modified CANapi.cpp from original files)


- Now constructing ROS ENV (Publishing joint state, FT / Subscribing desired allegro joint config)
