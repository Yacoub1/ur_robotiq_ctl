# urs_robotiq_ctl
Universal Robot Scripts Package to control Robotiq grippers, this package impliment a socket communcation pacake with UR CB3 controller. Then call these commands throug a ROS service that defined three main parameter which are 'force', 'pose' and 'speed'.

## Requirements

1. https://github.com/ros-industrial/universal_robot
2. https://github.com/ros-industrial/robotiq
3. https://www.universal-robots.com/plus/products/robotiq/robotiq-2f-140/

## How to use?
1. Install the package in the ROS Catkin workspace
  '''
  git clone https://github.com/Yacoub1/ur_robotiq_ctl.git
  '''
3. Build the package
    '''
    ctakin_make
    '''
    
4. Start the ROS Service Node

  '''
  rosrun ur_robotiq_ctl 2f140_srv_node
  '''
## How to test?
Run the **2f140_client_node.cp** which request the ROSSRV with the following parameter:
* force: 100 N
* speed: 10 mm/s
* pose: 50

Code:
* '''
rosrun ur_robotiq_ctl 2f140_client_node
'''
This parameter can be changed in the **2f140_client_node.cp** file, lines (18-20), and then re build the packge and rerun **2f140_client_node.cp**.
