# cobots19
Main repository for bachelor work at Syscon 2019

## Required packages
Packages used (ros melodic) for this project:  
Universal-robot: https://github.com/ros-industrial/universal_robot/tree/kinetic-devel  
UR-modern-driver(modified for ur-modern-driver): https://github.com/endre90/ur_modern_driver  
Robotiq: http://wiki.ros.org/robotiq  
Optoforce: https://github.com/OptoForce/etherdaq_ros.git  
ROS Industrial Core: https://github.com/ros-industrial/industrial_core

## Prerequisits
### Software
* ROS installed and sourced. Catkin installed. See [ROSWiki](http://wiki.ros.org/) for more information.

 To avoid having to source the environment manually with every new terminal, we can automate using the `.bashrc` file. Do this **after** step 3 described below.
 Open the file using `nano ~/.bashrc` and append the following lines after **changing them** according to your environment:
 
 ```
 source /opt/ros/melodic/setup.bash
 source ~/<Name of Catkin WS>/devel/setup.bash
 export LC_NUMERIC="en_US.UTF-8"
 ```

 *Restart your terminal* to apply the changes.
* git installed.

#### If using a gripper
* Pip installed. Pip can be installed with `sudo apt update && sudo apt install python-pip`
* The 'pymodbus' package for python installed. This is needed for the gripper and can be installed using 'sudo pip install pymodbus'
* Your user is a member of the `dialout` group. This can be done by running `sudo usermod -a -G dialout $USER` and after that restarting your system.

### Hardware
* UR10 Robot with a working network connection and a known ip-address.
* Optoforce controller with working network connection and knwon ip-address. The actual sensor should be mounted on the robot's end affector with the correct orientation.
* A computer that can run ROS, connected on the same network as the robot and Optoforce controller.
* *Optional:* A Robotiq 2 finger gripper that is mounted to the force/torque sensor, and connected to the computer running ROS.

## Set-up

1. Create a catkin workspace, preferably in you home directory. See [Catkin Tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
2. Navigate to `<Name of you workspace>/src` and use `git clone <URL to github repo>` to clone all packages listed under "Required packages" individually. Make sure to also clone **this** repository.
3. Build the workspace by running `catkin_make` in the root of the workspace (might take some time).
4. Modify the launch file `<Name of Catkin WS>/src/cobots19/launch/launchDriver.launch` so that the 'optoforce_etherdaq_driver' has the ip address to the optoforce controller.
5. If you are **not** using a gripper, you can comment the "Robotiq2FGripperRtuNode" line to avoid starting the gripper driver. 
6. The drivers can now be started by running `roslaunch cobots19 launchDriver.launch robot_ip:=<IP address of robot controller>`. If this command cannot be run, make sure that you have sourced the environment properly (see "Prerequisits").
