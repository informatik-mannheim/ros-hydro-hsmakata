====================
|| HOW TO INSTALL ||
====================

# build the wet workspace (rosbuild)
===========================================
mkdir -p ~/hsma/rosbuild_ws/ 
cd  ~/hsma/rosbuild_ws/
rosws init
source setup.bash
git clone https://github.com/oh-cpt/hsmakata.git
rosws set hsmakata/
source setup.bash
rosmake -a

and start the dependency hunt - ENJOY!

# build the dry workspace (catkin)
===========================================
mkdir -p ~/hsma/catkin_ws/src
cd ~/hsma/catkin_ws/src
git clone https://github.com/oh-cpt/ikfast_plugin.git
cd ~/hsma/catkin_ws/
catkin_make


====================
|| REQUIRENMNETS  ||
====================

a. for communication over ethernet:
   katana ip is set to 192.168.168.232. Configure your interface to
   an address in that range e.g. 192.168.168.200 and a subnet mask 
   of 255.255.0.0.

b. for communication over usb:
   katana ip is set to 192.168.1.1. Configure your interface to 
   an address in that range e.g. 192.168.1.200 and a subnet mask 
   of 255.255.255.0.
   remove <param name="ip" type="string" value="192.168.168.232"/> in 
   hsmakata_bring/launch/katana.launch

c. set KATANA_TYPE="katana_400_6m180" in your .bashrc


====================
||  HOW TO START  ||
====================

1. driver and planner start up

   roslaunch hsmakata_bringup hsmakata.launch

   roslaunch hsmakata_bringup moveit.launch

2. pick n place demo

   rosrun hsmakata_pick_n_place hsmakata_pick_n_place
   



