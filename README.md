works for hydro

# build the wet workspace (rosbuild)
===========================================
cd  ~/hsma/rosbuild/ 
gitclone https://github.com/oh-cpt/hsmakata.git
rosws set hsmakata_*
source setup.bash
rosmake -a

and start the dependency hunt - ENJOY!

# build the dry workspace (catkin)
===========================================
cd ~/hsma/catkin_ws/src
gitclone https://github.com/oh-cpt/ikfast_plugin.git
cd ~/catkin_ws/
catkin_make
ln -s ~/ros-hydro/dry/uos-ros-pkg/katana_driver/katana_moveit_ikfast_plugin/ src/




