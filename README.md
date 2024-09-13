# Week-1-2-Introduction-to-ROS2

Install Ubuntu 24.04 LTS
https://www.youtube.com/watch?v=kDosGTdwqO0&list=LL&index=1&t=1s

1) Native
2) WSL
3) VMware fusion is free for personal use
4) Docker
5) ....

VMware shared folder mount:
/usr/bin/vmhgfs-fuse .host:/ /home/david/Documents -o subtype=vmhgfs-fuse,allow_other
(https://www.liquidweb.com/blog/create-vmware-shared-folder/)

Install ROS:
https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html

Setub .bashrc:


Run samples:
https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html#try-some-examples
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener

Install Gazebo:
compatibility: https://gazebosim.org/docs/harmonic/ros_installation/
install binary: https://gazebosim.org/docs/harmonic/install_ubuntu/
test it: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html

Various troubleshoot:
https://robotics.stackexchange.com/questions/111547/gazebo-crashes-immediately-segmentation-fault-address-not-mapped-to-object-0
https://github.com/gazebosim/gz-sim/issues/1492
https://github.com/gazebosim/gz-gui/issues/618
https://github.com/gazebosim/gz-sim/issues/1116#issuecomment-1142388038

Run Gazebo:
QT_QPA_PLATFORM=xcb gz sim shapes.sdf --render-engine ogre --render-engine-gui-api-backend opengl
