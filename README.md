# Week-1-2-Introduction-to-ROS2

Install Ubuntu 24.04 LTS
https://www.youtube.com/watch?v=kDosGTdwqO0&list=LL&index=1&t=1s

1) Native
2) WSL
3) VMware fusion is free for personal use
4) Docker
5) ....

VMware shared folder mount:
/usr/bin/vmhgfs-fuse .host:/BME/ROS2-lessons /home/david/ros2_ws/src/ROS2-lessons -o subtype=vmhgfs-fuse,allow_other
(https://www.liquidweb.com/blog/create-vmware-shared-folder/)

Install ROS:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Setup .bashrc:
source /opt/ros/jazzy/setup.bash
https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

Run samples:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#try-some-examples
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener

turtlesim
RQT, rqt_graph

Create colcon workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

Create our first python package:
ros2 pkg create --build-type ament_python bme_ros2_tutorials_py

Create hello world
```python
#!/usr/bin/env python3

# Main entry point, args is a parameter that is used to pass arguments to the main function
def main(args=None):
    print("Hello world!")

# Check if the script is being run directly
if __name__ == '__main__':
    main()
```

Edit setup.py
    1) packages
    2) entry point

build

source: source install/setup.bash
this we add to .bashrc

Upgrade hello world to ROS hello world

```python
#!/usr/bin/env python3
import rclpy # Import ROS2 python interface

# Main entry point, args is a parameter that is used to pass arguments to the main function
def main(args=None):
    rclpy.init(args=args)                          # Initialize the ROS2 python interface
    node = rclpy.create_node('python_hello_world') # Node constructor, give it a name
    node.get_logger().info("Hello ROS2!")          # Use the ROS2 node's built in logger
    node.destroy_node()                            # Node destructor
    rclpy.shutdown()                               # Shut the ROS2 python interface down

# Check if the script is being run directly
if __name__ == '__main__':
    main()
```

Create our python publisher
```python
#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String # Import 'String' from ROS2 standard messages
import time

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('python_publisher')
    # Register the node as publisher
    # It will publish 'String' type to the topic named 'topic' (with a queue size of 10)
    publisher = node.create_publisher(String, 'topic', 10)

    msg = String()                     # Initialize msg as a 'String' instance
    i = 0
    while rclpy.ok():                  # Breaks the loop on ctrl+c
        msg.data = f'Hello World: {i}' # Write the actual string into msg's data field
        i += 1
        node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)         # Let the node publish the msg according to the publisher setup
        time.sleep(0.5)                # Python wait function in seconds

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

let's see it with echo and rqt

Let's make it more OOP:

HERE

Let's make it in cpp, create a new package, it's less convinient in ROS2 to mix C++ and Python nodes within a package...
ros2 pkg create --build-type ament_cmake bme_ros2_tutorials_cpp

Cpp publisher here

build, run

echo, rqt

create python subscriber

rqt, rqt_graph, language agnostic pub-sub

create cpp subscriber

create launchfiles in a launch package









Build the workspace:
colcon build


Create first node:



Source the worspace:
source install/setup.bash


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
