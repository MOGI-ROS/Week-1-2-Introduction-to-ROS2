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

turtlesim:
sudo apt install ros-jazzy-turtlesim

ros2 run turtlesim turtlesim_node

ros2 run turtlesim turtle_teleop_key

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
    print("Hello, world!")

# Check if the script is being run directly
if __name__ == '__main__':
    main()
```

Edit setup.py

1: packages


```python
    #packages=find_packages(exclude=['test']),
    packages=[package_name, 'scripts'],
```


2: entry point

```python
    entry_points={
        'console_scripts': [
            'py_hello_world = scripts.hello_world:main'
        ],
    },
```

build:
`colcon build`

source: source install/setup.bash
this we add to .bashrc:
source ~/ros2_ws/install/setup.bash

run it:
ros2 run bme_ros2_tutorials_py py_hello_world


Upgrade hello world to ROS hello world

```python
#!/usr/bin/env python3
import rclpy # Import ROS2 python interface

# Main entry point, args is a parameter that is used to pass arguments to the main function
def main(args=None):
    rclpy.init(args=args)                          # Initialize the ROS2 python interface
    node = rclpy.create_node('python_hello_world') # Node constructor, give it a name
    node.get_logger().info("Hello, ROS2!")          # Use the ROS2 node's built in logger
    node.destroy_node()                            # Node destructor
    rclpy.shutdown()                               # Shut the ROS2 python interface down

# Check if the script is being run directly
if __name__ == '__main__':
    main()
```

ros2 run bme_ros2_tutorials_py py_hello_world

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
        msg.data = f'Hello, world: {i}' # Write the actual string into msg's data field
        i += 1
        node.get_logger().info(f'Publishing: "{msg.data}"')
        publisher.publish(msg)         # Let the node publish the msg according to the publisher setup
        time.sleep(0.5)                # Python wait function in seconds

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Edit setup.py

run it:
ros2 run bme_ros2_tutorials_py py_publisher


let's see it with echo and rqt

ros2 topic echo /topic

rqt_image

Let's make it more OOP in a new file:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node     # Import ROS2 Node as parent for our own node class
from std_msgs.msg import String

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__("python_publisher_oop")
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)     # Timer callback, period in seconds, not frequency!
        self.i = 0
        self.msg = String()
        self.get_logger().info("Publisher OOP has been started.")

    def timer_callback(self):                                        # Timer callback function implementation
        self.msg.data = f"Hello, world: {self.i}"
        self.i += 1
        self.get_logger().info(f'Publishing: "{self.msg.data}"')
        self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode() # node is now a custom class based on ROS2 Node
    rclpy.spin(node)         # Keeps the node running until it's closed with ctrl+c
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

Edit setup.py, run it:
ros2 run bme_ros2_tutorials_py py_publisher_oop


Let's make it in cpp, create a new package, it's less convinient in ROS2 to mix C++ and Python nodes within a package...
it's possible though, see example:
...

Create new package:
ros2 pkg create --build-type ament_cmake bme_ros2_tutorials_cpp

Cpp publisher here

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPublisherNode : public rclcpp::Node
{
public:
    MyPublisherNode() : Node("cpp_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_     = this->create_wall_timer(std::chrono::milliseconds(500),
                                                std::bind(&MyPublisherNode::publishString, this));
        RCLCPP_INFO(this->get_logger(), "CPP publisher has been started.");
    }

private:
    void publishString()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello, world: " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        publisher_->publish(msg);
    }

    size_t count_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
```

Edit the CMakeLists.txt:
```bash
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(publisher_cpp src/publisher.cpp)
ament_target_dependencies(publisher_cpp rclcpp std_msgs)

install(TARGETS
  publisher_cpp
  DESTINATION lib/${PROJECT_NAME})
```
build, run:
ros2 run bme_ros2_tutorials_cpp publisher_cpp


echo, rqt

create python subscriber
```python
#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('python_subscriber')

    def subscriber_callback(msg):                      # Subscriber callback will be invoked every time when a message arrives to the topic it has subsctibed
        node.get_logger().info(f"I heard: {msg.data}")

    # Register the node as a subscriber on a certain topic: 'topic' (with a certain data type: String)
    # and assign the callback function that will be invoked when a message arrives to the topic
    # with a queue size of 10 which determines how many incoming messages can be held in the subscriber’s
    # queue while waiting to be processed by the callback function
    subscriber = node.create_subscription(String, 'topic', subscriber_callback, 10) 

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Add node to setup.py
build
run:


create oop subscriber based on our previous template
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriberNode(Node):
    def __init__(self):
        super().__init__("python_subsciber_oop")
        self.subscriber_ = self.create_subscription(String, 'topic', self.subscriber_callback, 10)
        self.get_logger().info("Subsciber OOP has been started.")

    def subscriber_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

rqt, rqt_graph, language agnostic pub-sub
PIC!

create cpp subscriber
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MySubscriberNode : public rclcpp::Node
{
public:
    MySubscriberNode() : Node("cpp_subscriber")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MySubscriberNode::subscriber_callback, this, std::placeholders::_1));
    }

private:
    void subscriber_callback(const std_msgs::msg::String & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MySubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

create launchfiles in a launch package withou specifying any build type (it sets it up as cmake package)
ros2 pkg create bme_ros2_tutorials_bringup

Create a launch folder
mkdir launch

We can freely delete include and src folders:
rm -rf include/ src/

Add to CMakeLists.txt:
```bash
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```


Create a new launch file, publisher_subscriber.py
touch publisher_subscriber.py

create launch file, first only with the publisher:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = Node(
        package="bme_ros2_tutorials_py",
        executable="py_publisher",
        name="my_publisher"
    )

    ld.add_action(publisher_node)
    return ld
```

Build the workspace:
colcon build

Source the worspace:
source ~/ros2_ws/install/setup.bash

Execute the launchfile:
ros2 launch bme_ros2_tutorials_bringup publisher_subscriber.py

We can notice that our node is now called `my_publisher` instead of `python_publisher` as earlier (and as we hardcoded in the publisher's python code). With launch files we can easily rename our nodes for better handling as our application scales up.

We can use the ros2 node list tool to list our nodes and the output should look like this:
```bash
david@david-ubuntu24:~/ros2_ws$ ros2 node list 
/my_publisher
```

Let's add the subscriber too:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = Node(
        package="bme_ros2_tutorials_py",
        executable="py_publisher",
        name="my_publisher",
    )

    subscriber_node = Node(
        package="bme_ros2_tutorials_py",
        executable="py_subscriber",
        name="my_subscriber",
    )

    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)
    return ld
```

rebuild, and run
ros2 launch bme_ros2_tutorials_bringup publisher_subscriber.py

both nodes are started, we can visually see using rqt_graph or the cli tool:
```bash
david@david-ubuntu24:~/ros2_ws$ ros2 node list 
/my_publisher
/my_subscriber
```

There is another useful cli tool which list the topics:
```bash
david@david-ubuntu24:~/ros2_ws$ ros2 topic list 
/parameter_events
/rosout
/topic
```

With launch files we can not just rename our nodes, but we can also re-map topics, let's try to remap the existing `topic` to `another_topic`:

Add remapping to the publisher:
```python
        remappings=[
            ("topic", "another_topic")
        ]
```

Build and run

with ros2 topic list:
david@david-ubuntu24:~/ros2_ws$ ros2 topic list 
/another_topic
/parameter_events
/rosout
/topic

Now the subsriber is not triggered because it's still listening to the topic while the publisher sends its messages to another_topic, we can see it visually that our two nodes are unconnected with rqt_grap:


We can also use the node info cli tool to check what are the published topics and the subscriptions for a specific node:
ros2 node info /my_publisher

Let's remap our subscriber too!

Don't forget to rebuild!

Now let's add more publishers and subscribers, also mixing our cpp and python nodes:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    cpp_publisher_node = Node(
        package="bme_ros2_tutorials_cpp",
        executable="publisher_cpp",
        name="my_cpp_publisher",
    )

    py_publisher_node = Node(
        package="bme_ros2_tutorials_py",
        executable="py_publisher",
        name="my_py_publisher",
        remappings=[
            ("topic", "another_topic")
        ]
    )

    py_subscriber_node1 = Node(
        package="bme_ros2_tutorials_py",
        executable="py_subscriber",
        name="my_py_subscriber1",
    )

    py_subscriber_node2 = Node(
        package="bme_ros2_tutorials_py",
        executable="py_subscriber_oop",
        name="my_py_subscriber2",
    )

    cpp_subscriber_node1 = Node(
        package="bme_ros2_tutorials_cpp",
        executable="subscriber_cpp",
        name="my_cpp_subscriber1",
        remappings=[
            ("topic", "another_topic")
        ]
    )

    ld.add_action(cpp_publisher_node)
    ld.add_action(py_publisher_node)
    ld.add_action(py_subscriber_node1)
    ld.add_action(py_subscriber_node2)
    ld.add_action(cpp_subscriber_node1)
    return ld
```


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
