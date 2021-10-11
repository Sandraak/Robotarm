# Run demo application
source /opt/ros/foxy/setup.bash
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select robotarm
. install/setup.bash
ros2 run robotarm demo
