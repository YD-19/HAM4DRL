# HAM4DRL
A Holistic Assessment Method For Deep Reinforcement Learning


Open a Terminal

`git clone git@github.com:YD-19/HAM4DRL.git`

`cd ~/HAM4DRL/src`

`git clone git@github.com:ROBOTIS-GIT/turtlebot3.git`

`git clone git@github.com:ROBOTIS-GIT/turtlebot3_msgs.git`

if you want to train in our small environment, you need to modify the description file in turtlebot3 package.

`roscd turtlebot3_description/urdf && gedit turtlebot3_waffle_pi.gazebo.xacro`

find the base_scan model and modify as follow:

`<scan>`

`    <horizontal>`

`    <samples>24</samples>`

`    <resolution>1</resolution>`

`    <min_angle>-1.57</min_angle>`

`    <max_angle>1.57</max_angle>`

`    </horizontal>`

`</scan>`


After modification, make the files

`cd ~/HAM4DRL && catkin_make`

Open a new terminal:

`source ~/HAM4DRL/devel/setup.bash`

`roslaunch turtlebot_DDPG acps_ddpg.launch`

Open another terminal:

`source ~/HAM4DRL/devel/setup.bash`

`rosrun turtlebot_DDPG ddpg_main_1804.py`



