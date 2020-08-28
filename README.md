# Pick and place simulation

Simulation of a pick and place pipeline, the robotic arm has to take some box and place them on a destination table. The simulation is divided into two main part: detection and execution.

The detection part is performed in Gazebo, we recreated a simple scene as shown in the figure:

![gazebo](https://github.com/SuperDiodo/pick_and_place/blob/master/images/gazebo_sim.png)

A ROS node called "vision_node" is responsible to process images in order to find objects position and orientation and publish them into the ```\poses``` topic.
For more info read "final_report.pdf" chapter 3.

The execution simulation is performed in **Rviz**, with the help of **Moveit!** 


![rviz](https://github.com/SuperDiodo/pick_and_place/blob/master/images/rviz_image.png)

The manipulator receives the poses, in order to perform grasping and motion planning. For more info read "final_report.pdf" chapter 4.

You can see the video here:

[![Watch the video](https://img.youtube.com/vi/DumeUdD6gvw/hqdefault.jpg)](https://www.youtube.com/watch?v=DumeUdD6gvw)


### Start the simulation

The project was developed in Ubuntu 18.04, with ROS melodic.

<h3> NOTE: This is a work in progess project, some error or bug may occour. To allow us improving please report us any problem. </h3>

1. Install file:
   1. **[ROS](http://wiki.ros.org/melodic/Installation)** 
   2. **[MoveIt!](https://moveit.ros.org/install/)**
   3. **[OpenCV](https://wiki.ros.org/vision_opencv)**: ```sudo apt-get install ros-melodic-vision-opencv```.

2. Drag the files in the **utils** folder directly in the gazebo models folder.

3. Launch!

   

For a simplest launch just digit in terminal ```roslaunch bigger_cr35_config simulation.launch```, or type in  a terminal tab or window for each file:

1. ```roslaunch vision scene.launch```: this will start gazebo.
2.  ``` roslaunch bigger_cr35_config demo.launch```: this will start rviz.
3. ```rosrun arm application```: this will add some collision objects into rviz.
4. ```rosrun vision vision_node```: this will show the detected target from gazebo and start the arm.



### Authors

**Alessio Saccuti**, master degree student in computer engineering, alessio.saccuti@gmail.com.

**Vito FIlomeno**, master degree student in computer engineering, filomeno.vito@gmail.com.

