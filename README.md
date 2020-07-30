# Pick and place

Simulation of a pick and place pipeline, the robotic arm has to take some box and place them on a destination table. The simulation is divided into two main part: detection and execution.

We recreated a simple gazebo scene as shown in the figure:

![gazebo](https://github.com/SuperDiodo/pick_and_place/blob/master/images/gazebo_image.png)

The kinect will publish the detected poses into the ```\poses``` topic. The execution simulation can be done in **Rviz**, with the help of **Moveit!** 

![rviz](https://github.com/SuperDiodo/pick_and_place/blob/master/images/rviz_image.png)



### Start the simulation

Open a terminal tab or window for each file:

1. ```roslaunch vision scene.launch```: this will start gazebo.
2.  ``` roslaunch bigger_cr35_config demo.launch```: this will start rviz.
3. ```rosrun arm application```: this will add some collision objects into rviz.
4. ```rosrun vision vision_node```: this will show the detected target from gazebo and start the arm.



### Authors

**Alessio Saccuti**, master degree student in computer engineering, a.s@gmail.com

**Vito Cornettone**, master degree student in computer engineering, donvito.cornettone@crema.com



