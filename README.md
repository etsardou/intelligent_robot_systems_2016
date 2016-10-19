##Intelligent Robot Systems 2016-2017

This code was created in order to allow for experimentation, towards developing specific modules of an autonomous simulated vehicle that performs full exploration **and coverage** of an unknown a priori environment. Next, you can find instructions on how to setup the code in your PC / laptop, the description of the code, as well as the challenges.

The perfect score is 90 points (30 pts = 1 degree)!

- [Installation / setup](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/documentation/setup.md)
- [Modules description](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/documentation/structure.md)
- [How to execute the code](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/documentation/launch_code.md)

---

Challenge 1 [*10 pts*]: **Laser-based obstacle avoidance**

You must fill the part of the code that calculates linear and rotational velocities using the LIDAR values. The objective is for the robot to wander but not collide to obstacles. Please add the code [here](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/art_autonomous_exploration/src/speeds_assignment.py#L75) and [here](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/art_autonomous_exploration/src/speeds_assignment.py#L117).

Notes: The robot **must** have a maximum absolute linear speed of **0.3 m/s** and maximum absolute rotational speed **0.3 rad/sec**. In order to check this functionality turn [this](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/art_autonomous_exploration/config/autonomous_expl.yaml#L9) to ```False```.

---

Challenge 2 [*5 pts*]: **Path visualization**

This task is about making the path visible to RViz. Please add the code [here](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/art_autonomous_exploration/src/navigation.py#L233). The ```self.robot_perception.resolution``` and ```self.robot_perception.origin``` parameters may be useful, thus you must understand how the robot perception module works.

In order to test it, the exploration mode must be enabled, thus turn [this](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/art_autonomous_exploration/config/autonomous_expl.yaml#L9) to ```True```. This value will be kept ```True``` for all the remaining tasks.

---

Challenge 3 [*10 pts*]: **Path following**

This task is about producing the correct velocities for the robot to follow the produced path. Please add the code [here](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/art_autonomous_exploration/src/navigation.py#L282). The known parameters are the global robot pose acquired from the SLAM algorithm and the next subtarget.

Again, the robot **must** have a maximum absolute linear speed of **0.3 m/s** and maximum absolute rotational speed **0.3 rad/sec**.

---

Challenge 4 [*15 pts*]: **Path following & obstacle avoidance**

This task is about combining the path following and obstacle avoidance velocities using a strategy like motor schema, subsumption or a hubrid one. Please fill the code [here](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/art_autonomous_exploration/src/speeds_assignment.py#L111).

Again, the robot **must** have a maximum absolute linear speed of **0.3 m/s** and maximum absolute rotational speed **0.3 rad/sec**.

---

Challenge 5 [*5 pts*]: **Smarter subgoal checking**

This task is about making the subgoal checking routine smarter. Right now it checks if the next subtarget has been approached. But what if the robot reaches the end or a later subgoal without visiting the next? Please fill your code [here](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/art_autonomous_exploration/src/navigation.py#L102).

---

Challenge 6 [*15 pts*]: **Smart target selection**

This task is about finding a smart approach in the target selection problem. Remember that this approach must work well in **a priori unknown environments** which may be quite heterogeneous in structure (e.g. wide, narrow or both). The available tools you have are the robot's pose, the so far explored map, the coverage field containing information on what is already covered by the robot, the Brushfire field of the OGM, the skeleton of the OGM and a topological graph.

It is not necessary to use all of them. If you don't remember to **erase them** in order to speed up the exploration, since as you will notice more available information requires more time to calculate. 

Also there is a possibility of path planning to fail for specific targets that are close to obstacles or that they exist in a "strage" topology. In that case you must write code to deal with these cases (e.g. select a more naive target selection method). Bear in mind that the robot should always have a target to reach in order for the exploration to end!.

Please alter [this](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/art_autonomous_exploration/src/target_selection.py#L39) code.

---

---

Extra Challenge 1 [*10 pts*]: **Path optimization / alteration**

The path planning module produces length optimal paths using an A* algorithm, along with a uniform PRM graph. This doesn't mean that the length optimal paths will result in less coverage / exploration time.

Investigate ways to alter the path, aiming at increasing the coverage rate, inserting code [here](https://github.com/etsardou/intelligent_robot_systems_2016/blob/master/art_autonomous_exploration/src/navigation.py#L214) (easy approach), or directly altering the path planning code [here](https://github.com/etsardou/intelligent_robot_systems_2016/tree/master/art_ogmpp) (hard approach).

---

Extra Challenge 2 [*10 pts*]: **Algorithmic optimization**

You will notice that a serious amount of time is being spent in calculations concerning the target selection. Feel free to alter whatever code you want in order to optimize it! Optimizations already exist using the Cffi library, where C code is being executed within Python. You can follow this approach or improve Python code.

---

Extra Challenge 3 [*10 pts*]: **Surprize me**

Open challenge! Do something that I do not expect!

---

**How to test different maps**

- Paint a map!
- Add the png file [here](https://github.com/stdr-simulator-ros-pkg/stdr_simulator/tree/autonomous_systems/stdr_resources/maps) and create a yaml file for this map. Leave all values as they are, except for the image name.
- Change [this](https://github.com/stdr-simulator-ros-pkg/stdr_simulator/blob/autonomous_systems/stdr_launchers/launch/server_with_map_and_gui_plus_robot.launch) launcher by declaring the new yaml file.