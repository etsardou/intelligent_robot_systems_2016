# Files and structure

The ```Intelligent Robot Systems 2016-2017``` repository contains three ROS packages, named ```art_autonomous_exploration```, ```art_dummy_slam``` and ```art_ogmpp```. These are: 

- ```art_dummy_slam```: Provides the SLAM algorithms, i.e. the environment's mapping. It must be noted that a dummy SLAM is executed (the computed map will always be correct and not deformed), in order for the code to be able to run in slower machines, or even VMs. The map is provided via a ROS topic.
- ```art_ogmpp```: Provides a path planning algorithm using A* and uniform decomposition PRM. The path planning can be called via a ROS service.
- ```art_autonomous_exploration```: Contains the robot controller, i.e. the Python code that controls the simulated robot. It reads the map from the ROS topic and invokes the path planning ROS service whenever needed.

The system's structure follows:
	
```
                                 +---------+              +----------------+
                                 | main.py |              | STDR Simulator |
                                 +----^----+              +--------------+-+
                                      |                                  |
                            +---------+---------+                        |
           +----------------> speeds_assignment <----------+             |
           |                +---------^---------+          |             |
           |                          |                    |             |
           |                          |                    |             |
+----------+------------+ +-----------+-----------+ +------+-----+       |
| sonar_data_aggregator | | laser_data_aggregator | | navigation <       |
+-----------------------+ +-----------------------+ +------^-----+       |
                                                           |             |
             +----------------------+----------------------+             |
             |                      |                      |             |
     +-------+-------+    +---------+--------+   +---------+--------+    |
     | path_planning |    > target_selection |   | robot_perception <----+
     +-----^---------+    +---------^--------+   +----------------^-+
           |                        |                             |
           |              +---------+------------+                |
           |      +-----------+           +--------------+        |
+-------+--+      | utilities +------+---->  brushfires  |        |
| ogmpp |         +-----+-----+      |    +--------------+     +--+---+
+-------+               |            |                         | SLAM |
                        |            |      +----------+       +------+
                        +-------------------> topology |
                                     |      +----^-----+
                         +-----------+---+       |
                         | cpp_functions +-------+
                         +---------------+

```

- ```main.py``` is the frontend of the system. Contains the main function of the controller
- ```speeds_assignment``` contains methods for assigning speeds to the robot
- ```sonar_data_aggregator``` and ```laser_data_aggregator``` get the sonar and laser measurements from the STDR Simulator
- ```navigation``` contains the system's "intelligence". It controls the whole exploration workflow.
- ```path_planning``` invokes the ```art_ogmpp``` ROS service to get the path form one point to another
- ```target_selection``` selects the next best target and returns it to ```navigation```
- ```robot_perception``` maintains information concerning the robot status in the environment such as the map, the robot's pose, the coverage field etc.
- ```utilities``` contains helper functions such as visualization, prints etc
- ```brushfires``` contains implementations of brushfire algorithms, applied on the OGM
- ```topology``` contains functionalities to extract a topological graph from the OGM
- ```cpp_functions``` contains C implementations of algorithms using Cffi in order to boost performance