First you must execute the following command:

```bash
source ~/catkin_ws/devel/setup.bash
```

In order not to execute the above command each time you open a terminal you can add it at the end of your ```~/.bashrc``` file.

To execute the code open a console and run:

```bash
roslaunch art_autonomous_exploration everything.launch
```

This will launch the simulator, the code and rviz.

In order to actually see the simulator's GUI execute:

```bash
roslaunch stdr_gui stdr_gui.launch
```
