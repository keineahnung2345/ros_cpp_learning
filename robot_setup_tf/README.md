# robot_setup_tf

[Setting up your robot using tf](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF)

Compile:
```sh
catkin_make
```

Run:
```sh
roscore
```

In second terminal:

```sh
rosrun robot_setup_tf tf_broadcaster
```

In third terminal:

```sh
rosrun robot_setup_tf tf_listener
```

The result will be:

```
[ INFO] [1628389049.024391122]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389049.02
[ INFO] [1628389050.024274915]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389050.02
[ INFO] [1628389051.024214076]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389051.02
[ INFO] [1628389052.024201540]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389052.02
[ INFO] [1628389053.024222118]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389053.02
[ INFO] [1628389054.024237200]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389054.02
[ INFO] [1628389055.024194216]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389055.02
[ INFO] [1628389056.024280857]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389056.02
[ INFO] [1628389057.024210355]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389057.02
[ INFO] [1628389058.024196804]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389058.02
[ INFO] [1628389059.024205539]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389059.02
[ INFO] [1628389060.024210049]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389060.02
[ INFO] [1628389061.024221027]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389061.02
[ INFO] [1628389062.024215766]: base_laser: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1628389062.02
...
```
