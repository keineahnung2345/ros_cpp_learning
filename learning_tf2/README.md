# learning_tf2

## static broadcaster
[Writing a tf2 static broadcaster (C++)](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20%28C%2B%2B%29)

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
rosrun learning_tf2 static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
```
In third terminal:
```sh
rostopic echo /tf_static
```

The result will be:

```
transforms: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 1628389861
        nsecs: 271600946
      frame_id: "world"
    child_frame_id: "mystaticturtle"
    transform: 
      translation: 
        x: 0.0
        y: 0.0
        z: 1.0
      rotation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
---
```

## broadcaster
[Writing a tf2 broadcaster (C++)](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29)

Compile:
```sh
catkin_make
```

Run:
```sh
roslaunch learning_tf2 start_demo.launch
```
In another terminal:
```sh
rosrun tf tf_echo /world /turtle1
```

The result will be:

![broadcaster.gif](broadcaster.gif)

## listener
[Writing a tf2 listener (C++)](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29)

Compile:
```sh
catkin_make
```

Run:
```sh
roslaunch learning_tf2 start_demo.launch
```

The result will be:

![listener.gif](listener.gif)

## frame tf2 broadcaster
[Adding a frame (C++)](http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28C%2B%2B%29)

Compile:
```sh
catkin_make
```

Run:
```sh
roslaunch learning_tf2 start_demo.launch
```

The result will be:

![frame_tf2_broadcaster.gif](frame_tf2_broadcaster.gif)

## tf2 and time

[Learning about tf2 and time (C++)](http://wiki.ros.org/tf2/Tutorials/tf2%20and%20time%20%28C%2B%2B%29)


Compile:
```sh
catkin_make
```

Run:
```sh
roslaunch learning_tf2 start_demo.launch
```

The result is almost same as "listener".

## time travel

[Time travel with tf2 (C++)](http://wiki.ros.org/tf2/Tutorials/Time%20travel%20with%20tf2%20%28C%2B%2B%29)

Compile:
```sh
catkin_make
```

Run:
```sh
roslaunch learning_tf2 start_demo.launch
```

![time_travel_uncontrol.gif](time_travel_uncontrol.gif)
