# BOSCH-BMI160-ROS-Library

Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.

BSD-3-Clause

This is a ROS Package of "BMI160 6 DOF IMU C++ driver". 

### To use in an another package:

1. Clone this repo into src folder
2. cd ~/catkin_ws/
3. catkin_make
4. source devel/setup.bash

5. Add CMakeLists.txt of the "other" package:
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ...
  imu_bmi160
)
```

6. Add package.xml of the "other" package:
```
  <build_depend>imu_bmi160</build_depend>
```

7. Copy bmi160_acc_example.cpp to the "other" package's /src folder, compile and run as an executable.

```
add_executable(bmi160_acc_example src/bmi160_acc_example.cpp)
target_link_libraries(bmi160_acc_example 
${catkin_LIBRARIES} 
)
```

8. This will print out acceleration values only:
```
rosrun other_package bmi160_acc_example
```