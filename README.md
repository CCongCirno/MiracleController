# MiracleController
## environment

ROS2/humble

**fmt**

```
git clone https://github.com/fmtlib/fmt.lib
cd fmt
mkdir build
cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
make
sudo make install
```

## compile & run

Config USB2CAN

```
cd MiracleController
sudo bash ./CAN_init.sh
```

DBus node

```
cd MiracleController
colcon build
source install/setup.bash
ros2 run dbus_dr16 dbus_dr16_node
```

Chassis node

```
source install/setup.bash
ros2 run chassis chassis_node
```

