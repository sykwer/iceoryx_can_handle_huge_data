# iceoryx_can_handle_huge_data
## Build & Install
We assume ROS2 libraries are already installed and the distribution is newer or equal to galactic.
```
source /opt/ros/galactic/setup.bash
colcon build
source install/setup.bash
```

## Huge Data (>2GiB) Communication via CycloneDDS
Launch a talker and a listener on different terminal windows respectively. 
```
ros2 run cpp_pubsub talker
ros2 run cpp_pubsub listener
```
You can see error messages indicating the limitation on the size of the messages.
```
1660812920.773519 [0]     recvUC: dropping oversize (2147483664 > 2147483647) sample 1 from remote writer 74e91001:b7595080:bd9c357:1503 rt/topic/interfaces::msg::dds_::DynamicSizeArray_
1660812923.804118 [0]     recvUC: dropping oversize (2147483664 > 2147483647) sample 2 from remote writer 74e91001:b7595080:bd9c357:1503 rt/topic/interfaces::msg::dds_::DynamicSizeArray_
1660812926.797288 [0]     recvUC: dropping oversize (2147483664 > 2147483647) sample 3 from remote writer 74e91001:b7595080:bd9c357:1503 rt/topic/interfaces::msg::dds_::DynamicSizeArray_
1660812929.817960 [0]     recvUC: dropping oversize (2147483664 > 2147483647) sample 4 from remote writer 74e91001:b7595080:bd9c357:1503 rt/topic/interfaces::msg::dds_::DynamicSizeArray_
1660812932.803308 [0]     recvUC: dropping oversize (2147483664 > 2147483647) sample 5 from remote writer 74e91001:b7595080:bd9c357:1503 rt/topic/interfaces::msg::dds_::DynamicSizeArray_
```

## Huge Data (>2GiB) Communication via IceOryx
Launch a RouDi daemon for IceOryx communication.
```
iox-roudi -c roudi_config.toml
```
Launch a talker and a listener on different terminal windows respectively.
```
CYCLONEDDS_URI=file://$PWD/cyclonedds.xml ros2 run cpp_pubsub iox_talker
CYCLONEDDS_URI=file://$PWD/cyclonedds.xml ros2 run cpp_pubsub iox_listener
```
Confirm that huge data (>2GiB) can be communicated powered by IceOryx.

![iceoryx_without_loan](https://user-images.githubusercontent.com/18254663/194262430-5131380c-c6ee-4f9a-bb83-26fb03a9952d.png)

You can also try true zero copy communication powered by the loan API.
```
CYCLONEDDS_URI=file://$PWD/cyclonedds.xml ros2 run cpp_pubsub iox_talker_loaned
CYCLONEDDS_URI=file://$PWD/cyclonedds.xml ros2 run cpp_pubsub iox_listener
```

Communication latency is drastically reduced by the true zero copy mechanism.

![iceoryx_with_loan](https://user-images.githubusercontent.com/18254663/194264016-86137ae2-804a-42c4-bfe6-141262d2ec36.png)

## For Galactic Users (rclcpp bug)
Apply this PR (https://github.com/ros2/rclcpp/pull/1657/files) as a patch, or the publisher and the subscriber will fail by `std::bad_alloc`.
This bug is fixed in Humble and later versions.
