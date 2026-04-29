# VID-Dataset

arxiv: [pdf](https://arxiv.org/abs/2103.11152)

Youtube: [video](https://youtu.be/K6Cks1QuyqY)

Bilibili: [video](https://www.bilibili.com/video/BV1s54y1a7x2?spm_id_from=333.999.0.0)

[![IMAGE ALT TEXT](https://github.com/ZJU-FAST-Lab/VID-Dataset/blob/main/image/vedio.gif)](http://www.youtube.com/watch?v=K6Cks1QuyqY)

## The Flight Platform

The [VID-Flight-Platform](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform) project contains [PCB design files](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform/tree/main/pcb), [MCU code](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform/tree/main/mcu),  [mechanical assembly drawings](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform/tree/main/assembly), [onboard ROS workspace](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform/tree/main/onboard_rosws) for communication and data collection.

[![IMAGE ALT TEXT](https://github.com/ZJU-FAST-Lab/VID-Dataset/blob/main/image/drone.png)](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform)

## Run the Rosbag with the time-sync tool

```
git clone https://github.com/ZJU-FAST-Lab/VID-Dataset.git
cd VID-Dataset
catkin_make
source devel/setup.bash
./tools/convertbag.sh [path of the ROS bag]
```

## ROS topic

|    | Description                               | Raw topic                      | Aligned topic        |
| -- | ----------------------------------------- | :----------------------------- | :------------------- |
| 1  | Realsense D435 depth image                | /camera/depth/image_rect_raw   | /synced/depth        |
| 2  | Realsense D435 left gray image            | /camera/infra1/image_rect_raw  | /synced/infra1       |
| 3  | Realsense D435 right gray image           | /camera/infra2/image_rect_raw  | /synced/infra2       |
| 4  | IMU data with hardware timestamp          | /djiros/imu_hwts               | /synced/imu          |
| 5  | PPS from N3 flight controller             | /djiros/pulse                  |                      |
| 6  | N3's 1Hz pulse detected by MCU2           | /m100withm3508/cap_n3pps       |                      |
| 7  | D435 camera's 60Hz pulse detected by MCU2 | /m100withm3508/cap_camerapulse |                      |
| 8  | Target rotor speed of 4 motors            | /m100withm3508/target_rpm      | /synced/target_rpm   |
| 9  | Target current of 4 motors                | /m100withm3508/ctrl_current    | /synced/ctrl_current |
| 10 | Measured speed and current of motor 1     | /m100withm3508/m3508_m1        | /synced/m3508_m1     |
| 11 | Measured speed and current of motor 2     | /m100withm3508/m3508_m2        | /synced/m3508_m2     |
| 12 | Measured speed and current of motor 3     | /m100withm3508/m3508_m3        | /synced/m3508_m3     |
| 13 | Measured speed and current of motor 4     | /m100withm3508/m3508_m4        | /synced/m3508_m4     |
| 14 | Measured speed of 4 motors                |                                | /synced/allrpm       |
| 15 | GPS from N3 flight controller             | /djiros/gps                    |                      |
| 16 | RTK                                       | /rtk_zhd_parser/GPS            |                      |
| 17 | Ground truth from motion capture system   | /vicon/m100/m100               |                      |

## Sequences

Download link: \\
[http://zjufast.kmras.com:9110/share.cgi?ssid=e08267c50fbe4e4bb9cdfe40f9fdb712](http://zjufast.kmras.com:9110/share.cgi?ssid=e08267c50fbe4e4bb9cdfe40f9fdb712)

Password:\\
viddataset@2021




|      | Environment | Feature        | Trajectory  | Weight <br/>(gram) | Duration<br/>(second) | Imu<br/>Imagery | Dynamical<br/>data | RTK  | Ground<br/>truth | Force<br/>sensor |
| :--: | :---------: | -------------- | :---------- | :----------------: | :-------------------: | :-------------: | :----------------: | :--: | :--------------: | :--------------: |
|  1   |   Outdoor   | Fast, with yaw | Rectangle   |       3547.2       |        113.21         |        ✔        |         ✔          |  ✔   |                  |                  |
|  2   |   Outdoor   | Slow, with yaw | Rectangle   |       3547.2       |        175.38         |        ✔        |         ✔          |  ✔   |                  |                  |
|  3   |   Outdoor   | with yaw       | Round       |       3541.7       |        147.34         |        ✔        |         ✔          |  ✔   |                  |                  |
|  4   |   Outdoor   | without yaw    | Round       |       3541.7       |        105.85         |        ✔        |         ✔          |  ✔   |                  |                  |
|  5   |   Outdoor   | with yaw       | 8-character |       3541.7       |        184.29         |        ✔        |         ✔          |  ✔   |                  |                  |
|  6   |   Outdoor   | without yaw    | 8-character |       3541.4       |        243.73         |        ✔        |         ✔          |  ✔   |                  |                  |
|  7   |    Night    | Fast, with yaw | Rectangle   |       3460.0       |        133.58         |        ✔        |         ✔          |  ✔   |                  |                  |
|  8   |    Night    | Slow, with yaw | Rectangle   |       3541.6       |        179.48         |        ✔        |         ✔          |  ✔   |                  |                  |
|  9   |    Night    | without yaw    | Round       |       3541.6       |         82.07         |        ✔        |         ✔          |  ✔   |                  |                  |
|  10  |    Night    | without yaw    | 8-character |       3547.7       |        121.87         |        ✔        |         ✔          |  ✔   |                  |                  |
|  11  |   Indoor    | Loadless       | Hovor       |       3096.1       |         79.04         |        ✔        |         ✔          |      |        ✔         |                  |
|  12  |   Indoor    | Loadless       | Round       |       3096.1       |        117.89         |        ✔        |         ✔          |      |        ✔         |                  |
|  13  |   Indoor    | Loadless       | 8-character |       3096.1       |        109.17         |        ✔        |         ✔          |      |        ✔         |                  |
|  14  |   Indoor    | Loaded         | Hovor       |       3372.2       |         80.11         |        ✔        |         ✔          |      |        ✔         |                  |
|  15  |   Indoor    | Loaded         | Round       |       3372.2       |        107.15         |        ✔        |         ✔          |      |        ✔         |                  |
|  16  |   Indoor    | Loaded         | 8-character |       3372.2       |        138.41         |        ✔        |         ✔          |      |        ✔         |                  |
|  17  |   Indoor    | Rope pulled    | Random      |       3101.5       |        126.53         |        ✔        |         ✔          |      |        ✔         |        ✔         |
|  18  |   Indoor    | Rope pulled    | Random      |       3101.5       |        155.38         |        ✔        |         ✔          |      |        ✔         |        ✔         |
|  19  | Motor test  |                |             |                    |         45.04         |                 |         ✔          |      |                  |        ✔         |
|  20  | Motor test  |                |             |                    |         45.08         |                 |         ✔          |      |                  |        ✔         |
|  21  | Motor test  |                |             |                    |         43.00         |                 |         ✔          |      |                  |        ✔         |
|  22  | Motor test  |                |             |                    |         43.84         |                 |         ✔          |      |                  |        ✔         |
