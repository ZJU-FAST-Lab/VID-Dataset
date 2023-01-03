# VID-Dataset



arxiv: [pdf](https://arxiv.org/abs/2103.11152)

Youtube: [video](https://youtu.be/K6Cks1QuyqY)

Bilibili: [video](https://www.bilibili.com/video/BV1s54y1a7x2?spm_id_from=333.999.0.0)


[![IMAGE ALT TEXT](https://github.com/ZJU-FAST-Lab/VID-Dataset/blob/main/image/vedio.gif)](http://www.youtube.com/watch?v=K6Cks1QuyqY )

## The Flight Platform 

The [VID-Flight-Platform](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform) project contains [PCB design files](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform/tree/main/pcb), [MCU code](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform/tree/main/mcu),  [mechanical assembly drawings](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform/tree/main/assembly), [onboard ROS workspace](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform/tree/main/onboard_rosws) for communication and data collection.

[![IMAGE ALT TEXT](https://github.com/ZJU-FAST-Lab/VID-Dataset/blob/main/image/drone.png)](https://github.com/ZJU-FAST-Lab/VID-Flight-Platform )

## Run the Rosbag with the time-sync tool 

```
git clone https://github.com/ZJU-FAST-Lab/VID-Dataset.git
cd VID-Dataset
catkin_make
source devel/setup.bash
./tools/convertbag.sh [path of the ROS bag]
```

## ROS topic

|      | Description                               | Raw topic                      | Aligned topic        |
| ---- | ----------------------------------------- | :----------------------------- | :------------------- |
| 1    | Realsense D435 depth image                | /camera/depth/image_rect_raw   | /synced/depth        |
| 2    | Realsense D435 left gray image            | /camera/infra1/image_rect_raw  | /synced/infra1       |
| 3    | Realsense D435 right gray image           | /camera/infra2/image_rect_raw  | /synced/infra2       |
| 4    | IMU data with hardware timestamp          | /djiros/imu_hwts               | /synced/imu          |
| 5    | PPS from N3 flight controller             | /djiros/pulse                  |                      |
| 6    | N3's 1Hz pulse detected by MCU2           | /m100withm3508/cap_n3pps       |                      |
| 7    | D435 camera's 60Hz pulse detected by MCU2 | /m100withm3508/cap_camerapulse |                      |
| 8    | Target rotor speed of 4 motors            | /m100withm3508/target_rpm      | /synced/target_rpm   |
| 9    | Target current of 4 motors                | /m100withm3508/ctrl_current    | /synced/ctrl_current |
| 10   | Measured speed and current of motor 1     | /m100withm3508/m3508_m1        | /synced/m3508_m1     |
| 11   | Measured speed and current of motor 2     | /m100withm3508/m3508_m2        | /synced/m3508_m2     |
| 12   | Measured speed and current of motor 3     | /m100withm3508/m3508_m3        | /synced/m3508_m3     |
| 13   | Measured speed and current of motor 4     | /m100withm3508/m3508_m4        | /synced/m3508_m4     |
| 14   | Measured speed of 4 motors                |                                | /synced/allrpm       |
| 15   | GPS from N3 flight controller             | /djiros/gps                    |                      |
| 16   | RTK                                       | /rtk_zhd_parser/GPS            |                      |
| 17   | Ground truth from motion capture system   | /vicon/m100/m100               |                      |



## Sequences

The download link in the table is unavailable due to the expiration of the web disk. Please use the link below to download the data set

Download link: [Download](http://zjufast.tpddns.cn:9110/share.cgi?ssid=e977126ca1704d3b8b156164d3b3855b)

Password：vid



|      | Download link                                                | Environment | Feature        | Trajectory  | Weight <br/>(gram) | Duration<br/>(second) | Imu<br/>Imagery | Dynamical<br/>data | RTK  | Ground<br/>truth | Force<br/>sensor | File name                                                    |
| :--: | ------------------------------------------------------------ | :---------: | -------------- | :---------- | :----------------: | :-------------------: | :-------------: | :----------------: | :--: | :--------------: | :--------------: | :----------------------------------------------------------- |
|  1   | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ER751r6eJoFAj7Pcb7QMb50BoaEt71Us653gLrLsfMiVyg?e=uthb0H) |   Outdoor   | Fast, with yaw | Rectangle   |       3547.2       |        113.21         |        ✔        |         ✔          |  ✔   |                  |                  | [outdoor_rect_fast_3547.2g_113.21s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ER751r6eJoFAj7Pcb7QMb50BoaEt71Us653gLrLsfMiVyg?e=uthb0H) |
|  2   | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ETmoBKiEGVBNughR5VcALgsBEtorcFnPSgUFrhHlVQ3-Vg?e=5CdJNN) |   Outdoor   | Slow, with yaw | Rectangle   |       3547.2       |        175.38         |        ✔        |         ✔          |  ✔   |                  |                  | [outdoor_rect_slow_3547.2g_175.38s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ETmoBKiEGVBNughR5VcALgsBEtorcFnPSgUFrhHlVQ3-Vg?e=5CdJNN) |
|  3   | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EUI1UXS880RMvpNJ30VcGLoBdfBr5Nyrt8FOozJjz1mWMg?e=rAFFgj) |   Outdoor   | with yaw       | Round       |       3541.7       |        147.34         |        ✔        |         ✔          |  ✔   |                  |                  | [outdoor_round_yaw_3541.7g_147.34s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EUI1UXS880RMvpNJ30VcGLoBdfBr5Nyrt8FOozJjz1mWMg?e=rAFFgj) |
|  4   | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EUgGCSTKciVLmIxdYNOrg0kBXWhol9bbG_Brqt7Kd0gn6Q?e=TjgN5q) |   Outdoor   | without yaw    | Round       |       3541.7       |        105.85         |        ✔        |         ✔          |  ✔   |                  |                  | [outdoor_round_noyaw_3541.7g_105.85s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EUgGCSTKciVLmIxdYNOrg0kBXWhol9bbG_Brqt7Kd0gn6Q?e=TjgN5q) |
|  5   | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ESGZbmP13OFHh8EU84ev1Q8B7jh_IIy-dc4qzyKJ8n_DCw?e=LqxtPC) |   Outdoor   | with yaw       | 8-character |       3541.7       |        184.29         |        ✔        |         ✔          |  ✔   |                  |                  | [outdoor_8_yaw_3541.7g_184.29s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ESGZbmP13OFHh8EU84ev1Q8B7jh_IIy-dc4qzyKJ8n_DCw?e=LqxtPC) |
|  6   | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EfxXU01e6t1NsQAAVlKmMvcBcICoxKjdq0aWBq1NaMZfjg?e=FvItEH) |   Outdoor   | without yaw    | 8-character |       3541.4       |        243.73         |        ✔        |         ✔          |  ✔   |                  |                  | [outdoor_8_noyaw_3541.4g_243.73s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EfxXU01e6t1NsQAAVlKmMvcBcICoxKjdq0aWBq1NaMZfjg?e=FvItEH) |
|  7   | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EYyBEjZekZZEtSwzdD-ZfAYBlhmja_6u2BO6G8hWONn2BA?e=mHC3qS) |    Night    | Fast, with yaw | Rectangle   |       3460.0       |        133.58         |        ✔        |         ✔          |  ✔   |                  |                  | [night_rect_fast_3460.0g_133.58.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EYyBEjZekZZEtSwzdD-ZfAYBlhmja_6u2BO6G8hWONn2BA?e=mHC3qS) |
|  8   | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EWcPTXiYBDpOlNeTKQG3QG8ByfSpNxkuGjuQdwQeoN9dAA?e=DFopCY) |    Night    | Slow, with yaw | Rectangle   |       3541.6       |        179.48         |        ✔        |         ✔          |  ✔   |                  |                  | [night_rect_slow_3541.6g_179.48s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EWcPTXiYBDpOlNeTKQG3QG8ByfSpNxkuGjuQdwQeoN9dAA?e=DFopCY) |
|  9   | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EeTF-TTJ6K9NgP_Kx_zU5NMBjGGCb3UcYxhvVomYEQLe-g?e=uFjav2) |    Night    | without yaw    | Round       |       3541.6       |         82.07         |        ✔        |         ✔          |  ✔   |                  |                  | [night_round_no_yaw_3541.6g_82.07s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EeTF-TTJ6K9NgP_Kx_zU5NMBjGGCb3UcYxhvVomYEQLe-g?e=uFjav2) |
|  10  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EU3B3Z1nxqlJosNiN50NCkQBuG_jTKPpulZ0-8ZkG9ucWA?e=gDAEjU) |    Night    | without yaw    | 8-character |       3547.7       |        121.87         |        ✔        |         ✔          |  ✔   |                  |                  | [night_8_noyaw_3547.7g_121.87s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EU3B3Z1nxqlJosNiN50NCkQBuG_jTKPpulZ0-8ZkG9ucWA?e=gDAEjU) |
|  11  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EfxkbUiWvBhFosNp-MwaaJsBrpu_sqepv15Jdkt2pQ95Tg?e=BZUkTD) |   Indoor    | Loadless       | Hovor       |       3096.1       |         79.04         |        ✔        |         ✔          |      |        ✔         |                  | [indoor_loadless_hovor_3096.1g_79.04s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EfxkbUiWvBhFosNp-MwaaJsBrpu_sqepv15Jdkt2pQ95Tg?e=BZUkTD) |
|  12  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EVft7HlvXTREjufIB6HK5n4B8LnLcY51qgkadOw_P2qG8Q?e=Ywb81B) |   Indoor    | Loadless       | Round       |       3096.1       |        117.89         |        ✔        |         ✔          |      |        ✔         |                  | [indoor_loadless_round_3096.1g_117.89s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EVft7HlvXTREjufIB6HK5n4B8LnLcY51qgkadOw_P2qG8Q?e=Ywb81B) |
|  13  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ESlYQGYYPxJIs-_uE1BS8aQB_eCAWjCXLWeXA3NPPf4Azg?e=E3PUYd) |   Indoor    | Loadless       | 8-character |       3096.1       |        109.17         |        ✔        |         ✔          |      |        ✔         |                  | [indoor_loadless_8_3096.1g_109.17s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ESlYQGYYPxJIs-_uE1BS8aQB_eCAWjCXLWeXA3NPPf4Azg?e=E3PUYd) |
|  14  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EQLItrUwpf1CukOTbWS5L00BYRFRm6uRA12qVu_Ty40o6g?e=fiEYuh) |   Indoor    | Loaded         | Hovor       |       3372.2       |         80.11         |        ✔        |         ✔          |      |        ✔         |                  | [indoor_loaded_hovor_3103.2g_load269.0g_80.11s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EQLItrUwpf1CukOTbWS5L00BYRFRm6uRA12qVu_Ty40o6g?e=fiEYuh) |
|  15  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EZvbs8cvLktOjsT2tOVprZEBu_QrMAGu_SGpidG75d5fZQ?e=Yp702V) |   Indoor    | Loaded         | Round       |       3372.2       |        107.15         |        ✔        |         ✔          |      |        ✔         |                  | [indoor_loaded_round_3103.2g_load269.0g_107.15s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EZvbs8cvLktOjsT2tOVprZEBu_QrMAGu_SGpidG75d5fZQ?e=Yp702V) |
|  16  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ESlYQGYYPxJIs-_uE1BS8aQB_eCAWjCXLWeXA3NPPf4Azg?e=uGcdGK) |   Indoor    | Loaded         | 8-character |       3372.2       |        138.41         |        ✔        |         ✔          |      |        ✔         |                  | [indoor_loaded_8_3103.2g_load269.0g_138.41s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ERWQokKc-ItDqz05YZs2z7AB4qatTuipjKpie75wrlifYg?e=rSHHTf) |
|  17  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EcofuY536kVCgsEMPxV7b_ABPJcrfJWxJvc-hDVRK9QOxQ?e=AgIf0E) |   Indoor    | Rope pulled    | Random      |       3101.5       |        126.53         |        ✔        |         ✔          |      |        ✔         |        ✔         | [indoor_sensor_fast_3101.5g_126.53s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EcofuY536kVCgsEMPxV7b_ABPJcrfJWxJvc-hDVRK9QOxQ?e=AgIf0E) |
|  18  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ERYdQouQAuBMjyYsQUWztYQBU70M3_PQggBb3MrpnFrtuA?e=t8ernO) |   Indoor    | Rope pulled    | Random      |       3101.5       |        155.38         |        ✔        |         ✔          |      |        ✔         |        ✔         | [indoor_sensor_slow_3101.5g_155.38s.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/ERYdQouQAuBMjyYsQUWztYQBU70M3_PQggBb3MrpnFrtuA?e=t8ernO) |
|  19  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EabZZ1i9XRVDpeKLCK1ywvMBwQS7UcNLidJ-NBYbDIli-Q?e=snJxAn) | Motor test  |                |             |                    |         45.04         |                 |         ✔          |      |                  |        ✔         | [channel_1.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EabZZ1i9XRVDpeKLCK1ywvMBwQS7UcNLidJ-NBYbDIli-Q?e=snJxAn) |
|  20  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EbIqKs7QDM1MuTmzxjZDKSABa0EEQ6h3_bT1OJizo9FnrQ?e=7qaL2K) | Motor test  |                |             |                    |         45.08         |                 |         ✔          |      |                  |        ✔         | [channel_2.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EbIqKs7QDM1MuTmzxjZDKSABa0EEQ6h3_bT1OJizo9FnrQ?e=7qaL2K) |
|  21  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EbeFrfzSubxDiMA1QqQ7iO0BioSC1kkDgpll6CrQoM_NOA?e=eZlUjn) | Motor test  |                |             |                    |         43.00         |                 |         ✔          |      |                  |        ✔         | [channel_3.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EbeFrfzSubxDiMA1QqQ7iO0BioSC1kkDgpll6CrQoM_NOA?e=eZlUjn) |
|  22  | [Download](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EYCLRd5DbUFGlfcGiZT1PtYB6RSK13laHEmSsfwISv-2cg?e=lAZ6lz) | Motor test  |                |             |                    |         43.84         |                 |         ✔          |      |                  |        ✔         | [channel_4.bag](https://zjufast-my.sharepoint.com/:u:/g/personal/tkyang_zjufast_onmicrosoft_com/EYCLRd5DbUFGlfcGiZT1PtYB6RSK13laHEmSsfwISv-2cg?e=lAZ6lz) |





