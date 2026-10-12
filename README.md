# Fast-LIO2 Localization with Livox Mid-360

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Dự án này là phiên bản tùy chỉnh và nâng cấp của Fast-LIO2, tích hợp thêm tính năng định vị lại bằng ICP (ICP Relocalization) để robot có thể định vị dựa trên bản đồ sẵn có (prior map) với cảm biến LiDAR Livox Mid-360.

## Giới thiệu

![](./reloc.gif)

- Màu xanh: bản đồ đã có sẵn (`prior map`)
- Màu trắng: dữ liệu quét hiện tại từ LiDAR
- Khi các điểm quét hiện tại ổn định và khớp với bản đồ, quá trình ICP thành công và hệ thống chuyển sang chế độ định vị lại (`relocalization mode`)

## Cấu trúc thư mục

```text
livox_ws/
└── src/
    ├── FAST_LIO/             # hoặc fast_lio2
    │   ├── config/
    │   ├── include/
    │   ├── launch/
    │   ├── rviz/
    │   └── src/
    ├── icp_relocalization/
    │   ├── launch/
    │   └── src/
    ├── livox_ros_driver2/
    │   ├── 3rdparty/
    │   ├── config/
    │   ├── launch_ROS2/
    │   └── src/
    └── odom_subscriber/
        ├── odom_subscriber/
        └── resource/
```

## Yêu cầu hệ thống

- Ubuntu 22.04
- ROS 2 Humble
- LiDAR Livox Mid-360
- PCL
- Eigen3
- Livox SDK2

## Cài đặt

### 1. Tạo workspace

```bash
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
```

### 2. Cài đặt Livox SDK2 và driver ROS 2

- Cài đặt Livox SDK2 theo hướng dẫn chính thức của Livox
- Clone driver ROS 2:

```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

### 3. Thêm mã nguồn dự án vào workspace

Copy các thư mục như `FAST_LIO`, `icp_relocalization`, `odom_subscriber` vào `~/livox_ws/src/`.

### 4. Build workspace

```bash
cd ~/livox_ws
colcon build --symlink-install
source install/setup.bash
```

## Hướng dẫn sử dụng

### 1. Xây dựng bản đồ (Mapping)

```bash
ros2 launch fast_lio mapping.launch.py
```

### 2. Định vị lại (Relocalization)

Chạy launch file mẫu trong gói `icp_relocalization`:

```bash
ros2 launch icp_relocalization example.launch.py
```

Tệp `example.launch.py` sẽ khởi chạy node ICP cùng với file cấu hình định vị.

## Lưu ý quan trọng

- `map_path` trong `example.launch.py` và `prior_map_path` trong `FAST_LIO/config/fast_lio_relocalization_param.yaml` phải trỏ đến cùng một file `.pcd`
- Cần cung cấp `initial pose` ban đầu cho robot trước khi relocalization
- Có thể thiết lập trực tiếp các tham số như `initial_x`, `initial_y`, ... trong file launch, hoặc dùng công cụ `2D Pose Estimation` trong RViz

## Tóm tắt

Dự án này giúp tích hợp nhanh quy trình xây dựng bản đồ và định vị lại chính xác trên hệ thống ROS 2 với cảm biến Livox Mid-360, phù hợp cho ứng dụng robot di chuyển trong môi trường đã biết trước.