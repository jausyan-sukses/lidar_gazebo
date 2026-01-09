# 2D Lidar Simulation in Gazebo 11

this is tutorial how to simulating 2D lidaro in Gazebo <mark>Remember this is only for simulation purpose, if u want to try in the real drone make sure the data is already correct</mark> 

## Step - 1 : Add lidar into your drone

Edit file `models/iris_with_ardupilot/model.sdf`: <br> copy paste this plugin

```xml
<sensor name="laser" type="ray">
  <pose>0 0 0 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.12</min>
      <max>12.0</max>
      <resolution>0.015</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
  
  <!-- PLUGIN ROS2 -->
  <plugin name="laser_plugin" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
  
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Atau ini
```xml
     <!-- 2D Lidar sensor -->
    <link name="lidar_link">
      <pose>0 0 0.25 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.000001</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.000001</iyy>
          <iyz>0.0</iyz>  
          <izz>0.000001</izz>
        </inertia>
      </inertial>
      <collision name="lidar_collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.07</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="lidar_visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.07</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
      <sensor name="laser" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.12</min>
            <max>12.0</max>
            <resolution>0.015</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="laser_plugin" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <namespace>/</namespace>
            <remapping>~/out:=scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>lidar_link</frame_name>
        </plugin>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <visualize>true</visualize>
      </sensor>
    </link>

    <joint name="lidar_joint" type="fixed">
      <parent>iris::base_link</parent>
      <child>lidar_link</child>
    </joint>
```

**Old Plugin ROS1**
- `libgazebo_ros_laser.so` ❌ (ROS1 only)
- Parameter `<topicName>` dan `<frameName>` ❌ (deprecated)

**Latest Plugin ROS2**
- `libgazebo_ros_ray_sensor.so` ✅ (ROS2)
- Parameter `<remapping>~/out:=scan</remapping>` ✅

---

## How to Run

### 1. Start Gazebo with ROS2 Plugin

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Set plugin path
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH

# Start Gazebo
cd /home/el-system/ardupilot_gazebo
gazebo --verbose worlds/arena_safmc.world
```

### 2. Publish TF Transform 

```bash
source /opt/ros/humble/setup.bash

# Publish transform dari base_link ke lidar_link
ros2 run tf2_ros static_transform_publisher \
  --frame-id iris_demo::base_link \
  --child-frame-id iris_demo::lidar_link \
  --x 0 --y 0 --z 0.25
```

```bash
ros2 run tf2_ros static_transform_publisher --frame-id iris_demo::base_link --child-frame-id iris_demo::lidar_link --x 0 --y 0 --z 0.25
```

### 3. Check Topic /scan 

```bash
source /opt/ros/humble/setup.bash

# List topics
ros2 topic list | grep scan

# Info topic
ros2 topic info /scan

# Echo data
ros2 topic echo /scan
```

### 4. Rviz2 Time

```bash
source /opt/ros/humble/setup.bash
rviz2
```

**Setting di RViz2:**
1. **Fixed Frame**: `iris_demo::base_link`
2. **Add** → **LaserScan**
3. **Topic**: `/scan`
4. **Reliability Policy**: `Best Effort` (jika perlu)

---

## 🔧 Troubleshooting

### /scan topic does'nt appears

**Cause:**
- GAZEBO_PLUGIN_PATH tidak di-set
- Gazebo running sebelum model.sdf diperbaiki

**Solusi:**
```bash
# Stop Gazebo (Ctrl+C)
# Restart dengan plugin path yang benar
source /opt/ros/humble/setup.bash
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
gazebo worlds/arena_safmc.world
```

### RViz2 tidak menampilkan lidar

**Penyebab:**
- TF transform tidak ada
- Fixed Frame salah

**Solusi:**
1. Pastikan TF publisher jalan (lihat langkah 2)
2. Fixed Frame harus: `iris_demo::base_link` (bukan `base_link`)
3. Check frame_id dari topic:
```bash
ros2 topic echo /scan --once | grep frame_id
```

### Data lidar semua 0.0 atau inf

**Penyebab:**
- Lidar terlalu dekat dengan ground
- Tidak ada obstacle di sekitar

**Solusi:**
- Tambahkan obstacle di Gazebo
- Cek visualize lidar di Gazebo (ray beams hijau)

### Error: libgazebo_ros_ray_sensor.so not found

**Penyebab:**
- ROS2 gazebo_plugins tidak terinstall

**Solusi:**
```bash
sudo apt install ros-humble-gazebo-plugins
```

---

## Checklist Cepat

- [ ] Model.sdf sudah pakai `libgazebo_ros_ray_sensor.so`
- [ ] Source ROS2: `source /opt/ros/humble/setup.bash`
- [ ] Set plugin path: `export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH`
- [ ] Gazebo running
- [ ] TF publisher running
- [ ] Topic `/scan` muncul: `ros2 topic list | grep scan`
- [ ] Fixed Frame di RViz2: `iris_demo::base_link`

---

## Summary

**Keuntungan Direct ROS2 Plugin:**
- Tidak perlu Python bridge
-  Lebih efisien (native Gazebo plugin)
-  Latency lebih rendah
-  Lebih stabil

**Yang Dibutuhkan:**
1. Plugin ROS2 yang benar di model.sdf
2. GAZEBO_PLUGIN_PATH di-set
3. TF publisher untuk visualisasi RViz2

model.sdf in this repository is for iris with ardupilot drone model

---

**Author:** El Jausyan ~ vtol-Soeromiber  
**Date:** 2026-01-09  
**Status:** TESTED & WORKING
