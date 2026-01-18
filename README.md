# 2D Lidar Simulation in Gazebo 11

this is tutorial how to simulating 2D lidaro in Gazebo <mark>Remember this is only for simulation purpose, if u want to try in the real drone make sure the data is already correct</mark> 

## Step - 1 : Add lidar into your drone, (2D)

Edit file `models/iris_with_ardupilot/model.sdf`: <br> copy paste this plugin, copy paste this plugin into ur iris_with_ardupilot drone model.sdf under <mark> joint name="iris_gimbal_mount" type="revolute" </mark>

### 2D Lidar

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



this is 2D lidar visualization must be
<img alt="2D" src="img/2D.png">



### 3D Lidar 
```xml
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
            <vertical>
              <samples>16</samples>
              <resolution>1</resolution>
              <min_angle>-0.2617</min_angle>
              <max_angle>0.2617</max_angle>
            </vertical>
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
          <output_type>sensor_msgs/PointCloud2</output_type>
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
- `libgazebo_ros_laser.so` (ROS1 only)
- Parameter `<topicName>` dan `<frameName>` (deprecated)

**Latest Plugin ROS2**
- `libgazebo_ros_ray_sensor.so` (ROS2)
- Parameter `<remapping>~/out:=scan</remapping>` 

# LIVOX Mid-360 Lidar Simulation using Iris Drone Models



**Author:** El Jausyan ~ vtol-Soeromiber  
**Date:** 2026-01-09  
**Status:** TESTED & WORKING
