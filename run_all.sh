gnome-terminal -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0.25 0 0 0 iris::base_link livox_mid360_sensor; exec bash"
sleep 2
timeout 3 ros2 run tf2_ros tf2_echo iris::base_link livox_mid360_sensor 2>&1 | grep -A 2 "Translation" || echo "Waiting for TF..."
sleep 1

if [ -f "livox_view.rviz" ]; then
    rviz2 -d livox_view.rviz
else
    rviz2
fi


