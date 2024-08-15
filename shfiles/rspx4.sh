sudo cpufreq-set -g performance;

echo " " | sudo chmod 777 /dev/ttyUSB0 & sleep 1;
roslaunch mavros px4.launch & sleep 3;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;

roslaunch realsense2_camera rs_camera.launch & sleep 3;

roslaunch vins fast_drone_250.launch
wait;
