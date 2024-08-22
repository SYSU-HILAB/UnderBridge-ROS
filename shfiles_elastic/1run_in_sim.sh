cd /home/n21
source .bashrc
cd /home/n21/UnderBridge-ROS
chmod +x shfiles_elastic/pub_triger.sh
echo "ready"
roslaunch mapping run.launch & sleep 5;
echo "start mappping"
roslaunch planning fake_target.launch & sleep 5;
echo "target"
roslaunch planning simulation1.launch & sleep 3;
echo "simulation"
bash /home/n21/UnderBridge-ROS/shfiles_elastic/pub_triger.sh






