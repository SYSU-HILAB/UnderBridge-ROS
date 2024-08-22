# UnderBridge-ROS
桥下（定位导航失效环境）前期验证代码库，开发环境为ROS，平台Linux

## 描述： 
 v0.2.1

 主要的代码换成了elastic-planner，wind的部分不变，不适用ego

 2024-8-20 11：30 

## ADD：
 - 更换为elastic-planner
 - 增加了大桥的点云模型，替换原本的地图，能够正常运行（/mockamap/maps.cpp & mocakmap.cpp）
 - - 大桥点云是直接给出全局点云，整个桥梁一次性生成

## TODO：
 - launch中用xml读取大桥的生成等参数
 - 接入风机的mapping和地图接口，做到获取每一帧的点云实时处理

## TEST：

## FIX ME：

## SUPPLY：补充说明
- 使用方法：
 - 1. 编译之后source，如果想要全局地图，在mockamap.cpp中把type修改为3即可
 - 2. roslaunch mapping rviz_sim.launch
 - 3. roslaunch planning fake_target.launch 

- 备注：
 - 1. 目前仿真仅有单机，暂时不需要启动simulation1.launch和pub_triger.sh（后续接入可能需要给目标点，继续研究）
 - 2. 原仓库地址：<https://github.com/ZJU-FAST-Lab/Elastic-Tracker?tab=readme-ov-file>

- 详情见我来