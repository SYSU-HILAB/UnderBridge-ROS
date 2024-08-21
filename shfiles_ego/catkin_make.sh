# 第一次编译需要把vins编译了，后面不用编译，使用这个sh即可
catkin_make -DCATKIN_BLACKLIST_PACKAGES="camera_models global_fusion loop_fusion vins"