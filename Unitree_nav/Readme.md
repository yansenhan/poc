启动unitree巡检火源的流程

## 1. PC端
注意.bashrc中HOST等环境变量的IP在31网段下：
    export ROS_MASTER_URI=http://192.168.31.110:11311
    export ROS_HOSTNAME=192.168.31.110
- 启动 ROS核
    roscore



## 2. Unitree 端
- 进入运动控制模式：(L2+A)*3 + (L1+B) + (L1+START)
- 启动 雷达转发(终端保持开启)
    socat -d -d tcp-listen:1445,bind=0.0.0.0,reuseaddr,fork tcp:192.168.11.1:1445

- 启动 视频流捕获
    <!-- python ~/Thu_unitree/ucar_nav2/src/ucar_cam/usb_2_cam.py -->
    /usr/bin/python /home/unitree/Thu_unitree/ucar_test2/src/ucar_cam/scripts/usb_2_cam.py


## 3. PC端
- 预热动作模块
    roslaunch unitree_legged_real real.launch 

- 启动 雷达节点，Movebase，ros_udp
    roslaunch /home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/launch/unitree_navigation2.launch

    rosrun ucar_nav transform 

- 启动 high_cmd 转译
    conda activate noetic
    python ~/Thu_unitree/Unitree_nav/src/ucar_nav/script/control.py

- 启动 yolo
    conda activate noetic
    python ~/Thu_unitree/Unitree_nav/src/ucar_nav/script/ros_yolo.py

- 启动 决策主程序
    conda activate noetic
    python ~/Thu_unitree/Unitree_nav/src/ucar_nav/script/example_zyp.py



## 调试工具

rqt_graph 观察信号流图

rostopic list  查看消息集合

rostopic echo /high_cmd  查看控制是否发布

rqt 查看 tf tree

ps -ef | grep socat  查看转发是否成功




