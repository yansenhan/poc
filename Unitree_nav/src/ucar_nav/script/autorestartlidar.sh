while : ; do date ; sleep 8 ; pkill -KILL socat ; sleep 6; grep transfer soclidar.log|| ( pkill -KILL socat ; sleep 8; grep transfer soclidar.log || pkill -KILL socat ) ;  pgrep -af socat ; done

while : ; do date ; socat -d -d tcp-listen:1445,bind=0.0.0.0,reuseaddr,fork tcp:192.168.11.1:1445 2> soclidar.log; done

while : ; do date ; /home/lijixiang/miniconda3/envs/noetic/bin/python /home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/script/ros_yolo.py 2> yolo.log; done
