import os
import time


def open_terminal(commands):
    '''
    打开一个新的终端并执行命令
    未作参数检查，不要试图在传入的命令字符串中添加多余的引号，可能会引发错误
    '''
    cmd_list = []
    for cmd in commands:
        cmd_list.append(""" gnome-terminal --tab -e "bash -c '%s;exec bash'" >/dev/null  2>&1 """ %cmd)

    os.system(';'.join(cmd_list))


def launch_pkg():
    '''
    启动必要的程序
    '''
    print('即将打开导航文件')
    nav_cmd = [
        'roscore',
        'sleep 3; /home/lijixiang/miniconda3/envs/noetic/bin/python /home/lijixiang/Thu_unitree/Unitree_nav/src/ucar_nav/script/State2odom.py', 
        'sleep 3; roslaunch ucar_nav unitree_navigation.launch'
    ]
    open_terminal(nav_cmd)

    print('正在启动，请稍等...')
    # open_terminal(cmd)
    time.sleep(10)


def main():
    launch_pkg()   


if __name__  == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\node')
        print('操作已取消')
        exit(0)


        # // "x": -0.2504603767395,
        # // "y": -5.2709980011,
        # // "z": 0.0