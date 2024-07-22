#include<ros/ros.h>
#include<tf/tf.h> 
#include<tf/transform_listener.h>
#include<std_msgs/String.h>
#include<std_msgs/Float32MultiArray.h>


int main(int argc, char* argv[]) 
{
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle n;
    ros::Publisher posemsgpub = n.advertise<std_msgs::Float32MultiArray>("/posemsg", 10); 
    ros::Rate r(10.0);
    tf::TransformListener listener;
    double yaw,pitch,roll; 

    while (ros::ok())
    {
        tf::StampedTransform transform;

        try{
            listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("map", "base_link",
                                ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        transform.getBasis().getEulerYPR(yaw, pitch, roll);
        std_msgs::Float32MultiArray msg;
        msg.data.resize(3);
        msg.data[0]=transform.getOrigin().x();
        msg.data[1]=transform.getOrigin().y();
        msg.data[2]=yaw;
        posemsgpub.publish(msg);
        r.sleep();
    }
    return 0;
}


