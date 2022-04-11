#include <iostream>
#include <cstdlib>

#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


class Robot{
protected:

    ros::NodeHandle nh;

    ros::Subscriber sub_front;
    ros::Subscriber sub_left;
    ros::Subscriber sub_right;
    ros::Subscriber sub_pose;

    sensor_msgs::Range front_msg;
    sensor_msgs::Range left_msg;
    sensor_msgs::Range right_msg;
    nav_msgs::Odometry odom_msg;

    ros::Publisher pub_front;
    ros::Publisher pub_left;
    ros::Publisher pub_right;
    ros::Publisher pub_odom;

    tf::TransformBroadcaster odom_broadcaster;

    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;

public:
    Robot(){
    
        sub_front = nh.subscribe("front_distance", 1000, &Robot::front_Cb, this );
        sub_left = nh.subscribe("left_distance", 1000, &Robot::left_Cb, this );
        sub_right = nh.subscribe("right_distance", 1000, &Robot::right_Cb, this );
        sub_pose = nh.subscribe("pose", 1000, &Robot::pose_Cb, this );

        front_msg.header.frame_id = "front_ir";
        front_msg.radiation_type = 1;                     
        front_msg.field_of_view = 0.034906585;
        front_msg.min_range = 0.1;
        front_msg.max_range = 0.8;

        left_msg.header.frame_id = "left_ir";
        left_msg.radiation_type = 1;                     
        left_msg.field_of_view = 0.034906585;
        left_msg.min_range = 0.1;
        left_msg.max_range = 0.8;

        right_msg.header.frame_id = "right_ir";
        right_msg.radiation_type = 1;                     
        right_msg.field_of_view = 0.034906585;
        right_msg.min_range = 0.1;
        right_msg.max_range = 0.8;

        odom_msg.header.frame_id = "odom";

        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        pub_front =  nh.advertise<sensor_msgs::Range>("ir_front_sensor", 1000);
        pub_left =  nh.advertise<sensor_msgs::Range>("ir_left_sensor", 1000);
        pub_right =  nh.advertise<sensor_msgs::Range>("ir_right_sensor", 1000);
        pub_odom =  nh.advertise<nav_msgs::Odometry>("odom", 1000);

    }

    ~Robot(void)
    {
    }

    void front_Cb(const std_msgs::Float32::ConstPtr& front){

        front_msg.header.stamp = ros::Time::now();
        front_msg.range = front->data;
        pub_front.publish(front_msg);
    }

    void left_Cb(const std_msgs::Float32::ConstPtr& left){

        left_msg.header.stamp = ros::Time::now();
        left_msg.range =left->data;
        pub_left.publish(left_msg);
    }
    
    void right_Cb(const std_msgs::Float32::ConstPtr& right){

        right_msg.header.stamp = ros::Time::now();
        right_msg.range = right->data;
        pub_right.publish(right_msg);
    }    

    void pose_Cb(const geometry_msgs::Pose2D::ConstPtr& pose){

        odom_msg.header.stamp = ros::Time::now();
        odom_trans.header.stamp = ros::Time::now();

        odom_quat = tf::createQuaternionMsgFromYaw(pose -> theta);

        odom_msg.pose.pose.position.x = pose -> x;
        odom_msg.pose.pose.position.y = pose  -> y;
        odom_msg.pose.pose.orientation = odom_quat;

        odom_trans.transform.translation.x = pose -> x;
        odom_trans.transform.translation.y = pose  -> y;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);
        pub_odom.publish(odom_msg);
    }

    void run(){

        ros::Rate loop_rate(10);
        while (ros::ok())
        {   
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

};



int main(int argc, char **argv){

    ros::init(argc, argv, "robot_driver_node");
    ros::NodeHandle n;

    Robot rob1;
    rob1.run();

    return 0;
}
