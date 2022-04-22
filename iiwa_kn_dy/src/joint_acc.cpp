#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/JointState.h>
#include "iiwa_kdl/kdl_kine_solver.h"
#include "iiwa14Kine/iiwa14Kine.h"

#include "boost/foreach.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;


int main (int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_fk_cw3");
    bool use_own_dynamics = true;
    robot_kinematic h_kine;
    ros::Rate loop_rate(10);

    std::string base_link_name ="iiwa_link_0";
    std::string ee_link_name = "iiwa_link_ee";

    h_kine.init(base_link_name, ee_link_name);
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/iiwa/EffortJointInterface_trajectory_controller/command", 5);
    rosbag::Bag bag;
    bag.open("/home/cristobal/catkin_ws/src/comp0127_lab/cw3/cw3_launch/bags/cw3bag1.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/iiwa/EffortJointInterface_trajectory_controller/command"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    sleep(5);
    VectorXd finalPos(7);
    BOOST_FOREACH (rosbag::MessageInstance const m, view)
    {
        trajectory_msgs::JointTrajectory::ConstPtr J = m.instantiate<trajectory_msgs::JointTrajectory>();
        if (J != NULL)
        {
            if (J->points.size() != 0)
            {
                for (int i = 1; i < 7; i++) {
                    finalPos(i) = J->points.at(2).positions.at(i);
                }
                traj_pub.publish(J);
            }
        }
    }
    ofstream acc_file, pos_file, vel_file, time_file;
    acc_file.open("acc.txt");
    pos_file.open("pos.txt");
    vel_file.open("vel.txt");
    time_file.open("timestamp.txt");

    iiwa14_kinematic iiwa14;
    Matrix4d pose;
    iiwa14.init();
    while (ros::ok())
    {
        if (h_kine.msg_received)
        {
            MatrixXd angles_vel = VectorXd::Zero(7, 1);
            VectorXd angles = VectorXd::Zero(7, 1);
            angles << h_kine.current_joint_position.data(0), h_kine.current_joint_position.data(1),
                 h_kine.current_joint_position.data(2), h_kine.current_joint_position.data(3),
                h_kine.current_joint_position.data(4), h_kine.current_joint_position.data(5),
                h_kine.current_joint_position.data(6);
            angles_vel << h_kine.current_joint_velocity.data(0), h_kine.current_joint_velocity.data(1),
                h_kine.current_joint_velocity.data(2), h_kine.current_joint_velocity.data(3),
                h_kine.current_joint_velocity.data(4), h_kine.current_joint_velocity.data(5),
                h_kine.current_joint_velocity.data(6);
            MatrixXd angles_effort(7, 1);
            angles_effort << h_kine.current_joint_effort.data(0), h_kine.current_joint_effort.data(1),
                h_kine.current_joint_effort.data(2), h_kine.current_joint_effort.data(3),
                h_kine.current_joint_effort.data(4), h_kine.current_joint_effort.data(5),
                h_kine.current_joint_effort.data(6);
            MatrixXd angles_acc = VectorXd::Zero(7, 1);
            MatrixXd B;
            MatrixXd C;
            MatrixXd G;
            if (use_own_dynamics)
            {
                VectorXd angles_vel_2 = angles_vel;
                B = iiwa14.getB(angles);
                C = iiwa14.getC(angles, angles_vel_2) * angles_vel;
                G = iiwa14.getG(angles);
            }
            else
            {
                B = h_kine.getB(h_kine.current_joint_position);
                C = h_kine.getC(h_kine.current_joint_position, h_kine.current_joint_velocity);
                G = h_kine.getG(h_kine.current_joint_position);
            }
            angles_acc = B.inverse() * (angles_effort - C - G);
            pos_file << angles.transpose() << std::endl;
            acc_file << angles_acc.transpose() << std::endl;
            vel_file << angles_vel.transpose() << std::endl;
            time_file << h_kine.stamp << std::endl;
            h_kine.msg_received = false;
            if ((finalPos - angles).norm() < 0.7) 
            {
                break;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    acc_file.close();
    vel_file.close();
    pos_file.close();
    bag.close();
    return 123;
}
