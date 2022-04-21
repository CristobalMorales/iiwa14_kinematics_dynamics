#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/JointState.h>
#include <iiwa_kn_dy/kdl_kine_solver.h>
#include <iiwa14Kine/iiwa14Kine.h>

#include "boost/foreach.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include <math.h>
#include <stdio.h>
#include <ros/console.h>
 
using namespace Eigen;
using namespace std;
int main (int argc, char **argv)
{
    ros::init(argc, argv, "iiwa_mass_cw3");
    bool use_own_dynamics = true;
    robot_kinematic h_kine;
    ros::Rate loop_rate(10);
    bool calculate_force = true;

    std::string base_link_name ="object_iiwa_link_0";
    std::string ee_link_name = "object_iiwa_link_ee";

    h_kine.init(base_link_name, ee_link_name);

    trajectory_msgs::JointTrajectory my_traj;
    trajectory_msgs::JointTrajectoryPoint my_pt;
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/object_iiwa/EffortJointInterface_trajectory_controller/command", 5);

    my_traj.header.stamp = ros::Time::now();
    my_traj.joint_names.push_back("object_iiwa_joint_1");
    my_traj.joint_names.push_back("object_iiwa_joint_2");
    my_traj.joint_names.push_back("object_iiwa_joint_3");
    my_traj.joint_names.push_back("object_iiwa_joint_4");
    my_traj.joint_names.push_back("object_iiwa_joint_5");
    my_traj.joint_names.push_back("object_iiwa_joint_6");
    my_traj.joint_names.push_back("object_iiwa_joint_7");

    my_pt.positions.resize(7);
    my_pt.velocities.resize(7);
    my_pt.accelerations.resize(7);

    VectorXd mass(6);
    mass << 4, 4, 3, 2.7, 1.7, 1.8;//, 0.3;

    sleep(5);
    int n_data = 3;
    MatrixXd joints(n_data, 7);
    if (calculate_force)
    {
        joints << 45, -35, 55, -30, -25, 65, -10,
            5, 40, 80, 10, 8, -50, -10,
            -50, 60, 0, -15, 60, -25, -50;
    }
    else
    {
        n_data = 1;
        joints << 0, -90, -90, 90, -90, 90, -30,
        0, -90, -90, 90, -90, 90, -30,
        0, -90, -90, 90, -90, 90, -30;
    }
    for (int j = 0; j < n_data; j++)
    {
        my_pt.time_from_start.sec = 30 * (j + 1);
        for (int pos = 0; pos < 7; pos++)
        {
            my_pt.positions.at(pos) = joints(j, pos) * M_PI / 180;
            my_pt.accelerations.at(pos) = 0;
            my_pt.velocities.at(pos) = 0;
        }
        my_traj.points.push_back(my_pt);
    }
    traj_pub.publish(my_traj);

    iiwa14_kinematic iiwa14;
    Matrix4d pose;
    iiwa14.init();
    double mass_average = 0;
    VectorXd joint = joints.block(0, 0, 1, 7).transpose();
    int i = 0;
    while (ros::ok())
    {
        if (h_kine.msg_received)
        {
            VectorXd angles(7, 1);
            angles << h_kine.current_joint_position.data(0), h_kine.current_joint_position.data(1),
                h_kine.current_joint_position.data(2), h_kine.current_joint_position.data(3),
                h_kine.current_joint_position.data(4), h_kine.current_joint_position.data(5),
                h_kine.current_joint_position.data(6);
            if ((angles - (joint * M_PI / 180)).norm() > 0.05) {
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }
            MatrixXd angles_effort(7, 1);
            angles_effort << h_kine.current_joint_effort.data(0), h_kine.current_joint_effort.data(1),
                h_kine.current_joint_effort.data(2), h_kine.current_joint_effort.data(3),
                h_kine.current_joint_effort.data(4), h_kine.current_joint_effort.data(5),
                h_kine.current_joint_effort.data(6);
            MatrixXd G_M = iiwa14.getG(angles);
            MatrixXd Jaco = iiwa14.get_jacobian(angles);
            MatrixXd B = iiwa14.getB(angles);
            if (calculate_force) {
                MatrixXd g = (Jaco * B.inverse() * Jaco.transpose()).inverse();
                MatrixXd f_hard = g * (Jaco * B.inverse() * (angles_effort - G_M));
                Vector3d force = f_hard.block(0, 0, 3, 1);
                double me = force.norm() / 9.8;
                ROS_INFO("Mass: %f", me);
                mass_average = mass_average + me;
                if (i == 2)
                {
                    break;
                }
                i = i + 1;
                joint = joints.block(i, 0, 1, 7).transpose();
            }
            else
            {
                MatrixXd t_ext = G_M - angles_effort;
                std::cout << t_ext << std::endl;
                break;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Mass Average: %f", mass_average/3);
    return 123;
}
