#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <fstream>

// define pi
#define PI 3.14159265
#define THRESHOLD 0.001

/* 
Node to publish velocity command to the topic /robot_base_velocity_controller/cmd_vel
*/
geometry_msgs::PoseStamped current_pose;
geometry_msgs::Pose2D goal_pose;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose.pose.position.x = msg->pose.pose.position.x;
    current_pose.pose.position.y = msg->pose.pose.position.y;
    current_pose.pose.position.z = msg->pose.pose.position.z;
    current_pose.pose.orientation.x = msg->pose.pose.orientation.x;
    current_pose.pose.orientation.y = msg->pose.pose.orientation.y;
    current_pose.pose.orientation.z = msg->pose.pose.orientation.z;
    current_pose.pose.orientation.w = msg->pose.pose.orientation.w;
}

bool equalPoses(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::Pose2D& pose2)
{
    if (abs(pose1.pose.position.x - pose2.x) < 0.1 && 
        abs(pose1.pose.position.y - pose2.y) < 0.1)
    {
        // print the differences
        return true;
    }
    else
    {
        return false;
    }
}

// angle wrap function to keep angle between -pi and pi
double angleWrap(double angle)
{
    if (angle > PI)
        angle = angle - 2*PI;
    else if (angle < -PI)
        angle = angle + 2*PI;
    return angle;
}

struct poseXYT
{
    double x;
    double y;
    double theta;
};

void parse2DAstarPlan(std::vector<geometry_msgs::Pose2D>& plan, std::string filename)
{
    std::ifstream infile(filename);
    std::string line;
    std::cout << "Parsing plan from file: " << filename << std::endl;
    std::vector<geometry_msgs::Pose2D> plan2D;
    while (std::getline(infile, line))
    {
        std::stringstream ss(line);
        geometry_msgs::Pose2D pose;
        std::string xs, ys;
        std::getline(ss, xs, ',');
        std::getline(ss, ys);
        pose.x = std::stod(xs);
        pose.y = std::stod(ys);
        std::cout << pose.x << " " << pose.y << std::endl;

        pose.x = 0.5*(pose.x - 10);
        pose.y = -0.5*(pose.y - 10);
        plan2D.push_back(pose);
    }
    std::cout << "Plan without theta: " << std::endl;
    for (int i = 0; i < plan2D.size(); i++)
    {
        std::cout << plan2D[i].x << " " << plan2D[i].y << std::endl;
    }
    std::vector<geometry_msgs::Pose2D> planWithTheta;
    // convert 2D plan to plan with heading angle
    double lastTheta = 0.0;
    for (int i = 0; i < plan2D.size()-1; i++)
    {
        geometry_msgs::Pose2D pose;
        pose.x = plan2D[i].x;
        pose.y = plan2D[i].y;
        pose.theta = 0.0;

        double dx = plan2D[i+1].x - plan2D[i].x;
        double dy = plan2D[i+1].y - plan2D[i].y;

        if (dx == 0 && dy != 0)
        {
            if (dy > 0)
                pose.theta = PI/4;
            else if (dy < 0)
                pose.theta = -PI/2;
        }
        else if (dx != 0 && dy == 0)
        {
            if (dx > 0)
                pose.theta = 0;
        }
        else if (dx != 0 && dy != 0)
        {
            pose.theta = atan(dy/dx);
        }
        planWithTheta.push_back(pose);
        lastTheta = pose.theta;
    }
    // add the last pose
    geometry_msgs::Pose2D pose;
    pose.x = plan2D[plan2D.size()-1].x;
    pose.y = plan2D[plan2D.size()-1].y;
    pose.theta = lastTheta;
    planWithTheta.push_back(pose);

    plan = planWithTheta;
    // reverse the plan
    // std::reverse(plan.begin(), plan.end());
    // print the plan
    std::cout << "Plan with heading angle: " << std::endl;
    for (int i = 0; i < plan.size(); i++)
    {
        std::cout << plan[i].x << " " << plan[i].y << " " << plan[i].theta << std::endl;
    }
}

void parse3DAstarPlan(std::vector<geometry_msgs::Pose2D>& plan, std::string filename)
{
    std::ifstream infile(filename);
    std::string line;
    std::cout << "Parsing plan from file: " << filename << std::endl;
    while (std::getline(infile, line))
    {
        std::stringstream ss(line);
        geometry_msgs::Pose2D pose;
        std::string xs, ys, thetas;
        std::getline(ss, xs, ',');
        std::getline(ss, ys, ',');
        std::getline(ss, thetas);

        pose.x = std::stod(xs);
        pose.y = std::stod(ys);
        pose.theta = std::stod(thetas);
        std::cout << pose.x << " " << pose.y << " " << pose.theta << std::endl;

        pose.x = 0.5*(pose.x - 10);
        pose.y = -0.5*(pose.y - 10);
        // convert to radians
        pose.theta = pose.theta*PI/180;

        plan.push_back(pose);
    }

    std::cout << "Plan with heading angle: " << std::endl;
    for (int i = 0; i < plan.size(); i++)
    {
        std::cout << plan[i].x << " " << plan[i].y << " " << plan[i].theta << std::endl;
    }
}

void wrapAngle(double& angle)
{
    if (angle > PI)
        angle = angle - 2*PI;
    else if (angle < -PI)
        angle = angle + 2*PI;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planToVel");
    // file path is the first argument
    std::string filePath = argv[1];
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/robot_base_velocity_controller/cmd_vel", 1000);
    // subscribe to the topic /base_truth_odom
    ros::Subscriber sub = nh.subscribe("/base_truth_odom", 1000, &odomCallback);

    ros::Rate rate(1e15);

    // create a dummy plan vector of 10 points
    std::vector<geometry_msgs::Pose2D> plan;
    // //vector of positions in the plan
    // std::vector<poseXYT> planPose;

    // // manually adding some points to the plan
    // planPose.push_back({1.5, 2.0, 0.0});
    // planPose.push_back({0.5, 2.0, PI/4});
    // planPose.push_back({-0.5, 1.0, PI/4});
    // planPose.push_back({-1.5, 0.0, 0.0});

    // for (int i = 0; i < planPose.size(); ++i)
    // {
    //     // create a pose2D object to store the current goal
    //     geometry_msgs::Pose2D goal;
    //     goal.x = planPose[i].x;
    //     goal.y = planPose[i].y;
    //     goal.theta = planPose[i].theta;
    //     plan.push_back(goal);
    // }

    // read plan from text file and store in plan vector
    std::ifstream planFile(filePath);
    char option = argv[2][0];
    if (option == '1')
        parse2DAstarPlan(plan, filePath);
    else if (option == '2')
        parse3DAstarPlan(plan, filePath);
    
    // check is the current pose is close to the first point in the plan
    // if yes, remove the first point from the plan
    // if no, calculate the velocity command to move towards the first point in the plan
    // publish the velocity command to the topic /robot_base_velocity_controller/cmd_vel

    while (ros::ok())
    {
        // check if the plan is empty
        if (plan.empty())
        {
            geometry_msgs::Twist vel;
            vel.linear.x = 0;
            tf::Quaternion q(current_pose.pose.orientation.x, 
                            current_pose.pose.orientation.y, 
                            current_pose.pose.orientation.z, 
                            current_pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            wrapAngle(yaw);
            wrapAngle(goal_pose.theta);

            // correct the orientation of the robot based on theta in goal_pose
            if (abs(yaw - goal_pose.theta) > 0.1)
            {
                ROS_INFO_THROTTLE(1, "Correcting orientation");
                if (yaw > goal_pose.theta)
                    vel.angular.z = -0.2;
                else
                    vel.angular.z = 0.2;
                pub.publish(vel);
            }
            else
            {
                vel.angular.z = 0.0;
                pub.publish(vel);
                // exit the while loop
                break;
            }
        }
        else
        {
            auto first_point = plan.back();
            // ROS_INFO("First point in the plan: %f, %f, %f", first_point.x, first_point.y, first_point.theta);
            // get current pose from the topic /base_truth_odom
            // print the current pose
            // ROS_INFO("Current pose: [%f, %f, %f]", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.orientation.z);

            // if (!equalPoses(current_pose, first_point))
            if (true)
            {
                // ROS_INFO("x: %f, y: %f, theta: %f", abs(current_pose.pose.position.x - first_point.x), 
                //                                     abs(current_pose.pose.position.y - first_point.y), 
                //                                     abs(current_pose.pose.orientation.z - first_point.theta));

                // check if the next cell is diagonal to the current cell
                tf::Quaternion q(current_pose.pose.orientation.x, 
                                current_pose.pose.orientation.y, 
                                current_pose.pose.orientation.z, 
                                current_pose.pose.orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                // Translation change
                double dx = first_point.x - current_pose.pose.position.x;
                double dy = first_point.y - current_pose.pose.position.y;

                // if current pose is 0, 0, 0 then continue
                if (current_pose.pose.position.x == 0 && current_pose.pose.position.y == 0)
                {
                    geometry_msgs::Twist vel;
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                    pub.publish(vel);
                    ros::spinOnce();
                    rate.sleep();
                    continue;
                }

                double angleToGoal = atan2(dy, dx);

                // wrap the angle to [-PI, PI]
                wrapAngle(angleToGoal);

                geometry_msgs::Twist vel;

                if (abs(angleToGoal - yaw) > 0.08)
                {
                    // ROS_INFO("Rotating to goal");
                    // ROS_INFO("angleToGoal: %f, yaw: %f", angleToGoal, yaw);
                    vel.linear.x = 0;
                    if (angleToGoal > yaw)
                    {
                        vel.angular.z = 0.5;
                    }
                    else
                    {
                        vel.angular.z = -0.5;
                    }
                }
                else
                {
                    // ROS_INFO("Moving to goal");
                    vel.linear.x = 0.5;
                    vel.angular.z = 0;
                }

                pub.publish(vel);

                if (equalPoses(current_pose, first_point))
                {
                    ROS_INFO("=========================================>>>> Reached goal");
                    // store the executed plan pose as goal_pose
                    goal_pose = first_point;
                    plan.pop_back();
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "Trajectory completed" << std::endl;
    return 0;    
}