#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

namespace my_planner //命名空间与规划器前部分一致
{
    /*
    类名称与局部规划器后部分一致，继承基类为 nav_core::BaseLocalPlanner，在mose_base中被调用
    */
    class MyPlanner: public nav_core::BaseLocalPlanner
    {
        public:
            MyPlanner(); //构造函数，注意在此函数中并未初始化ROS，在此使用ROS的API可能会导致报错
            ~MyPlanner(); //析构函数

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros); //初始化函数
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan); //此函数可得到全局规划地图
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel); //局部路径规划器发布响应速度，主要函数！
            bool isGoalReached(); //是否到达终点
    };
}

#endif