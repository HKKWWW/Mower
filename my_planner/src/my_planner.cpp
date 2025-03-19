#include "my_planner.h"
#include "pluginlib/class_list_macros.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav_core::BaseLocalPlanner);

namespace my_planner
{
    MyPlanner::MyPlanner()
    {
        setlocale(LC_ALL, "");
    }
    
    MyPlanner::~MyPlanner()
    {}

    tf::TransformListener* tf_listener_; //单纯使用全局变量可能在ROS启动之前就加载tf
    costmap_2d::Costmap2DROS* costmap_ros_;
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("自制局部规划器");

        tf_listener_ = new tf::TransformListener();

        costmap_ros_ = costmap_ros;
    }

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_;
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        global_plan_ = plan;

        target_index_ = 0;

        pose_adjusting_ = false;

        goal_reached_ = false;

        return true;
    }

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap(); //获取代价地图类中的地图数据
        unsigned char* map_data = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        /*****加入代价地图*****/
        cv::Mat map_image(size_y, size_x, CV_8UC3, cv::Scalar(128, 128, 128));
        for(unsigned int y = 0; y < size_y; y++)
        {
            for(unsigned int x = 0; x < size_x; x++)
            {
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index]; //获取地图内某点的代价值
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(map_index); //获取代价地图像素的地址

                if(cost == 0) pixel = cv::Vec3b(128, 128, 128); //可通行区域，显示为灰色
                else if(cost == 254) pixel = cv::Vec3b(0, 0, 0); //障碍物，显示为黑色
                else if(cost == 253) pixel = cv::Vec3b(255, 255, 0); //禁行区域，显示为浅蓝色
                else
                {
                    //根据灰度值从红到蓝渐变
                    unsigned char blue = 255 - cost;
                    unsigned char red = cost;
                    pixel = cv::Vec3b(blue, 0, red);
                }
            }
        }

        //添加路线到局部代价地图坐标系
        for (int i = 0; i < global_plan_.size(); i++)
        {
            geometry_msgs::PoseStamped pose_odom;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("odom", global_plan_[i], pose_odom); //在局部代价地图中配置参数里，global_frame中配置的坐标系
            double odom_x = pose_odom.pose.position.x;
            double odom_y = pose_odom.pose.position.y;
            double origin_x = costmap->getOriginX();
            double origin_y = costmap->getOriginY();
            double local_x = odom_x - origin_x;
            double local_y = odom_y - origin_y;
            int x = local_x / costmap->getResolution(); //local_x单位为米，还不是栅格地图的坐标值，需除以分辨率
            int y = local_y / costmap->getResolution();

            cv::circle(map_image, cv::Point(x, y), 0, cv::Scalar(255, 0, 255));
            
            if(i >= target_index_ && i < target_index_ + 10)
            {
                cv::circle(map_image, cv::Point(x, y), 0, cv::Scalar(0, 255, 255));
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];

                if(cost >= 253) return false; //检测前方是否有障碍物，如果有返回false，重新进行全局路径规划
            }
        }
        
        map_image.at<cv::Vec3b>(size_y/2, size_x/2) = cv::Vec3b(0, 255, 0); //在图中心点绘制一个绿点

        //翻转地图 OpenCV->ROS
        cv::Mat flipped_image(size_x, size_y, CV_8UC3, cv::Scalar(128, 128, 128)); //长宽交换
        for (unsigned int y = 0; y < size_y; y++)
        {
            for(unsigned int x = 0; x < size_x; x++)
            {
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(y, x);
                flipped_image.at<cv::Vec3b>((size_x - 1 - x), (size_y - 1 -y)) = pixel; //对角线翻转->X轴翻转->Y轴翻转
            }
        }
        map_image = flipped_image;
        
        //显示代价地图
        cv::namedWindow("Map");
        cv::resize(map_image, map_image, cv::Size(size_y*5, size_x*5), 0, 0, cv::INTER_NEAREST); //放大显示，方便观察
        cv::resizeWindow("Map", size_y*5, size_x*5);
        cv::imshow("Map", map_image);
        /**********/

        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0); //凡是需要tf坐标变换的，都需要将时间戳设置为当前时间
        tf_listener_->transformPose("base_link", global_plan_[final_index], pose_final);

        if(pose_adjusting_ == false) //判断是否到达终点范围，如果否
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy); //计算与终点的直线距离

            if(dist < 0.05) pose_adjusting_ = true; //如果距离终点距离小于0.05，则开启调整
        }

        if(pose_adjusting_ == true) //判断是否到达终点范围，如果是则停止线速度，仅使用角速度进行姿态调整
        {
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            ROS_WARN("调整最终姿态, final_yaw = %.2f", final_yaw);

            cmd_vel.linear.x = pose_final.pose.position.x * 1.5;
            cmd_vel.angular.z = final_yaw * 0.5;

            if(abs(final_yaw) < 0.3) //机器最终朝向(yaw)误差
            {
                goal_reached_ = true;
                ROS_WARN("到达终点");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }

            return true;
        }
        
        /*  判断全局路径规划点中符合的点，作为目标点 */
        geometry_msgs::PoseStamped target_pose; 
        for(int i = target_index_; i < global_plan_.size(); i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link", global_plan_[i], pose_base);

            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if(dist > 0.2)
            {
                target_pose = pose_base;
                target_index_ = i;

                break;
            }

            if(i == global_plan_.size()-1) target_pose = pose_base; //机器人到达最后一个路径点时的特殊处理
        }

        cmd_vel.linear.x = target_pose.pose.position.x * 1.5;
        cmd_vel.angular.z = target_pose.pose.position.y * 5.0;
        // ROS_WARN("x = %.2f || y = %.2f || vx = %.2f || w = %.2f", 
        // target_pose.pose.position.x, 
        // target_pose.pose.position.y,
        // cmd_vel.linear.x,
        // cmd_vel.angular.z);

        cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0)); 
        for(int i = 0; i < global_plan_.size(); i++)
        {
            geometry_msgs::PoseStamped pose_base;

            global_plan_[i].header.stamp = ros::Time(0); //注意需要将时间戳更爱为当前时间
            tf_listener_->transformPose("base_link", global_plan_[i], pose_base); //将map下的点转化为base_link坐标系下的点
            
            int cv_x = 300 - pose_base.pose.position.y*100; //将base_link坐标系下的点转换为OpenCV坐标系下的点
            int cv_y = 300 - pose_base.pose.position.x*100;

            cv::circle(plan_image, cv::Point(cv_x, cv_y), 1, cv::Scalar(255, 0, 255));
        }

        cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
        cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
        cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);

        // cv::namedWindow("Plan");
        // cv::imshow("Plan", plan_image);
        cv::waitKey(1);
        
        return true;
    }

    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
}