#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include "MowerPathPlanner.h"
#include "tf/tf.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; //mose_base客户端

int main(int argc, char** argv){
    ros::init(argc, argv, "cleaning_using_movebase");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){ //等待mose_base服务端启动
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal nextGoal; //定义消息以发送目标坐标

    //load the global path.
    tf2_ros::Buffer tf_buffer(ros::Duration(10)); //定义tf监听缓冲区
    tf2_ros::TransformListener tf_listener(tf_buffer); 
    costmap_2d::Costmap2DROS costmap("mower_costmap", tf_buffer);
    MowerPathPlanning *pathPlanner = new MowerPathPlanning(&costmap);

    //full coverage path.
    std::vector<geometry_msgs::PoseStamped> fullCoverPath = pathPlanner->GetPathInROS(); //获得路径规划后的坐标点
    int beginNum = fullCoverPath.size(); //获得规划点数的数量

    /*
    如需进一步将边界区域进行覆盖，则打开以下被注释的代码
    将边界区域的坐标点推入队列末端
    需等到机器人将正常区域打扫完后再进行遍历
    */
    //border tracing path.
    //  std::vector<geometry_msgs::PoseStamped> borderTrackingPath = pathPlanner->GetBorderTrackingPathInROS();
    //  for(int i = 0;i<borderTrackingPath.size();i++)
    //  {
    //      fullCoverPath.push_back(borderTrackingPath[i]); //如果选择将边界进行进一步覆盖，则打开该段函数注释
    //  }

    //main loop
    ros::Rate r(10); //定义频率为10
    for(int i = 0; i < fullCoverPath.size(); i++) //对路径规划后的点进行遍历
    {
        nextGoal.target_pose.header.frame_id = "map";
        nextGoal.target_pose.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped posestamped = fullCoverPath[i];

        //call move base to plan a long distance.
        nextGoal.target_pose.pose.position.x = posestamped.pose.position.x;
        nextGoal.target_pose.pose.position.y = posestamped.pose.position.y;
        nextGoal.target_pose.pose.position.z = 0;
        nextGoal.target_pose.pose.orientation.w = posestamped.pose.orientation.w;
        nextGoal.target_pose.pose.orientation.x = posestamped.pose.orientation.x;
        nextGoal.target_pose.pose.orientation.y = posestamped.pose.orientation.y;
        nextGoal.target_pose.pose.orientation.z = posestamped.pose.orientation.z;

        ROS_INFO("Sending next goal!");
        ac.sendGoal(nextGoal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) //当mose_base成功到达目标点后
        {
            ROS_INFO("Hooray, the base moved a point forward in full path!");
            // pathPlanner->SetCoveredGrid(posestamped.pose.position.x,posestamped.pose.position.y); 
            // pathPlanner->PublishGrid(); //将覆盖后的区域从下采样地图从回复到原原地图，并进行可视化
        }
        else
        {
            ROS_INFO("The base failed to move forward to the next path for some reason!");
            continue;
        }

        // pathPlanner->PublishCoveragePath();
        ros::spinOnce();
        r.sleep();
        }

    delete pathPlanner;
    return 0;
}
