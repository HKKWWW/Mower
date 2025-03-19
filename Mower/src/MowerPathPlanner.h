#ifndef MOWERPATHPLANNER_H
#define MOWERPATHPLANNER_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace cv;
using namespace std;

constexpr double PI =3.14159;

struct cellIndex
{
    int row;
    int col;
    double theta; //{0, 45,90,135,180,225,270,315}
};

/*************************************************
 *
 * 读取删格地图并根据占据信息获取其对应的空闲（可行走）空间，
 * 按照遍历算法规划行走路线。
 *
 * **********************************************/
class MowerPathPlanning
{
    public:
        //MowerPathPlanning() = delete;
        MowerPathPlanning(costmap_2d::Costmap2DROS *costmap2d_ros); //初始化构造函数

        vector<geometry_msgs::PoseStamped> GetPathInROS(); 
        vector<geometry_msgs::PoseStamped> GetBorderTrackingPathInROS();

        void SetCoveredGrid(double wx,double wy);
        int GetSizeOfCell(){return this->SIZE_OF_CELL;}

        //for visualization
        void PublishCoveragePath();
        void PublishGrid();
    private:
        //helper functions.
        bool initializeMats(); //初始化cellMat_
        bool initializeCoveredGrid(); //初始化coveredGrid
        void getCellMatAndFreeSpace(Mat srcImg, Mat &cellMat,vector<cellIndex> &freeSpaceVec);
        void initializeNeuralMat(Mat cellMat, Mat neuralizedMat);
        void writeResult(Mat resultmat,vector<cellIndex> pathVec);
        void writeResult(cv::Mat resultmat,std::vector<cv::Point2i> pathVec);
        void mainPlanningLoop();
        double distance(Point2i pta,Point2i ptb);
        bool findElement(vector<cv::Point2i> pointsVec,cv::Point2i pt, int&index);
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        bool cellContainsPoint(cv::Point2i pt,cellIndex cell);

        void GetBorderTrackingPathInCV(vector<cv::Point2i>&resultVec);
        vector<cellIndex> GetPathInCV();


        bool initialized_;
        Mat srcMap_; //costmap的全局变量，但其格式已转化为opencv格式，其格式、分辨率均与代价地图相同
        Mat cellMat_; //对srcMap_地图变量进行划分为SIZE_OF_CELL*SIZE_OF_CELL个栅格地图大小的地图区域
        Mat neuralizedMat_; //对应cellMap_地图中的栅格进行赋值，存储相应的活力值
        vector<cellIndex> freeSpaceVec_; //存储空闲的栅格坐标
        vector<cellIndex> pathVec_; //存储经规划后的路径，使用openmv坐标系 
        vector<geometry_msgs::PoseStamped> pathVecInROS_;

        double resolution_;
        ros::Publisher plan_pub_;
        ros::Publisher grid_pub_;
        nav_msgs::OccupancyGrid covered_path_grid_;

        //tf::TransformListener &tf_;
        geometry_msgs::PoseStamped initPose_;

        costmap_2d::Costmap2D* costmap2d_;
        costmap_2d::Costmap2DROS* costmap2d_ros_;

        int SIZE_OF_CELL; //must be odd number.
        int GRID_COVERED_VALUE;
};

#endif