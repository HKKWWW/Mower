#include "MowerPathPlanner.h"

MowerPathPlanning::MowerPathPlanning(costmap_2d::Costmap2DROS *costmap2d_ros) //初始化构造函数
{
    costmap2d_ros_ = costmap2d_ros;
    costmap2d_ = costmap2d_ros->getCostmap();

    ros::NodeHandle private_nh("~/mower_path_nodeHandle");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("mower_path", 1);
    grid_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("covered_grid", 1);

    string sizeOfCellString, coveredValueStr;

    SIZE_OF_CELL = 3;
    if (private_nh.searchParam("size_of_cell", sizeOfCellString)) //搜索参数,根据名称"size of cell"搜索参数，将对应名称下的参数值赋给sizeOfCellString.
        private_nh.param("size_of_cell", SIZE_OF_CELL, 3);   //设置机器人占据n*n的栅格，决定规划的稀疏   

    GRID_COVERED_VALUE = 0;
    if (private_nh.searchParam("grid_covered_value", coveredValueStr))
        private_nh.param("grid_covered_value", GRID_COVERED_VALUE, 0);

    int sizex = costmap2d_->getSizeInCellsX(); //获取地图尺寸
    int sizey = costmap2d_->getSizeInCellsY();
    cout << "The size of map is " << sizex << "  " << sizey << endl;
    resolution_ = costmap2d_->getResolution(); //分辨率

    srcMap_ = Mat(sizey, sizex, CV_8U); //初始化srcMap_，注意在opencv::Mat中，x、y的对应为相反
    for (int r = 0; r < sizey; r++)
    {
        for (int c = 0; c < sizex; c++)
        {
            srcMap_.at<uchar>(r, c) = costmap2d_->getCost(c, sizey - r - 1); //??sizey-r-1 caution: costmap's origin is at left bottom ,while opencv's pic's origin is at left-top.
            //getCost（）:获取代价值
        }
    }

    initializeMats();
    // initializeCoveredGrid();

    imshow("debugMapImage",srcMap_);
    imshow("debugCellMatImage",cellMat_);
    waitKey(0);
    //imwrite("debug_srcmap.jpg",srcMap_);

    if (!srcMap_.empty())
        initialized_ = true; //这句话説明srcMap_里面得有东西才能说明初始化成功。
    else
        initialized_ = false;
    /*===============================================================================================================================*/
}

bool MowerPathPlanning::initializeMats() //初始化降采样后的地图、空闲坐标变量(freeSpaceVec_)以及活力值映射地图
{
    if(srcMap_.empty())
        return false;

    getCellMatAndFreeSpace(srcMap_, cellMat_, freeSpaceVec_); //初始化降采样后的地图、空闲坐标变量(freeSpaceVec_)

    neuralizedMat_ = Mat(cellMat_.rows, cellMat_.cols, CV_32F);

    initializeNeuralMat(cellMat_, neuralizedMat_); //初始化活力值地图

    return true;
}

//初始化降采样后的地图、空闲坐标变量(freeSpaceVec_)
void MowerPathPlanning::getCellMatAndFreeSpace(Mat srcImg, Mat &cellMat, vector<cellIndex> &freeSpaceVec)
{
cellMat = Mat(srcImg.rows / SIZE_OF_CELL, srcImg.cols / SIZE_OF_CELL, srcImg.type()); //cellMat为降采样后的地图数据

freeSpaceVec.clear();
bool isFree = true;
int r = 0, c = 0, i = 0, j = 0;
for (r = 0; r < cellMat.rows; r++)
{
    for (c = 0; c < cellMat.cols; c++)
    {
        isFree = true;
        for (i = 0; i < SIZE_OF_CELL; i++)
        {
            for (j = 0; j < SIZE_OF_CELL; j++)
            {
                if (srcImg.at<uchar>(r * SIZE_OF_CELL + i, c * SIZE_OF_CELL + j) != costmap_2d::FREE_SPACE)
                {
                    isFree = false;
                    i = SIZE_OF_CELL;
                    break;
                }
            }
        }
        if (isFree)
        {
            cellIndex ci;
            ci.row = r;
            ci.col = c;
            ci.theta = 0;
            freeSpaceVec.push_back(ci); //将空闲区域坐标信息压入栈
            cellMat.at<uchar>(r, c) = costmap_2d::FREE_SPACE; //0
        }
        else
        {
            cellMat.at<uchar>(r, c) = costmap_2d::LETHAL_OBSTACLE;
        } //254
    }
}
cout << "freespace size:" << freeSpaceVec.size() << endl;
//imwrite("cellMat.jpg",cellMat);
return;
}

void MowerPathPlanning::initializeNeuralMat(Mat cellMat, Mat neuralizedMat) //初始化活力值地图
{
    int i = 0, j = 0;
    for (i = 0; i < neuralizedMat.rows; i++)
    {
        for (j = 0; j < neuralizedMat.cols; j++)
        {
            if (cellMat.at<uchar>(i, j) == costmap_2d::LETHAL_OBSTACLE)
                neuralizedMat.at<float>(i, j) = -100000.0; 
            else
                neuralizedMat.at<float>(i, j) = 50.0 / j; //将初始活力值按列划分不同活力值，使其进行弓字型遍历
        }
    }
}

bool MowerPathPlanning::initializeCoveredGrid() //初始化coveredGrid
{
    //使用boost::unique_lock对地图的mutex进行加锁，确保在地图操作过程中不会被其他线程中断。
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap2d_->getMutex()));
    double resolution = costmap2d_->getResolution(); //分辨率

    covered_path_grid_.header.frame_id = "map"; //covered_path_grid_是costmap库中的占据栅格地图消息。
    covered_path_grid_.header.stamp = ros::Time::now();
    covered_path_grid_.info.resolution = resolution;

    covered_path_grid_.info.width = costmap2d_->getSizeInCellsX();
    covered_path_grid_.info.height = costmap2d_->getSizeInCellsY();

    double wx, wy;
    costmap2d_->mapToWorld(0, 0, wx, wy); //从地图坐标系转换至世界坐标系。
    covered_path_grid_.info.origin.position.x = wx - resolution / 2;
    covered_path_grid_.info.origin.position.y = wy - resolution / 2;
    covered_path_grid_.info.origin.position.z = 0.0;
    covered_path_grid_.info.origin.orientation.w = 1.0;

    covered_path_grid_.data.resize(covered_path_grid_.info.width * covered_path_grid_.info.height);

    unsigned char *data = costmap2d_->getCharMap(); //data为一位数组，内存放代价值
    for (unsigned int i = 0; i < covered_path_grid_.data.size(); i++)
    {
        /*if(data[i]==costmap_2d::FREE_SPACE)
            covered_path_grid_.data[i] = costmap_2d::FREE_SPACE;
        else
            covered_path_grid_.data[i] = 0;*/
        covered_path_grid_.data[i] = data[i]; //这里我理解为将代价值赋予到栅格地图的每个对应栅格当中去。
    }
    return true; 
}

vector<geometry_msgs::PoseStamped> MowerPathPlanning::GetPathInROS()
{
    //    vector<geometry_msgs::PoseStamped> resultVec;
    if(!pathVecInROS_.empty())pathVecInROS_.clear();
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Pose pose;
    vector<cellIndex> cellvec;
    cellvec = GetPathInCV();
    /*trasnsform*/
    vector<cellIndex>::iterator iter;
    int sizey = cellMat_.rows;

    for(iter=cellvec.begin(); iter!=cellvec.end();iter++)
    {
        //坐标系转换，将下采样后的地图中的坐标值转换为ROS坐标系下的坐标值
        //下式中SIZE_OF_CELL/2是为使坐标值居于下采样后的每个SIZE_OF_CELL*SIZE_OF_CELL块中的中心点
        costmap2d_->mapToWorld((*iter).col * SIZE_OF_CELL + SIZE_OF_CELL/2 , (sizey-(*iter).row-1)*SIZE_OF_CELL + SIZE_OF_CELL/2, pose.position.x, pose.position.y);
        pose.orientation.w = cos((*iter).theta * PI / 180 / 2); //(sizey-(*iter).row-1)
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = sin((*iter).theta * PI / 180 / 2);
        posestamped.header.stamp= ros::Time::now();
        posestamped.header.frame_id = "map";
        posestamped.pose = pose;
        //上式中w、x为四元素转换的值

        pathVecInROS_.push_back(posestamped); //将转换后的坐标点压入队列中
    }
    publishPlan(pathVecInROS_); //发布路径话题
    cout<<"The path size is "<<pathVecInROS_.size()<<endl;
    return pathVecInROS_;
}

void MowerPathPlanning::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if (!initialized_) 
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //定义路径发布话题的数据类型
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) 
    {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

vector<cellIndex> MowerPathPlanning::GetPathInCV()
{
    mainPlanningLoop();
    return this->pathVec_;
}

void MowerPathPlanning::mainPlanningLoop() //主算法函数 生物激励神经网络算法
{
    cellIndex initPoint,nextPoint, currentPoint;
//    initPoint.row = cellMat_.rows/2; //initPoint to be made interface.
//    initPoint.col = cellMat_.cols/2;
    initPoint.theta = 90;
    if(!costmap2d_ros_->getRobotPose(initPose_))
    {
        ROS_INFO("Failed to get robot location! Please check where goes wrong!");
        return;
    }
    //initPoint.row = initPose_.getOrigin().y()
    unsigned int mx,my;
    double wx = initPose_.pose.position.x;
    double wy = initPose_.pose.position.y;
    //geometry_msgs::PoseStamped current_position;
    //tf::poseStampedTFToMsg(global_pose, current_position);

    bool getmapcoor = costmap2d_->worldToMap(wx,wy,mx,my);
    if(!getmapcoor)
    {
        ROS_INFO("Failed to get robot location in map! Please check where goes wrong!");
        return;
    }
    initPoint.row = cellMat_.rows - my/SIZE_OF_CELL - 1;
    initPoint.col = mx/SIZE_OF_CELL;


    currentPoint = initPoint;
    pathVec_.clear();
    pathVec_.push_back(initPoint);

    float initTheta = initPoint.theta; //initial orientation
    const float c_0 = 0.001;
    float e = 0.0, v = 0.0, deltaTheta = 0.0, lasttheta = initTheta, PI = 3.14159;
    vector<float> thetaVec = {0, 45,90,135,180,225,270,315};

    //the main planning loop
    while(freeSpaceVec_.size()>0)
    {
        //erase current point from free space first.
        vector<cellIndex>::iterator it;
        for(it=freeSpaceVec_.begin();it!=freeSpaceVec_.end();)
        {
            if((*it).row==nextPoint.row && (*it).col==nextPoint.col)
            {it = freeSpaceVec_.erase(it);continue;}
            it++;
        }

        //compute neiborhood's activities
        int maxIndex = 0;
        float max_v = -3;
        neuralizedMat_.at<float>(currentPoint.row ,currentPoint.col) = -2.0;
        for(int id = 0; id < 8; id++)
        {
            deltaTheta = max(thetaVec[id],lasttheta)-min(thetaVec[id],lasttheta);
            if(deltaTheta>180) deltaTheta=360-deltaTheta;
            e = 1 - abs(deltaTheta) / 180;
            switch (id)
            {
                case 0:
                    if(currentPoint.col==neuralizedMat_.cols-1){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row ,currentPoint.col+1) + c_0 * e;
                    break;
                case 1:
                if(currentPoint.col==neuralizedMat_.cols-1 || currentPoint.row == 0){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row-1 ,currentPoint.col+1) + c_0 * e;
                    break;
                case 2:
                if(currentPoint.row == 0){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row-1 ,currentPoint.col) + c_0 * e;
                    break;
                case 3:
                if(currentPoint.col== 0 || currentPoint.row == 0){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row-1 ,currentPoint.col-1) + c_0 * e;
                    break;
                case 4:
                if(currentPoint.col== 0){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row ,currentPoint.col-1) + c_0 * e;
                    break;
                case 5:
                if(currentPoint.col== 0 || currentPoint.row == neuralizedMat_.rows-1){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row+1 ,currentPoint.col-1) + c_0 * e;
                    break;
                case 6:
                if(currentPoint.row == neuralizedMat_.rows-1){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row+1 ,currentPoint.col) + c_0 * e;
                    break;
                case 7:
                if(currentPoint.col==neuralizedMat_.cols-1 || currentPoint.row == neuralizedMat_.rows-1){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row+1 ,currentPoint.col+1) + c_0 * e;
                    break;
                default:
                    break;
            }
            if(v > max_v)
            {
                max_v = v;
                maxIndex=id;
            }
        }


        if(max_v <= 0)
        {
            float dist = 0.0, min_dist = 100000;
            //vector<cellIndex>::iterator min_iter;
            int ii=0, min_index=-1;
            for(it=freeSpaceVec_.begin();it!=freeSpaceVec_.end();it++)
            {
                if(neuralizedMat_.at<float>((*it).row,(*it).col) > 0)
                {
                    dist = sqrt((currentPoint.row-(*it).row)*(currentPoint.row-(*it).row)+(currentPoint.col-(*it).col)*(currentPoint.col-(*it).col));
                    if(dist < min_dist)
                    {
                        min_dist = dist;
                        min_index = ii;
                    }
                }
                ii++;
            }
            if(min_dist==0 || min_index == -1)
            {break;}
            else
            {
                cout << "next point index: "<<min_index<< endl;
                cout << "distance: "<<min_dist << endl;
                nextPoint = freeSpaceVec_[min_index];
                currentPoint = nextPoint;
                pathVec_.push_back(nextPoint);

                continue;
            }
        }

        //next point.
        switch (maxIndex)
        {
        case 0:
            nextPoint.row = currentPoint.row;
            nextPoint.col = currentPoint.col+1;
            break;
        case 1:
            nextPoint.row = currentPoint.row-1;
            nextPoint.col = currentPoint.col+1;
            break;
        case 2:
            nextPoint.row = currentPoint.row-1;
            nextPoint.col = currentPoint.col;
            break;
        case 3:
            nextPoint.row = currentPoint.row-1;
            nextPoint.col = currentPoint.col-1;
            break;
        case 4:
            nextPoint.row = currentPoint.row;
            nextPoint.col = currentPoint.col-1;
            break;
        case 5:
            nextPoint.row = currentPoint.row+1;
            nextPoint.col = currentPoint.col-1;
            break;
        case 6:
            nextPoint.row = currentPoint.row+1;
            nextPoint.col = currentPoint.col;
            break;
        case 7:
            nextPoint.row = currentPoint.row+1;
            nextPoint.col = currentPoint.col+1;
            break;
        default:
            break;
        }
        nextPoint.theta = thetaVec[maxIndex];
        currentPoint = nextPoint;
        pathVec_.push_back(nextPoint);
    }

    Mat resultMat = Mat(srcMap_.rows,srcMap_.cols, CV_8UC3);
    writeResult(resultMat,pathVec_);    
}

double MowerPathPlanning::distance(Point2i pta,Point2i ptb)
{
    return sqrt((pta.x-ptb.x)*(pta.x-ptb.x)+(pta.y-ptb.y)*(pta.y-ptb.y));
}

void MowerPathPlanning::writeResult(Mat resultmat,vector<cellIndex> pathVec) //使用opencv可视化结果
{
    int i = 0,j = 0;
    Point initpoint = Point(pathVec[0].col*SIZE_OF_CELL+SIZE_OF_CELL/2,pathVec[0].row*SIZE_OF_CELL+SIZE_OF_CELL/2);
    for(i = 1; i<pathVec.size();i++)
    {
        Point cupoint = Point(pathVec[i].col*SIZE_OF_CELL+SIZE_OF_CELL/2,pathVec[i].row*SIZE_OF_CELL+SIZE_OF_CELL/2);
        if(sqrt((initpoint.x-cupoint.x)*(initpoint.x-cupoint.x)+(initpoint.y-cupoint.y)*(initpoint.y-cupoint.y))>2)
        {
            line(resultmat,initpoint,cupoint,Scalar(0,255,0),0.3,8);
        }
        else line(resultmat,initpoint,cupoint,Scalar(0,0,255),0.5);
        initpoint = cupoint;
        cout << "The point of step "<<i<<" is: "<<pathVec[i].row<<" "<<pathVec[i].col<<endl;
        /*resultMat.at<Vec3b>(pathVec[i].row*SIZE_OF_CELL,pathVec[i].col*SIZE_OF_CELL)[0] = 0;
        resultMat.at<Vec3b>(pathVec[i].row*SIZE_OF_CELL,pathVec[i].col*SIZE_OF_CELL)[1] = 0;
        resultMat.at<Vec3b>(pathVec[i].row*SIZE_OF_CELL,pathVec[i].col*SIZE_OF_CELL)[2] = 255;*/
    }
    imshow("resultMat",resultmat);
    waitKey(0);
    // imwrite("reaultMat.jpg",resultmat);
}
