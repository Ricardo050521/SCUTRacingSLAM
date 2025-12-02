#include <ros/ros.h>
#include "frenet_coordinate_handle.hpp"
#include "fsd_common_msgs/Map.h"
#include <time.h>
#include <iomanip>
#include <vector>
#include "Eigen/Dense"
#include <fstream>
#include <iostream> 
#include <tf/transform_datatypes.h>




namespace ns_frenet_coordinate
{

    // Constructor
    FrenetCoordinateHandle::FrenetCoordinateHandle(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
                                                                                  //boundaryDetector_(nodeHandle)
    {
        ROS_INFO("Constructing Handle");
        loadParameters();
        subscribeToTopics();
        publishToTopics();
        srvCall();
    }

    // Getters
    int FrenetCoordinateHandle::getNodeRate() const { return node_rate_; }

    // Methods
    void FrenetCoordinateHandle::loadParameters()
    {
        ROS_INFO("loading handle parameters");
        if (!nodeHandle_.param<std::string>("global_map_topic_name",
                                            global_map_topic_name_,
                                            "/estimation/slam/map"))
        {
            ROS_WARN_STREAM("Did not load global_map_topic_name. Standard value is: " << global_map_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("slam_map_topic_name",
                                        slam_map_topic_name_,
                                        "/estimation/slam/map_fssim")) {
        ROS_WARN_STREAM("Did not load slam_map_topic_name. Standard value is: " << slam_map_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("gps_centerline_topic_name",
                                            gps_centerline_topic_name_,
                                            "/control/pure_pursuit/asensing_global_path"))
        {
            ROS_WARN_STREAM("Did not load global_map_topic_name. Standard value is: " << gps_centerline_topic_name_);
        }
        if (!nodeHandle_.param("node_rate", node_rate_, 5))
        {
            ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
        }
        if (!nodeHandle_.param<std::string>("slam_state_topic_name",
                                        slam_state_topic_name_,
                                        "/estimation/slam/state")) {
        ROS_WARN_STREAM("Did not load slam_state_topic_name. Standard value is: " << slam_state_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("frenet_coordinate_topic_name",
                                        frenet_coordinate_topic_name_,
                                        "/planning/global_center_path")) {
        ROS_WARN_STREAM("Did not load slam_state_topic_name. Standard value is: " << frenet_coordinate_topic_name_);
        }
        if (!nodeHandle_.param<std::string>("frame_id",
                                        frame_id_,
                                        "fssim/vehicle/base_link ")) {
        ROS_WARN_STREAM("Did not load frame_id. Standard value is: " << frame_id_);
        }
        if (!nodeHandle_.param<std::string>("frenet_coordinate_compute_time_topic_name",
                                            frenet_coordinate_compute_time_topic_name_,
                                            "/frenet_coordinate_compute_time"))
        {
            ROS_WARN_STREAM("Did not load visual_path_topic_name. Standard value is: " << frenet_coordinate_compute_time_topic_name_);
        }
        if (!nodeHandle_.param<double>("evaluation_threshold", 
                                        evaluation_threshold_,
                                        1.0))
        {
            ROS_WARN_STREAM("Did not load evaluation_threshold. Standard value is: " << evaluation_threshold_);
        }
        if (!nodeHandle_.param<double>("dis_evaluation_weight", 
                                        dis_evaluation_weight_, 
                                        0.5)) 
        {
            ROS_WARN_STREAM("Did not load dis_evaluation_weight. Standard value is: " << dis_evaluation_weight_);
        }
        if (!nodeHandle_.param<double>("centre_evaluation_weight", 
                                        centre_evaluation_weight_, 
                                        0.5)) 
        {
            ROS_WARN_STREAM("Did not load centre_evaluation_weight. Standard value is: " << centre_evaluation_weight_);
        }
        if (!nodeHandle_.param<double>("discrete_dis", 
                                        discrete_dis_, 
                                        2.5)) 
        {
            ROS_WARN_STREAM("Did not load discrete_dis. Standard value is: " << discrete_dis_);
        }
        if (!nodeHandle_.param<int>("jump_points", 
                                    jump_points_,
                                    5)) 
        {
            ROS_WARN_STREAM("Did not load jump_points. Standard value is: " << jump_points_);
        }
        if (!nodeHandle_.param<double>("record_dis", 
                                        record_dis_, 
                                        0.01)) 
        {
            ROS_WARN_STREAM("Did not load record_dis. Standard value is: " << record_dis_);
        }
        if (!nodeHandle_.param<double>("start_dis", 
                                        start_dis_, 
                                        10.0)) 
        {
            ROS_WARN_STREAM("Did not load start_dis. Standard value is: " << start_dis_);
        }
        if (!nodeHandle_.param<double>("end_dis", 
                                        end_dis_, 
                                        0.3)) 
        {
            ROS_WARN_STREAM("Did not load end_dis. Standard value is: " << end_dis_);
        }
        if (!nodeHandle_.param<double>("change_angle_rate", 
                                        change_angle_rate_, 
                                        0.8)) 
        {
            ROS_WARN_STREAM("Did not load change_angle_rate. Standard value is: " << change_angle_rate_);
        }
        if (!nodeHandle_.param<double>("change_reference_rate", 
                                        change_reference_rate_, 
                                        1)) 
        {
            ROS_WARN_STREAM("Did not load change_reference_rate. Standard value is: " << change_reference_rate_);
        }
        if (!nodeHandle_.param<bool>("map_load_switch", map_load_switch_, false))
        {
            ROS_WARN_STREAM("Did not load map_load_switch. Standard value is: " << map_load_switch_);
        }
        if (!nodeHandle_.param<bool>("is_start_switch", is_start_switch_, true))
        {
            ROS_WARN_STREAM("Did not load is_start_switch. Standard value is: " << is_start_switch_);
        }
        if (!nodeHandle_.param<std::string>("center_line_save_path",
                                            center_line_save_path_,
                                            "/home/ricardo/map/")) {
            ROS_WARN_STREAM("Did not load center_line_save_path. Standard value is: " << center_line_save_path_);
        }
        nodeHandle_.param<bool>("load_frenet_cones_from_file", load_frenet_cones_from_file_, false);
        // nodeHandle_.param<std::string>("frenet_cones_path", center_line_save_path_, "");
    }

    // 保存中心线
    void FrenetCoordinateHandle::saveCenterLine()
    {

        std::ofstream center_line(center_line_save_path_ + "center_map.txt", std::ofstream::out | std::ofstream::trunc);
        if (!center_line.is_open())
        {
            std::cerr << "Failed to open center map file for writing!" << std::endl;
            return;
        }

        for (const auto& pose : global_center_path.poses)
        {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            // 从四元数获取偏航角
            tf::Quaternion quat(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            );
            tf::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);

            center_line << std::fixed << std::setprecision(4) 
                    << x << " " << y << " " << yaw << std::endl;
        }

        center_line.close();
        ROS_WARN("================== center line have been saved ==================");
        save_center_line_ = true;
    }
    void FrenetCoordinateHandle::subscribeToTopics()
    {
        // 仅在非文件模式下订阅话题
        if (!map_load_switch_) {
            ROS_INFO("subscribe to topics");
            globalMapSubscriber = nodeHandle_.subscribe(global_map_topic_name_, 1, &FrenetCoordinateHandle::globalMapCallback, this);
            GpsSubscriber = nodeHandle_.subscribe(gps_centerline_topic_name_, 1, &FrenetCoordinateHandle::gpscenterlineCallback_new, this);
            // slamStateSubscriber_ = nodeHandle_.subscribe(slam_state_topic_name_, 1, &FrenetCoordinateHandle::slamStateCallback, this);
            // fssim
            // slamMapSubscriber_ = nodeHandle_.subscribe(slam_map_topic_name_, 1, &FrenetCoordinateHandle::slamMapCallback, this);
        }
        
        // globalMapSubscriber = nodeHandle_.subscribe(global_map_topic_name_, 1, &FrenetCoordinateHandle::globalMapCallback, this);
        // //该话题订阅车辆当前位置
        // GpsSubscriber = nodeHandle_.subscribe(gps_centerline_topic_name_, 1, &FrenetCoordinateHandle::gpscenterlineCallback_new, this);
        // //slamStateSubscriber_  = nodeHandle_.subscribe(slam_state_topic_name_, 1, &FrenetCoordinateHandle::slamStateCallback, this);
        // // fssim
        // //slamMapSubscriber_   = nodeHandle_.subscribe(slam_map_topic_name_, 1, &FrenetCoordinateHandle::slamMapCallback, this);
    }

    void FrenetCoordinateHandle::publishToTopics()
    {
        ROS_INFO("publish to topics");
        FrenetDetectionsPublisher = nodeHandle_.advertise<nav_msgs::Path>(frenet_coordinate_topic_name_, 1); //发送全局中心线
        FilteredPath = nodeHandle_.advertise<nav_msgs::Path>("filtered_path", 1); //发送行驶过的路径
        Visual_Center_Line = nodeHandle_.advertise<visualization_msgs::MarkerArray>("visual_center_line", 1);
        Visual_Discrete_Map = nodeHandle_.advertise<visualization_msgs::MarkerArray>("visual_discrete_map", 1);
        //Discrete_Map = nodeHandle_.advertise<fsd_common_msgs::ConeDetections>("discrete_map", 1);
        //frenet_coordinate_compute_time_Publisher = nodeHandle_.advertise<std_msgs::double32>(frenet_coordinate_compute_time_topic_name_, 1);
        Map_Left = nodeHandle_.advertise<fsd_common_msgs::ConeDetections>("map_left", 1); //发送左侧地图，目前只发送slam地图
        Map_Right = nodeHandle_.advertise<fsd_common_msgs::ConeDetections>("map_right", 1); //发送右侧地图，目前只发送slam地图
        Visual_Map_Right = nodeHandle_.advertise<visualization_msgs::MarkerArray>("visual_map_right", 1);
        Visual_Map_Left = nodeHandle_.advertise<visualization_msgs::MarkerArray>("visual_map_left", 1);
        Visual_Temp_1 = nodeHandle_.advertise<visualization_msgs::MarkerArray>("Visual_Temp_1", 1);
        Visual_Temp_2 = nodeHandle_.advertise<visualization_msgs::MarkerArray>("Visual_Temp_2", 1);
        width_pub = nodeHandle_.advertise<frenet_coordinate::RoadWidth>("road_width", 1); //发送道路宽度信息，感觉此信息有误，且规划控制未用到此信息
        auto_color_map_publisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("auto_color_map", 1);
    }

    void FrenetCoordinateHandle::srvCall()
    {
        ROS_INFO("call srv");
    }

    void FrenetCoordinateHandle::slamStateCallback( const fsd_common_msgs::CarState &state)
    {
        setState(state);
    }//TODO
    void FrenetCoordinateHandle::setState(const fsd_common_msgs::CarState &state) 
    {
        state_ = state;
    }



    void FrenetCoordinateHandle::run()
    {
        //若启用文件加载模式，直接从文件读取数据
        if (map_load_switch_) {
            if (!cone_map_loaded_) {
                loadConeMapFromFile();
                cone_map_loaded_ = true;  //只加载一次
            }
            if (!gps_map_loaded_) {
                loadGpsMapFromFile();
                gps_map_loaded_ = true;
            }
        }
        //若已得到车辆一圈的运动轨迹，则将其发布出去
        if(get_gps) FilteredPath.publish(gps_path_filtered);
        //已得到椎桶地图以及车辆运动一圈的轨迹  
        if(get_cone && get_gps && !sign_finish)
        {
            gettimeofday(&start_time, NULL);
            runAlgorithm();
            gettimeofday(&end_time, NULL);

            //计算运算时间
            const auto compute_time = static_cast<float>((end_time.tv_sec * 1000.0 + end_time.tv_usec / 1000.0) - 
                                                    (start_time.tv_sec * 1000.0 + start_time.tv_usec / 1000.0));
            frenet_coordinate_compute_time.data = compute_time;
            std::cout << "Comepute time" << frenet_coordinate_compute_time.data << std::endl;
            //frenet_coordinate_compute_time_Publisher.publish(frenet_coordinate_compute_time);
        }

        if(sign_finish)
        {
            // Visual_Center_Line.publish(marker_array_center_line);
            // Visual_Discrete_Map.publish(marker_array_discrete_line);

            //Discrete_Map.publish(map_for_ribbon);

            global_center_path.header.frame_id = frame_id_;
            FrenetDetectionsPublisher.publish(global_center_path);

            // width_pub.publish(road_width);

            // Map_Left.publish(map_for_publish_left);
            // Map_Right.publish(map_for_publish_right);

            // Visual_Map_Right.publish(marker_array_map_right);
            // Visual_Map_Left.publish(marker_array_map_left);
            // Visual_Temp_1.publish(Temp_1);
            // Visual_Temp_2.publish(Temp_2);

            auto_color_map_publisher_.publish(auto_color_map_);
        }
    }



    void FrenetCoordinateHandle::runAlgorithm()
    {  
        route(gps_center_path);   //该函数将 gps_center_path.poses[i].pose.position.z 的值存储为前面所有相邻两点之间距离的总和 第一个元素的值存为 0，且将第二个元素的角度赋给第一个元素
    // 根据开关决定是从文件加载还是从原始数据处理
        if (load_frenet_cones_from_file_)
        {
            // 尝试从文件加载Frenet锥桶
            if (!loadFrenetConesFromFile())
            {
                ROS_WARN("Failed to load Frenet cones from file, falling back to normal processing");
                // 加载失败，使用正常处理流程
                processConesNormally();
            }
        }
        else
        {
            // 正常处理流程
            processConesNormally();
        }
        
        ROS_INFO("finish frenet transmation");

        //该函数将两侧椎桶通过样条插值法进行拟合（frenet坐标系下），并且计算出了在frenet坐标系下的中心线
        cal_center_path_frenet();

        //将中心线转换成cartesian坐标
        //找到距离距离全局中心线最近的走过的路径点，用该路径点的位姿对全局中心线的点进行frenet到笛卡尔坐标系的变换
        int match_index_rs = 0;
        for(const auto & t : global_center_line_frenet.polygon.points)
        {
            double min_r = 9999;
            for( int k = match_index_rs; k < gps_center_path.poses.size(); k++)
            {
                double dr = fabs(t.x - gps_center_path.poses[k].pose.position.z); //计算全局中心线与之前走过的路线之间的距离差值
                if(dr < min_r)
                {
                    min_r = dr;
                    match_index_rs = k;
                }
                else
                {
                    frenet_to_cartesian(gps_center_path.poses[match_index_rs].pose.position.x, 
                                        gps_center_path.poses[match_index_rs].pose.position.y,
                                        gps_center_path.poses[match_index_rs].pose.orientation.z, 
                                        gps_center_path.poses[match_index_rs].pose.position.z, 
                                        t.y);
                    break;
                }
            }
        }
        
        // std::ofstream write;
        // write.open("/home/auto-lp/map.txt");
        // for(int i = 0;i<global_center_path.poses.size();i++ )
        // {
        //     write << global_center_path.poses[i].pose.position.x << '\t';
        //     write << global_center_path.poses[i].pose.position.y << '\t';
        //     write << "\n";
        //     std::cout << global_center_path.poses[i].pose.position.x << '\t' << global_center_path.poses[i].pose.position.y << std::endl;
        // }
        // std::cout << "Finish write" << std::endl;

        // deal_global_center_path(0, 1); //修复中心线头尾不相接问题 选择为0使用三次多项式拟合，选择1使用可变斜率法
        deal_global_center_path(); //修复中心线头尾不相接问题
        std::cout << "global_center_path: " << global_center_path.poses.size() << std::endl;

        //基于中心线进行两侧离散
        discrete_center_line();

        //对地图进行评价
        evaluate_data = evaluate_discrete_map(); //经过前代更改，目前只会使用slam地图，不会使用离散出的地图 //对gps中心线以及frenet中心线做比较，选出更加合适的线作为全局中心线
        if(evaluate_data == 0)
        {
            map_for_publish_left = map_for_ribbon_left;
            map_for_publish_right = map_for_ribbon_right;
        }
        else
        {
            map_for_publish_left = map_from_slam_left_ordered;
            map_for_publish_right = map_from_slam_right_ordered;
        }

        //MarkerArray可视化左右车道
        for (const auto & cone : map_for_publish_left.cone_detections)
        {
            visiual_marker_array_CUBE(marker_array_map_left, "world", "r", cone.position.x, cone.position.y, cone.position.z);
        }
        for (const auto & cone : map_for_publish_right.cone_detections)
        {
            visiual_marker_array_CUBE(marker_array_map_right, "world", "b", cone.position.x, cone.position.y, cone.position.z);
        }

        //MarkerArray可视化global_center_path
        global_center_path_TO_visiual_marker_array_CUBE();
        
        // std::ofstream write;
        // write.open("/home/marco/AMZ/src/center_line.txt");
        // for(int i = 0;i<global_center_path.poses.size();i++ )
        // {
        //     write << global_center_path.poses[i].pose.position.x << '\t';
        //     write << global_center_path.poses[i].pose.position.y << '\t';
        //     write << "\n";
        //     // std::cout << global_center_path.poses[i].pose.position.x << '\t' << global_center_path.poses[i].pose.position.y << std::endl;
        // }
        sign_finish = true;
        saveCenterLine();  // 保存中心线
        saveFrenetConesAndColorMap();  // 保存锥桶数据
    }

    // 添加一个辅助函数处理正常流程
    void FrenetCoordinateHandle::processConesNormally()
    {
        for(const auto &point : cone_global_disorder_coordinate.polygon.points)
        {
            double x = point.x;
            double y = point.y;
            find_rs(x,y);
        }
    }

  void FrenetCoordinateHandle::globalMapCallback(const fsd_common_msgs::Map &msg)
    {
        if (map_load_switch_) return;

        if(!get_cone )
        {
            map_from_slam_left.cone_detections = msg.cone_red;
            map_from_slam_right.cone_detections = msg.cone_blue;
            // map_from_slam.cone_detections = msg.cone_red;
            // map_from_slam.cone_detections.insert(map_from_slam.cone_detections.end(),msg.cone_blue.begin(),msg.cone_blue.end());
            
            for (size_t i = 0; i < msg.cone_red.size(); i++) //遍历左侧红色椎桶，将左侧红色椎桶放入全部椎桶地图
            {
                fsd_common_msgs::Cone cone;
                cone.position.x = msg.cone_red[i].position.x;
                cone.position.y = msg.cone_red[i].position.y;
                map_from_slam.cone_detections.push_back(cone);
            }
            for (size_t i = 0; i < msg.cone_blue.size(); i++) //遍历右侧蓝色椎桶，将右侧蓝色椎桶放入全部椎桶地图
            {
                fsd_common_msgs::Cone cone;
                cone.position.x = msg.cone_red[i].position.x;
                cone.position.y = msg.cone_red[i].position.y;
                map_from_slam.cone_detections.push_back(cone); 
            }

            int msg_size1 = msg.cone_red.size(), msg_size2 = msg.cone_blue.size();
            std::cout << "msg_size1: " << msg_size1 << ", msg_size2: " << msg_size2 << std::endl;
            double x_g[msg_size1 + msg_size2] = {}, y_g[msg_size1 + msg_size2] = {};
            //x_g前半段储存红色椎桶的x坐标，后半段储存蓝色椎桶的x坐标
            //y_g前半段储存红色椎桶的y坐标，后半段储存蓝色椎桶的y坐标
            for (size_t i = 0; i < msg_size1; i++)
            {
                //存储红色椎桶的xy坐标
                x_g[i] = msg.cone_red[i].position.x;
                y_g[i] = msg.cone_red[i].position.y;
            }
            for (size_t i = 0; i < msg_size2; i++)
            {
                //存储蓝色椎桶的xy坐标
                x_g[i + msg_size1] = msg.cone_blue[i].position.x;
                y_g[i + msg_size1] = msg.cone_blue[i].position.y;
            }

            for(size_t i = 0; i < (msg_size1 + msg_size2); i++)
            {
                geometry_msgs::Point32 t;
                // t.x = x_g[i] + 22.2200411219;
                // t.y = y_g[i] - 26.375358041;
                t.x = x_g[i];
                t.y = y_g[i];
                cone_global_disorder_coordinate.polygon.points.push_back(t); //将t作为多边形的定点构建多边形
                std::cout << "i: " << i << ", cone" << ": "<< x_g[i] << ',' << y_g[i] << std::endl;
            }
            // std::cout << cone_global_disorder_coordinate.polygon.points.size() << std::endl;
            if(msg_size1 != 0 && msg_size2 != 0)
            {
                get_cone = true;
                std::cout << "fet_cone" << std::endl;
            }
        }
    }

    // void FrenetCoordinateHandle::slamMapCallback(const fsd_common_msgs::Map &map)
    // {
    //     std::cout << "sadf" << std::endl;
    //     if(!get_cone )
    //     {
    //         for (const auto &yellow: map.cone_red)
    //         {
    //             geometry_msgs::Point32 pp;
    //             pp.x = yellow.position.x;
    //             pp.y = yellow.position.y;
    //             cone_global_disorder_coordinate.polygon.points.push_back(pp);
    //         }

    //         for (const auto &blue: map.cone_blue)
    //         {
    //             geometry_msgs::Point32 pp;
    //             pp.x = blue.position.x;
    //             pp.y = blue.position.y;
    //             cone_global_disorder_coordinate.polygon.points.push_back(pp);
    //         }

    //         std::cout << "get_cone" << std::endl;
    //         get_cone = true;
    //     }
    // }   

    void FrenetCoordinateHandle::gpscenterlineCallback_new(const fsd_common_msgs::CarState &gpscenterpath)
    {
        if (map_load_switch_) return;

        //get_gps的意思是 是否已经得到车辆运行的轨迹，若已经得到，不运行该函数，否则，运行该段函数计算得到车辆运行的轨迹
        if(get_gps) return;

        geometry_msgs::PoseStamped p;
        double delta_dis = 0.0;
        if(flag_gps_num == 0)
        {
            gps_path_filtered.header.frame_id = "world";
            p.pose.position.x = gpscenterpath.car_state.x;
            p.pose.position.y = gpscenterpath.car_state.y;
            gps_path_filtered.poses.push_back(p); //将车辆运动轨迹记录在  gps_path_filtered  变量中
            flag_gps_num = 1;
            return;
        }

        //计算当前位置与上一位置之间位移的变化量
        delta_dis = sqrt(pow(gpscenterpath.car_state.x - gps_path_filtered.poses[gps_path_filtered.poses.size()-1].pose.position.x,2)+
                         pow(gpscenterpath.car_state.y - gps_path_filtered.poses[gps_path_filtered.poses.size()-1].pose.position.y,2));
        
        //若两帧之间的距离太短，则不将这一帧的位置记录
        if(delta_dis < record_dis_)
        {
            goto SHOW;
        }
        odom_dis += delta_dis;
        //将车辆运动轨迹记录在  gps_path_filtered  变量中
        p.pose.position.x = gpscenterpath.car_state.x;
        p.pose.position.y = gpscenterpath.car_state.y;
        gps_path_filtered.poses.push_back(p);

SHOW:   int gps_path_filtered_path_size = gps_path_filtered.poses.size();
        //计算当前最后位置与初始位置之间的距离
        double start_end_dis = sqrt(pow(gps_path_filtered.poses[0].pose.position.x-
                                gps_path_filtered.poses[gps_path_filtered.poses.size()-1].pose.position.x,2)+
                                pow(gps_path_filtered.poses[0].pose.position.y-
                                gps_path_filtered.poses[gps_path_filtered.poses.size()-1].pose.position.y,2));

        std::cout << "get_gps: " << get_gps << ", get_cone: " << get_cone << std::endl;
        std::cout << "gps_path_filtered_path_size: " << gps_path_filtered_path_size << std::endl;
        std::cout << "delta_dis: " << delta_dis << ", odom_dis: " << odom_dis << std::endl;
        //输出起始位置的x坐标和y坐标
        std::cout << "init: [" << gps_path_filtered.poses[0].pose.position.x << "," 
                               << gps_path_filtered.poses[0].pose.position.y << "]" << std::endl;
        //输出当前位置的x坐标和y坐标
        std::cout << "current: [" << gps_path_filtered.poses[gps_path_filtered_path_size-1].pose.position.x << "," 
                                  << gps_path_filtered.poses[gps_path_filtered_path_size-1].pose.position.y << "]" << std::endl;
        
        //当odom_dis大于阈值时，认为车辆已经启动，并开始检测是否完成回环
        if(odom_dis > start_dis_ && !get_gps)
        {
            // for(int i = 0; i < gpscenterpath.poses.size()-1;i++)
            // {
            //     gps_center_path.poses[i].pose.position.x = gpscenterpath.poses[i].pose.position.x;
            //     gps_center_path.poses[i].pose.position.y = gpscenterpath.poses[i].pose.position.y;
            //     gps_center_path.poses[i].pose.orientation.z = gpscenterpath.poses[i].pose.orientation.z;
            // }
            
            std::cout << "start_end_dis: " << start_end_dis << std::endl;
            //当距离小于阈值时，认为车辆完成一次回环，
            if(start_end_dis < end_dis_)
            {
                get_gps = true;
                std::cout << "==================================" << std::endl;
                ROS_WARN_STREAM("Have gotten gps. gps points: " << gps_path_filtered_path_size);
                std::cout << "==================================" << std::endl;

                gps_center_path = gps_path_filtered;

                //利用两帧之间车辆的位姿变换得到车辆的转角    但是这种计算方式明显有较大的误差
                for (size_t i = 0; i < gps_path_filtered_path_size-1; i++)
                {
                    double yaw,yaw_temp,
                            x1 = gps_center_path.poses[i].pose.position.x,
                            x2 = gps_center_path.poses[i+1].pose.position.x,
                            y1 = gps_center_path.poses[i].pose.position.y,
                            y2 = gps_center_path.poses[i+1].pose.position.y;
                    yaw = atan2(y2-y1,x2-x1);
                    //std::cout << "[" << x2-x1 << "," << y2-y1 << "]\t\tyaw: " << yaw << std::endl;
                    gps_center_path.poses[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
                }
                //认为最后一帧的车辆旋转位姿与倒数第二帧的车辆旋转位姿相同
                gps_center_path.poses[gps_path_filtered_path_size-1].pose.orientation = gps_center_path.poses[gps_path_filtered_path_size-2].pose.orientation;

                // for( auto &p_temp:gps_center_path.poses)
                // {
                //     std::cout << "center_path: [" << p_temp.pose.position.x << ","
                //                                  << p_temp.pose.position.y << "," 
                //                                  << p_temp.pose.position.z << "," 
                //                                  << p_temp.pose.orientation.x << "," 
                //                                  << p_temp.pose.orientation.y << "," 
                //                                  << p_temp.pose.orientation.z << ","
                //                                  << p_temp.pose.orientation.w << "]" << std::endl;
                // }
                get_gps = true;
            }
            // geometry_msgs::Point32 top_p;
            // geometry_msgs::Point32 last_p;
            // top_p.x = gpscenterpath.poses[0].pose.position.x;
            // top_p.y = gpscenterpath.poses[0].pose.position.y;
            // last_p.x = gpscenterpath.poses[gps_num-1].pose.position.x;
            // last_p.y = gpscenterpath.poses[gps_num-1].pose.position.y;
            // double dis = sqrt(pow(top_p.x - last_p.x, 2) +pow(top_p.y - last_p.y, 2));

            // if(dis < 0.3)
            // {
            //     get_gps = true;
            //     std::cout << "finish gps"  << gps_num << std::endl;
            //     gps_center_path = gpscenterpath;
            //     for( auto &p:gps_center_path.poses)
            //     {
            //         p.pose.position.z = 0;
            //         p.pose.orientation.x = 0;
            //         p.pose.orientation.y = 0;
            //         // std::cout << p.pose.position.x << '\t' << p.pose.position.y << std::endl;
            //     }
            // }
        } 
        std::cout << "--------------------" << std::endl;
    }

    /**
     * @brief 该函数将路径中的position.z变量存储为当前点所走过的总距离，用于fernet坐标系
    */
    void FrenetCoordinateHandle::route(const nav_msgs::Path &gps_center_path_)
    {
        double rs_ = 0; //改变量存储为当前点所走过的总距离，用于fernet坐标系
        double k_ ;
        double rx_;
        double ry_;
        double rtheta_;
        for(int i = 0; i < gps_center_path_.poses.size(); i++)
        {
            rx_ = gps_center_path_.poses[i].pose.position.x;
            ry_ = gps_center_path_.poses[i].pose.position.y;

            if(i  == 0) //若为第一个元素，将下一个元素的角度当作第一个元素的角度为其赋值
            {
                rtheta_ = gps_center_path_.poses[i+1].pose.orientation.z;
                gps_center_path.poses[i].pose.orientation.z = rtheta_;
            }

            if(i != 0) //计算两个元素之间的距离并且将其累加到 rs_ 变量中
            {
                rs_ += std::hypot(rx_ - gps_center_path_.poses[i-1].pose.position.x, ry_ - gps_center_path_.poses[i-1].pose.position.y);
            }
            
            gps_center_path.poses[i].pose.position.z = rs_; //将z的值存储为前面所有相邻两点之间距离的总和 第一个元素的值存为 0
        }

        // for(const auto &p:gps_center_path.poses)
        // { 
        //     std::cout << p.pose.position.x << '\t' << p.pose.position.y << '\t' << p.pose.position.z << '\t' << p.pose.orientation.z << std::endl;
        // }
        // std::cout << "finish route" << std::endl;
    }

    /**
     * @brief 该函数找到与该椎桶最近的路径点，并且将该椎桶分为左侧和右侧转换到frenet坐标系中
    */
    void FrenetCoordinateHandle::find_rs(double x, double y)
    {
        int gps_size = gps_center_path.poses.size();
        double min_dis = 9999;
        int flag_min_index = 0;

        //找到与当前椎桶最近的中心线的点
        for(int i = 0; i < gps_size; i++)
        {
            double distance =  std::hypot(x - gps_center_path.poses[i].pose.position.x, y - gps_center_path.poses[i].pose.position.y);
            if(distance < min_dis)
            {
                min_dis = distance;
                flag_min_index = i;
            }
        }

        double flag_rs = gps_center_path.poses[flag_min_index].pose.position.z; //当前位置下所走过的总距离
        double flag_rx = gps_center_path.poses[flag_min_index].pose.position.x;
        double flag_ry = gps_center_path.poses[flag_min_index].pose.position.y;
        double flag_rtheta = gps_center_path.poses[flag_min_index].pose.orientation.z; //当前位置的角度
        cartesian_to_frenet(flag_rs,flag_rx,flag_ry,flag_rtheta,x,y);
    }
    
    // 修改笛卡尔坐标系转弗雷内坐标系函数，增加序号跟踪
    void FrenetCoordinateHandle::cartesian_to_frenet(double rs, double rx, double ry, double rtheta, double x, double y)
    {
        double dx = x - rx;
        double dy = y - ry;
        double cos_theta_r = cos(rtheta);
        double sin_theta_r = sin(rtheta);
        double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
        double cone_d_ = copysign(sqrt(dx * dx + dy * dy), cross_rd_nd);
        double cone_s_ = rs;
        
        // 记录锥桶序号（用于可视化）
        static int left_counter = 1, right_counter = 1;
        int current_index;
        std::string label;
        
        if(cone_d_ > 0)
        {
            left_cone_frenet.insert(std::pair <double, frenetcone> (cone_s_, frenetcone(x,y,cone_d_)));
            current_index = left_counter++;
            label = "left_" + std::to_string(current_index);
            visiual_marker_array_CUBE(auto_color_map_, frame_id_, "r", x, y, 0);
        }
        else
        {
            right_cone_frenet.insert(std::pair <double, frenetcone> (cone_s_, frenetcone(x,y,cone_d_)));
            current_index = right_counter++;
            label = "right_" + std::to_string(current_index);
            visiual_marker_array_CUBE(auto_color_map_, frame_id_, "b", x, y, 0);
        }
        
        // 添加序号文本标记
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = frame_id_;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "cone_labels";
        text_marker.id = auto_color_map_.markers.size() + 1;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = x + 0.1;
        text_marker.pose.position.y = y + 0.1;
        text_marker.pose.position.z = 0.2;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.15;
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0;
        text_marker.text = std::to_string(current_index);
        auto_color_map_.markers.push_back(text_marker);
    }

    //弗雷内坐标系转笛卡尔坐标系
    void FrenetCoordinateHandle::frenet_to_cartesian(double rx,double ry,double rtheta,double s_condition,double d_condition)
    {
        double cos_theta_r = cos(rtheta);
        double sin_theta_r = sin(rtheta);
        
        double x = rx - sin_theta_r * d_condition;
        double y = ry + cos_theta_r * d_condition;

        geometry_msgs::PoseStamped p;
        p.pose.position.x = x;
        p.pose.position.y = y;
        global_center_path.poses.push_back(p);
    }

    //将两侧的椎桶转换到frenet坐标系下，进行样条插值拟合，并计算出frenet坐标系下的中心线
    void FrenetCoordinateHandle::cal_center_path_frenet()
    {
        for(std::map<double, frenetcone>::iterator it = left_cone_frenet.begin(); it != left_cone_frenet.end(); it++)
        {
            frenetcone point = it->second; //point中存储该椎桶的笛卡尔坐标以及fernet的纵坐标
            geometry_msgs::Point32 t;
            t.x = it->first; //t.x存储该椎桶在fernet坐标系下的横坐标
            t.y = point.d; //t.y存储该椎桶在frenet坐标系下的纵坐标
            left_cone_.polygon.points.push_back(t); //left_cone_存储左侧椎桶的全部frenet坐标
            visiual_marker_array_CUBE(Temp_1,"world","r", t.x, t.y, 0);
        }

        for(std::map<double, frenetcone>::iterator it = right_cone_frenet.begin(); it != right_cone_frenet.end(); it++)
        {
            frenetcone point = it->second;
            geometry_msgs::Point32 t;
            t.x = it->first;
            t.y = point.d;
            right_cone_.polygon.points.push_back(t);
            visiual_marker_array_CUBE(Temp_1,"world","b", t.x, t.y, 0);
        }

        std::cout << "left_cone_: " << left_cone_.polygon.points.size() << ", right_cone_: " << right_cone_.polygon.points.size() << std::endl;



        double zoom = 10;
        //左右桩通头尾对齐
        //得出最前面的椎桶的x坐标（frenet坐标系下）
        s_start = left_cone_.polygon.points[0].x <right_cone_.polygon.points[0].x ? right_cone_.polygon.points[0].x :  left_cone_.polygon.points[0].x;
        //得出最后面的椎桶的x坐标（fernet坐标系下）
        s_end = left_cone_.polygon.points[left_cone_.polygon.points.size() - 1].x < right_cone_.polygon.points[right_cone_.polygon.points.size() - 1].x ? left_cone_.polygon.points[left_cone_.polygon.points.size() - 1].x : right_cone_.polygon.points[right_cone_.polygon.points.size() - 1].x;
        ROS_INFO("start: %f, end: %f", s_start, s_end);
        int n_origin = (s_end - s_start)  / 1 + 1; //将 s_end - s_start 的值转换为整数再加上 1
        //增加n点的数量，增加样条插值法的准确性
        int n = int(zoom) * n_origin;
        //std::cout << "n: " << n << std::endl;
        double s[n] ={0};
        
        //s[i]的值为距离原点的距离
        for(int i = 0; i < n; i++)
        {
            s[i] = s_start + (double(1 / zoom) * i);
            //std::cout << "i: " << i  << " " << "val: " << s[i] << " ";
        }
        //std::cout << std::endl;

        //计算样条插值
        ns_frenet_coordinate::Spline sp_left(left_cone_, left_cone_.polygon.points.size());
        sp_left.run_algorithm();

        ns_frenet_coordinate::Spline sp_right(right_cone_, right_cone_.polygon.points.size());
        sp_right.run_algorithm();

        //找对应样条线段
        int match_index_left = 1;
        int match_index_right = 1;

        for(int i = 0; i < n; i++)
        {
            for(int j = match_index_left; j < left_cone_.polygon.points.size(); j++ )
            {
                if(s[i] >= left_cone_.polygon.points[j-1].x  && s[i] < left_cone_.polygon.points[j].x )
                {
                    match_index_left = j;
                    //ROS_INFO("%d left find %d", i, j);
                    break;
                }
            }

            for(int j = match_index_right; j < right_cone_.polygon.points.size(); j++ )
            {
                if(s[i] >= right_cone_.polygon.points[j-1].x  && s[i] < right_cone_.polygon.points[j].x )
                {
                    match_index_right = j;
                    //ROS_INFO("%d right find %d", i, j);
                    break;
                }
            }
            //带入样条插值
            double d_left = sp_left.cal_spline_course(match_index_left,s[i]);
            double d_right = sp_right.cal_spline_course(match_index_right,s[i]);

            geometry_msgs::Point32 t;
            t.x = s[i];
            t.y = d_left;
            visiual_marker_array_CUBE(Temp_2,"world","r", t.x, t.y, 0);
            //ROS_INFO("%f left added", s[i]);
            left_cone_sp.polygon.points.push_back(t);
            t.y = d_right;
            visiual_marker_array_CUBE(Temp_2,"world","b", t.x, t.y, 0);
            //ROS_INFO("%f right added", s[i]);
            right_cone_sp.polygon.points.push_back(t);
        }
        //计算frenet坐标下的中心线
        double width;
        for(int i = 0 ; i< n; i++)
        {
            geometry_msgs::Point32 t;
            t.x = s[i];
            t.y = (left_cone_sp.polygon.points[i].y + right_cone_sp.polygon.points[i].y ) / 2;
            width = (left_cone_sp.polygon.points[i].y - right_cone_sp.polygon.points[i].y ) / 2;
            global_center_line_frenet.polygon.points.push_back(t); //以之前走过的路线为x轴，计算出的中心线也在frenet坐标系下
            road_width.data.push_back(width);
            visiual_marker_array_CUBE(Temp_2,"world","g", t.x, t.y, 0);
        }


        // std::ofstream write;
        // write.open("/home/aracing/lr.txt");
        // for(int i = 0 ; i < n; i++)
        // {
        //     write << left_cone_sp.polygon.points[i].x << '\t';
        //     write << left_cone_sp.polygon.points[i].y << '\t';
        //     write << right_cone_sp.polygon.points[i].x << '\t';
        //     write << right_cone_sp.polygon.points[i].y << '\t';
        //     write << global_center_line_frenet.polygon.points[i].x << '\t';
        //     write << global_center_line_frenet.polygon.points[i].y << '\t';
        //     write << '\n';
        //     // std::cout << "left_s" << '\t' << left_cone_sp.polygon.points[i].x << '\t' <<  "left_y" << '\t' << left_cone_sp.polygon.points[i].y << '\t'
        //     //                     <<"right_s" << '\t' << right_cone_sp.polygon.points[i].x << '\t' <<  "right_y" << '\t' << right_cone_sp.polygon.points[i].y << '\t'
        //     //                     << "center_s" << '\t' << global_center_line_frenet.polygon.points[i].x << '\t' <<  "center_y" << '\t' << global_center_line_frenet.polygon.points[i].y << std::endl;
        // }
        // write.close();
        // std::cout << "\nwrite complete\n" << std::endl;
    }

    void FrenetCoordinateHandle::visiual_marker_array_CUBE(visualization_msgs::MarkerArray & marker_array, std::string frame_id_, std::string color, 
                                                            double x, double y, double z)
    {
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = frame_id_;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.lifetime = ros::Duration();
        bbox_marker.frame_locked = true;
        bbox_marker.type = visualization_msgs::Marker::CUBE;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        if(color == "r")
        {
            bbox_marker.color.r = 1.0f;
            bbox_marker.color.g = 0.0f;
            bbox_marker.color.b = 0.0f;
            bbox_marker.color.a = 1.0f;
        }
        if(color == "g")
        {
            bbox_marker.color.r = 0.0f;
            bbox_marker.color.g = 1.0f;
            bbox_marker.color.b = 0.0f;
            bbox_marker.color.a = 1.0f;
        }
        if(color == "b")
        {
            bbox_marker.color.r = 0.0f;
            bbox_marker.color.g = 0.0f;
            bbox_marker.color.b = 1.0f;
            bbox_marker.color.a = 1.0f;
        }
        bbox_marker.scale.x = 0.25;
        bbox_marker.scale.y = 0.25;
        bbox_marker.scale.z = 0.25;
        // default val to fix warn of rviz
        bbox_marker.pose.orientation.x = 0;
        bbox_marker.pose.orientation.y = 0;
        bbox_marker.pose.orientation.z = 0;
        bbox_marker.pose.orientation.w = 1;
        bbox_marker.id = marker_array.markers.size();
        bbox_marker.pose.position.x = x;
        bbox_marker.pose.position.y = y;
        bbox_marker.pose.position.z = z;
        marker_array.markers.push_back(bbox_marker);
    }

    void FrenetCoordinateHandle::visiual_marker_array_ARROW(visualization_msgs::MarkerArray & marker_array, std::string frame_id_, std::string color, 
                                                            double x1, double y1, double z1, double x2, double y2, double z2)
    {
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = frame_id_;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.lifetime = ros::Duration();
        bbox_marker.frame_locked = true;
        bbox_marker.type = visualization_msgs::Marker::ARROW;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        if(color == "r")
        {
            bbox_marker.color.r = 1.0f;
            bbox_marker.color.g = 0.0f;
            bbox_marker.color.b = 0.0f;
            bbox_marker.color.a = 1.0f;
        }
        if(color == "g")
        {
            bbox_marker.color.r = 0.0f;
            bbox_marker.color.g = 1.0f;
            bbox_marker.color.b = 0.0f;
            bbox_marker.color.a = 1.0f;
        }
        if(color == "b")
        {
            bbox_marker.color.r = 0.0f;
            bbox_marker.color.g = 0.0f;
            bbox_marker.color.b = 1.0f;
            bbox_marker.color.a = 1.0f;
        }
        bbox_marker.scale.x = 0.05;
        bbox_marker.scale.y = 0.08;
        bbox_marker.scale.z = 0.1;
        bbox_marker.id = marker_array.markers.size();
        geometry_msgs::Point p1, p2;
        p1.x = x1;
        p1.y = y1;
        p1.z = z1;
        p2.x = x1 + x2;
        p2.y = y1 + y2;
        p2.z = z1 + z2;
        bbox_marker.points.push_back(p1) ;
        bbox_marker.points.push_back(p2) ;
        marker_array.markers.push_back(bbox_marker);
    }

    void FrenetCoordinateHandle::discrete_center_line()
    {
        int global_center_path_path_size = global_center_path.poses.size();
        fsd_common_msgs::Cone slam_temp;
        for (size_t i = 0; i < global_center_path_path_size-1; i++)
        {
            
            double x1 = global_center_path.poses[i].pose.position.x,
                   x2 = global_center_path.poses[i+1].pose.position.x,
                   y1 = global_center_path.poses[i].pose.position.y,
                   y2 = global_center_path.poses[i+1].pose.position.y;
        
            double delta_x = x2 - x1,
                  delta_y = y2 - y1,
                  normal_dis = sqrt(delta_x * delta_x + delta_y * delta_y);
        
            double xx = delta_y * discrete_dis_ / normal_dis,
                  yy = -delta_x * discrete_dis_ / normal_dis;
            
            slam_temp.position.x = x1 + xx;
            slam_temp.position.y = y1 + yy;
            map_for_ribbon.cone_detections.push_back(slam_temp);
            map_for_ribbon_right.cone_detections.push_back(slam_temp);
            visiual_marker_array_CUBE(marker_array_discrete_line,"world","b",slam_temp.position.x,slam_temp.position.y,0);

            slam_temp.position.x = x1 - xx;
            slam_temp.position.y = y1 - yy;
            map_for_ribbon.cone_detections.push_back(slam_temp);
            map_for_ribbon_left.cone_detections.push_back(slam_temp);
            visiual_marker_array_CUBE(marker_array_discrete_line,"world","r",slam_temp.position.x,slam_temp.position.y,0);

        }
        double x1 = global_center_path.poses[global_center_path_path_size-1].pose.position.x,
               x2 = global_center_path.poses[0].pose.position.x,
               y1 = global_center_path.poses[global_center_path_path_size-1].pose.position.y,
               y2 = global_center_path.poses[0].pose.position.y;

        double delta_x = x2 - x1,
              delta_y = y2 - y1,
              normal_dis = sqrt(delta_x * delta_x + delta_y * delta_y);

        double xx = delta_y * discrete_dis_ / normal_dis,
              yy = -delta_x * discrete_dis_ / normal_dis;

        slam_temp.position.x = x1 + xx;
        slam_temp.position.y = y1 + yy;
        map_for_ribbon.cone_detections.push_back(slam_temp);
        map_for_ribbon_right.cone_detections.push_back(slam_temp);
        visiual_marker_array_CUBE(marker_array_discrete_line,"world","b",slam_temp.position.x,slam_temp.position.y,0);

        slam_temp.position.x = x1 - xx;
        slam_temp.position.y = y1 - yy;
        map_for_ribbon.cone_detections.push_back(slam_temp);
        map_for_ribbon_left.cone_detections.push_back(slam_temp);
        visiual_marker_array_CUBE(marker_array_discrete_line,"world","r",slam_temp.position.x,slam_temp.position.y,0);
 
    }

    // //对离散结果进行评价
    // double FrenetCoordinateHandle::evaluate_discrete_map()
    // {
        
    //     double n_map_for_ribbon = map_for_ribbon.cone_detections.size(), n_map_from_slam = map_from_slam.cone_detections.size();
    //     std::cout << "map_for_ribbon: " << n_map_for_ribbon << ", map_from_slam: " << n_map_from_slam << std::endl;

    //     //距离偏差
    //     for (size_t i = 0; i < n_map_from_slam; i++)
    //     {
    //         double min_dis = 99999999.0;
    //         for (size_t j = 0; j < n_map_for_ribbon; j++)
    //         {
    //             double dis_temp = sqrt(pow(map_for_ribbon.cone_detections[j].position.x - map_from_slam.cone_detections[i].position.x, 2) +
    //                                   pow(map_for_ribbon.cone_detections[j].position.y - map_from_slam.cone_detections[i].position.y, 2));
    //             min_dis = dis_temp > min_dis ? min_dis : dis_temp;
    //         }
    //         //std::cout << "min_dis: " << min_dis << std::endl;
    //         dis_evaluate_map += min_dis;
    //     }
    //     dis_evaluate_map = dis_evaluate_map / n_map_from_slam;

    //     //形心偏差
    //     int n_map_from_slam_left = map_from_slam_left.cone_detections.size(), n_map_from_slam_right = map_from_slam_right.cone_detections.size();
    //     int n_map_for_ribbon_left = map_for_ribbon_left.cone_detections.size(), n_map_for_ribbon_right = map_for_ribbon_right.cone_detections.size();
    //     std::cout << "n_map_from_slam_left: " << n_map_from_slam_left << ", n_map_from_slam_right: " << n_map_from_slam_right << std::endl;
    //     std::cout << "n_map_for_ribbon_left: " << n_map_for_ribbon_left << ", n_map_for_ribbon_right: " << n_map_for_ribbon_right << std::endl;
    //     std::cout << std::endl;

    //     double cx_map_from_slam_left = 0, cy_map_from_slam_left = 0, A_map_from_slam_left = 0, 
    //           cx_map_from_slam_right = 0, cy_map_from_slam_right = 0, A_map_from_slam_right = 0, 
    //           cx_map_for_ribbon_left = 0, cy_map_for_ribbon_left = 0, A_map_for_ribbon_left = 0, 
    //           cx_map_for_ribbon_right = 0, cy_map_for_ribbon_right = 0, A_map_for_ribbon_right = 0;
        
    //     calculate_center(map_from_slam_left, n_map_from_slam_left, 1.0, 0,
    //                      cx_map_from_slam_left, cy_map_from_slam_left, A_map_from_slam_left);
    //     std::cout << "cx_map_from_slam_left: " << cx_map_from_slam_left << " cy_map_from_slam_left: " << cy_map_from_slam_left 
    //               << " A_map_from_slam_left: " << A_map_from_slam_left << std::endl;
        
    //     calculate_center(map_from_slam_right, n_map_from_slam_right, -1.0, 1,
    //                      cx_map_from_slam_right, cy_map_from_slam_right, A_map_from_slam_right);
    //     std::cout << "cx_map_from_slam_right: " << cx_map_from_slam_right << " cy_map_from_slam_right: " << cy_map_from_slam_right 
    //               << " A_map_from_slam_right: " << A_map_from_slam_right << std::endl;
        
    //     calculate_center(map_for_ribbon_left, n_map_for_ribbon_left, 1.0, 2,
    //                      cx_map_for_ribbon_left, cy_map_for_ribbon_left, A_map_for_ribbon_left);
    //     std::cout << "cx_map_for_ribbon_left: " << cx_map_for_ribbon_left << " cy_map_for_ribbon_left: " << cy_map_for_ribbon_left 
    //               << " A_map_for_ribbon_left: " << A_map_for_ribbon_left << std::endl;
        
    //     calculate_center(map_for_ribbon_right, n_map_for_ribbon_right, -1.0, 2,
    //                      cx_map_for_ribbon_right, cy_map_for_ribbon_right, A_map_for_ribbon_right);
    //     std::cout << "cx_map_for_ribbon_right: " << cx_map_for_ribbon_right << " cy_map_for_ribbon_right: " << cy_map_for_ribbon_right 
    //               << " A_map_for_ribbon_right: " << A_map_for_ribbon_right << std::endl;
    //     std::cout << std::endl;

    //     double A_map_from_slam = A_map_from_slam_left + A_map_from_slam_right;
    //     double A_map_for_ribbon = A_map_for_ribbon_left + A_map_for_ribbon_right;
    //     double cx_map_from_slam = (cx_map_from_slam_left*A_map_from_slam_left + cx_map_from_slam_right*A_map_from_slam_right) / A_map_from_slam;
    //     double cy_map_from_slam = (cy_map_from_slam_left*A_map_from_slam_left + cy_map_from_slam_right*A_map_from_slam_right) / A_map_from_slam;
    //     double cx_map_for_ribbon = (cx_map_for_ribbon_left*A_map_for_ribbon_left + cx_map_for_ribbon_right*A_map_for_ribbon_right) / A_map_for_ribbon;
    //     double cy_map_for_ribbon = (cy_map_for_ribbon_left*A_map_for_ribbon_left + cy_map_for_ribbon_right*A_map_for_ribbon_right) / A_map_for_ribbon;
        
    //     std::cout << "cx_map_from_slam: " << cx_map_from_slam << " cy_map_from_slam: " << cy_map_from_slam << std::endl;
    //     std::cout << "cx_map_for_ribbon: " << cx_map_for_ribbon << " cy_map_for_ribbon: " << cy_map_for_ribbon << std::endl;
    //     std::cout << "A_map_from_slam: " << A_map_from_slam << " A_map_for_ribbon: " << A_map_for_ribbon << std::endl;
        
    //     std::cout << std::endl;
    //     centre_evaluate_map = sqrt(pow(cx_map_from_slam - cx_map_for_ribbon,2) + pow(cy_map_from_slam - cy_map_for_ribbon,2));
    //     area_evaluate_map = sqrt(abs(A_map_from_slam - A_map_for_ribbon));
    //     std::cout << "dis_evaluate_map: " << dis_evaluate_map 
    //               << ", centre_evaluate_map: " << centre_evaluate_map 
    //               << ", area_evaluate_map: " << area_evaluate_map << std::endl;
    //     std::cout << std::endl;
    //     double final_evaluate = dis_evaluation_weight_*dis_evaluate_map + centre_evaluation_weight_* centre_evaluate_map;
    //     std::cout << "final_evaluate: " << final_evaluate << std::endl;
    //     /*
    //     if ( final_evaluate < evaluation_threshold_)
    //     {
    //         std::cout << "I will choose Map_For_Ribbon !!" << std::endl;
    //         return 0;
    //     }
    //     else
    //     {
    //         std::cout << "I will choose Map_From_Slam !!" << std::endl;
    //         return 1;
    //     }
    //     */
    //    std::cout << "I will choose Map_From_Slam !!" << std::endl;
    //     return 1;
    // }

    /**
     * @brief 新的地图评价算法
     */
    double FrenetCoordinateHandle::evaluate_discrete_map()
    {
        geometry_msgs::Point map_center, gps_center, coordinate_center;

        //计算椎桶地图形心
        int counter_map = 0;
        for(auto i: map_from_slam.cone_detections)
        {
            map_center.x += i.position.x;
            map_center.y += i.position.y;
            counter_map++;
        }
        map_center.x = map_center.x / counter_map;
        map_center.y = map_center.y / counter_map;

        //计算gps线形心
        int counter_gps = 0;
        for(auto j: gps_path_filtered.poses)
        {
            gps_center.x += j.pose.position.x;
            gps_center.y += j.pose.position.y;
            counter_gps++;
        }
        gps_center.x = gps_center.x / counter_gps;
        gps_center.y = gps_center.y / counter_gps;

        //计算中心线形心
        int counter_coordinate = 0;
        for(auto k: global_center_path.poses)
        {
            coordinate_center.x += k.pose.position.x;
            coordinate_center.y += k.pose.position.y;
            counter_coordinate++;
        }
        coordinate_center.x = coordinate_center.x / counter_coordinate;
        coordinate_center.y = coordinate_center.y / counter_coordinate;

        //计算形心偏差,认为椎桶地图为准确地图
        double dis_map_gps, dis_map_coordinate;
        dis_map_gps = sqrt(pow(map_center.x - gps_center.x, 2) + pow(map_center.y - gps_center.y, 2));
        dis_map_coordinate = sqrt(pow(map_center.x - coordinate_center.x, 2) + pow(map_center.y - coordinate_center.y, 2));

        std::cout << "dis between map gps: " << dis_map_gps << std::endl;
        std::cout << "dis between map coordinate: " << dis_map_coordinate << std::endl;

        if(dis_map_gps < dis_map_coordinate)
        {
            std::cout << "i will use gps line" << std::endl;
            return 1;
        }
        else
        {
            std::cout << "i will use coordinate line" << std::endl;
            return 0;
        }
    }


    void FrenetCoordinateHandle::calculate_center(fsd_common_msgs::ConeDetections & map, int map_size, double in_or_out, double flag_which,
                                                  double & cx, double & cy, double & A)
    {
        std::cout << std::endl;
        double init_x = 99999999.0, init_y = 0.0;
        for (size_t i = 0; i < map_size; i++)
        {
            if(init_x > map.cone_detections[i].position.x)
            {
                init_x = map.cone_detections[i].position.x;
                init_y = map.cone_detections[i].position.y;
            }
        }

        double polar_coordinates[map_size][4] = {};
        for (size_t i = 0; i < map_size; i++)
        {
            double vector_x = map.cone_detections[i].position.x - init_x, vector_y = map.cone_detections[i].position.y - init_y;
            if(vector_x == 0 && vector_x == 0)
            {
                polar_coordinates[i][0] = -9999999.0;
                polar_coordinates[i][1] = -9999999.0;
                polar_coordinates[i][2] = -9999999.0;
                polar_coordinates[i][3] = -9999999.0;
                continue;
            }
            polar_coordinates[i][0] = atan2(vector_y, vector_x);
            polar_coordinates[i][1] = sqrt(pow(vector_x, 2) + pow(vector_y, 2));
            polar_coordinates[i][2] = map.cone_detections[i].position.x;
            polar_coordinates[i][3] = map.cone_detections[i].position.y;
        }

        for (size_t i = 0; i < map_size-1; i++)
        {
            for (size_t j = 0; j < map_size - 1 - i; j++)
            {
                if(polar_coordinates[j][0] < polar_coordinates[j+1][0])
                {
                    double temp = polar_coordinates[j+1][0];
                    polar_coordinates[j+1][0] = polar_coordinates[j][0];
                    polar_coordinates[j][0] = temp;
                    
                    temp = polar_coordinates[j+1][1];
                    polar_coordinates[j+1][1] = polar_coordinates[j][1];
                    polar_coordinates[j][1] = temp;

                    temp = polar_coordinates[j+1][2];
                    polar_coordinates[j+1][2] = polar_coordinates[j][2];
                    polar_coordinates[j][2] = temp;

                    temp = polar_coordinates[j+1][3];
                    polar_coordinates[j+1][3] = polar_coordinates[j][3];
                    polar_coordinates[j][3] = temp;
                }
            }
        }

        if(flag_which == 0)
        {
            std::cout << "Will Copy Data To map_from_slam_left_ordered" << std::endl;
            fsd_common_msgs::Cone slam_temp;
            slam_temp.position.x = init_x;
            slam_temp.position.y = init_y;
            map_from_slam_left_ordered.cone_detections.push_back(slam_temp);
            for (size_t i = 0; i < map_size-1; i++)
            {
                //std::cout << i << std::endl;
                slam_temp.position.x = polar_coordinates[i][2];
                slam_temp.position.y = polar_coordinates[i][3];
                map_from_slam_left_ordered.cone_detections.push_back(slam_temp);
            }
            std::cout << "map_from_slam_left_ordered: " << map_from_slam_left_ordered.cone_detections.size() << std::endl;
        }

        if(flag_which == 1)
        {
            std::cout << "Will Copy Data To map_from_slam_right_ordered" << std::endl;
            fsd_common_msgs::Cone slam_temp;
            slam_temp.position.x = init_x;
            slam_temp.position.y = init_y;
            map_from_slam_right_ordered.cone_detections.push_back(slam_temp);
            for (size_t i = 0; i < map_size-1; i++)
            {
                slam_temp.position.x = polar_coordinates[i][2];
                slam_temp.position.y = polar_coordinates[i][3];
                map_from_slam_right_ordered.cone_detections.push_back(slam_temp);
            }
            std::cout << "map_from_slam_right_ordered: " << map_from_slam_right_ordered.cone_detections.size() << std::endl;
        }

        double center[map_size-2][2] = {}, area[map_size-2] = {};
        for (size_t i = 0; i < map_size - 2; i++)
        {
            double x2 = polar_coordinates[i][2], y2 = polar_coordinates[i][3], 
                  x3 = polar_coordinates[i+1][2], y3 = polar_coordinates[i+1][3];
            center[i][0] = (init_x + x2 + x3) / 3, center[i][1] = (init_y + y2 + y3) / 3;
            area[i] = in_or_out*((x2 - init_x)*(y3 - init_y) - (x3 - init_x)*(y2 - init_y)) / 2;
        }

        for (size_t i = 0; i < map_size - 2; i++)
        {
            cx += center[i][0] * area[i];
            cy += center[i][1] * area[i];
            A += area[i];
        }
        cx = cx / A;
        cy = cy / A;
        
    }

    // /***
    //  * @brief 该函数用于处理中心线的头尾不相接问题
    //  * @param flag_choose_what 选择处理方法，0使用三次多项式拟合法 1使用可变斜率法
    //  * @param line_choose_what 选择所处理的中心线 0为gps所记录的中心线 1为使用frenet坐标系算法计算出来的中心线
    //  */
    // void FrenetCoordinateHandle::deal_global_center_path(int flag_choose_what, int line_choose_what)
    // {
    //     nav_msgs::Path& temp_line = (line_choose_what == 0) ? gps_path_filtered : global_center_path;
        
    //     int temp_line_size = temp_line.poses.size();
    //     //计算前两个点之间的距离作为参考距离
    //     double reference_dis = sqrt(pow(temp_line.poses[0].pose.position.x - temp_line.poses[1].pose.position.x,2) + 
    //                                 pow(temp_line.poses[0].pose.position.y - temp_line.poses[1].pose.position.y,2));
        
    //     double start_point_x = temp_line.poses[0].pose.position.x, 
    //             start_point_y = temp_line.poses[0].pose.position.y;
    //     double end_point_x = temp_line.poses[temp_line_size - 1].pose.position.x, 
    //             end_point_y = temp_line.poses[temp_line_size - 1].pose.position.y;
    //     double start_end_dis = sqrt(pow(start_point_x - end_point_x,2) + pow(start_point_y - end_point_y,2));
    //     std::cout << "reference_dis: " << reference_dis << ", start_end_dis: " << start_end_dis << std::endl;
    //     std::cout << "start: [" << start_point_x << ", " << start_point_y << "]" << std::endl;
    //     std::cout << "end: [" << end_point_x << ", " << end_point_y << "]" << std::endl;

    //     geometry_msgs::PoseStamped p;
    //     //若结束点在起始点之前，且起止点之间的距离小于参考距离*2，则认为中心线生成符合预期，不需要补充
    //     if(start_end_dis > 0 && start_end_dis < (reference_dis * 2))
    //     {
    //         std::cout << "Don't need any replenish !!" << std::endl;
    //     }
    //     else if(start_end_dis >= (reference_dis * 2) && start_end_dis <= (reference_dis * 4))
    //     {
    //         std::cout << "So close, Just need to make Linear Interpolation !!" << std::endl;
    //         p.pose.position.x = (start_point_x + end_point_x) / 2;
    //         p.pose.position.y = (start_point_y + end_point_y) / 2;
    //         temp_line.poses.push_back(p);
    //     }
    //     else if(start_end_dis > (reference_dis * 4) && flag_choose_what == 0)
    //     {
    //         std::cout << "So far, Need To Use Curves !!" << std::endl;
    //         //第三个点与第一个点之间的斜率
    //         double k1 = (temp_line.poses[2].pose.position.y - start_point_y) / (temp_line.poses[2].pose.position.x - start_point_x);
    //         //最后一个点与倒数第三个点之间的斜率
    //         double k2 = (end_point_y - temp_line.poses[temp_line_size - 3].pose.position.y) / 
    //                     (end_point_x - temp_line.poses[temp_line_size - 3].pose.position.x);
            
    //         double a = 0.0, b = 0.0, c = 0.0, d = 0.0;
    //         LinearEquations4(start_point_x, end_point_x, start_point_y, end_point_y, k1, k2, a, b, c, d);//已知两点坐标以及斜率，使用三次多项式进行曲线拟合

    //         double xx = end_point_x;
    //         while(xx > start_point_x)
    //         {
    //             visiual_marker_array_CUBE(Temp_1,"world","r", xx, CurveFunction(a, b, c, d, xx), 0);
    //             xx -= 0.1;
    //         }
    //         visiual_marker_array_CUBE(Temp_1,"world","b", start_point_x, start_point_y, 0);
    //         visiual_marker_array_CUBE(Temp_1,"world","b", temp_line.poses[2].pose.position.x, temp_line.poses[2].pose.position.y, 0);
    //         visiual_marker_array_CUBE(Temp_1,"world","b", end_point_x, end_point_y, 0);
    //         visiual_marker_array_CUBE(Temp_1,"world","b", temp_line.poses[temp_line_size - 3].pose.position.x, temp_line.poses[temp_line_size - 3].pose.position.y, 0);

    //         std::cout << "a: " << a << ", b: " << b << ", c: " << c << ", d: " << d << ", k1: " << k1 << ", k2: " << k2 << std::endl;
    //         std::cout << "1: " << start_point_y - CurveFunction(a, b, c, d, start_point_x) << std::endl;
    //         std::cout << "2: " << end_point_y - CurveFunction(a, b, c, d, end_point_x) << std::endl;
    //         std::cout << "3: " << k1 - DiffFunction(a, b, c, d, start_point_x) << std::endl;
    //         std::cout << "4: " << k2 - DiffFunction(a, b, c, d, end_point_x) << std::endl;

    //         double delta_x = reference_dis / 10.0, x = end_point_x, y, x_last = end_point_x, y_last = end_point_y;
    //         double direction_x = (start_point_x - end_point_x) / abs(end_point_x - start_point_x);//计算方向
    //         std::cout << "delta_x: " << delta_x << ", direction_x: " << direction_x << std::endl;
    //         int add_point = 0;
    //         bool ok = 1;
    //         do
    //         {
    //             x += direction_x * delta_x;

    //             if(x == start_point_x)
    //             {
    //                 ok = 0;
    //             }
    //             else if(direction_x < 0)
    //             {
    //                 ok = x < start_point_x ? 0 : 1;
    //             }
    //             else if(direction_x > 0)
    //             {
    //                 ok = x > start_point_x ? 0 : 1;
    //             }
    //             if(!ok) {break;}

    //             y = CurveFunction(a, b, c, d, x); 
    //             double delta_xy = sqrt(pow(x_last - x,2) + pow(y_last - y,2));
    //             if(delta_xy >= reference_dis)
    //             {
    //                 p.pose.position.x = x;
    //                 p.pose.position.y = y;
    //                 temp_line.poses.push_back(p);
    //                 add_point++;
    //                 std::cout << "ADD: [" << x << ", " << y << "]" << "delta_xy: " << delta_xy << std::endl;

    //                 x_last = x;
    //                 y_last = y;
    //             }
                
    //         } while (1);
    //         std::cout << "ADD " << add_point-1 << " points in temp_line" << std::endl;
    //     }
    //     else if(start_end_dis > (reference_dis * 4) && flag_choose_what == 1)
    //     {
    //         std::cout << "So far, Need To Use VariableSlope !!" << std::endl;
    //         double k1 = (temp_line.poses[1].pose.position.y - start_point_y) / (temp_line.poses[1].pose.position.x - start_point_x);
    //         double k2 = (end_point_y - temp_line.poses[temp_line_size - 2].pose.position.y) / 
    //                     (end_point_x - temp_line.poses[temp_line_size - 2].pose.position.x);
    //         std::cout << "k1: " << k1 << ", k2:" << k2 << std::endl;

    //         int add_points_num = (int)(start_end_dis / (reference_dis * 1)) - 1;
    //         std::cout << "add_points_num: " << add_points_num << std::endl;
    //         double add_points[add_points_num][2] = {};

    //         double k_counts[add_points_num+2][2] = {};
    //         k_counts[0][0] = end_point_x - temp_line.poses[temp_line_size - 2].pose.position.x; //最后两个点x坐标之差
    //         k_counts[0][1] = end_point_y - temp_line.poses[temp_line_size - 2].pose.position.y; //最后两个点y坐标之差
    //         k_counts[add_points_num+1][0] = temp_line.poses[1].pose.position.x - start_point_x; //前面两个点x坐标之差
    //         k_counts[add_points_num+1][1] = temp_line.poses[1].pose.position.y - start_point_y; //前面两个点y坐标之差
            
    //         //顺时针插值
    //         if (end_point_x < start_point_x)
    //         {
    //             double temp_rate = change_angle_rate_;
    //             std::cout << "Change the end_point in clockwise: " << temp_rate << std::endl;
    //             double temp = pow(k_counts[0][0],2) + pow(k_counts[0][1],2);
    //             k_counts[0][0] = k_counts[0][0] * temp_rate;
    //             k_counts[0][1] = k_counts[0][1] / abs(k_counts[0][1]) * sqrt(temp - k_counts[0][0] * k_counts[0][0]);
    //         }
    //         //逆时针插值
    //         if (end_point_x > start_point_x)
    //         {
    //             double temp_rate = 2 - change_angle_rate_;
    //             std::cout << "Change the end_point in anticlockwise: " << temp_rate << std::endl;
    //             double temp = pow(k_counts[0][0],2) + pow(k_counts[0][1],2);
    //             k_counts[0][0] = k_counts[0][0] * temp_rate;
    //             k_counts[0][1] = k_counts[0][1] / abs(k_counts[0][1]) * sqrt(temp - k_counts[0][0] * k_counts[0][0]);
    //         }
            
    //         std::cout << "k_counts: [" << k_counts[0][0] << ", " << k_counts[0][1] << "], k: " << k_counts[0][1]/k_counts[0][0] << std::endl;
    //         double delta_x = k_counts[add_points_num+1][0] - k_counts[0][0], delta_y = k_counts[add_points_num+1][1] - k_counts[0][1];
    //         for (size_t i = 1; i < add_points_num+1; i++)
    //         {
    //             k_counts[i][0] = delta_x / (double)(add_points_num + 1) * i + k_counts[0][0];
    //             k_counts[i][1] = delta_y / (double)(add_points_num + 1) * i + k_counts[0][1];
    //             std::cout << "k_counts: [" << k_counts[i][0] << ", " << k_counts[i][1] << "], k: " << k_counts[i][1]/k_counts[i][0] << std::endl;
    //         }
    //         std::cout << "k_counts: [" << k_counts[add_points_num+1][0] << ", " << k_counts[add_points_num+1][1] << "], k: " << k_counts[add_points_num+1][1]/k_counts[add_points_num+1][0] << std::endl;
    //         VariableSlopeInterpolation(start_point_x, end_point_x, start_point_y, end_point_y, k_counts, add_points, add_points_num, reference_dis, line_choose_what);
            
    //         for (const auto & ap : add_points)
    //         {
    //             std::cout << "add_point: [" << ap[0] << ", " << ap[1] << "]" << std::endl;
    //             visiual_marker_array_CUBE(Temp_1,"world","r", ap[0], ap[1], 0);
    //         }
            
    //         for (size_t i = 0; i < add_points_num+2; i++)
    //         {
    //             if (i == 1)
    //             {
    //                 visiual_marker_array_ARROW(Temp_1,"world","b", end_point_x, end_point_y, 0, k_counts[i][0], k_counts[i][1], 0);
    //             }
    //             visiual_marker_array_ARROW(Temp_1,"world","b", add_points[i-1][0], add_points[i-1][1], 0, k_counts[i][0], k_counts[i][1], 0);
    //             if (i == add_points_num+1)
    //             {
    //                 visiual_marker_array_ARROW(Temp_1,"world","b", start_point_x, start_point_y, 0, k_counts[i][0], k_counts[i][1], 0);
    //             }
    //         }
    //     }
    // }




    // 直线场景用线性插值，非直线用三次样条
    void FrenetCoordinateHandle::deal_global_center_path()
    {
        // 1. 校验global_center_path有效性（至少2个点才需要闭合）
        if (global_center_path.poses.size() < 2) {
            ROS_WARN("deal_global_center_path: global_center_path has insufficient points (%lu), skip closing",
                    global_center_path.poses.size());
            return;
        }

        // 2. 获取关键参数：起点(P0)、终点(Pend)、坐标帧
        const auto& P0 = global_center_path.poses[0];          // 中心线起点
        const auto& Pend = global_center_path.poses.back();    // 中心线终点
        const std::string frame_id = global_center_path.header.frame_id;
        double x0 = P0.pose.position.x;
        double y0 = P0.pose.position.y;
        double x_end = Pend.pose.position.x;
        double y_end = Pend.pose.position.y;

        // 3. 计算起点-终点直线距离（距离过近无需插值）
        const double start_end_dis = hypot(x0 - x_end, y0 - y_end);
        const double close_threshold = 0.1; // 距离阈值：小于该值视为已闭合（可调整）
        if (start_end_dis < close_threshold) {
            ROS_INFO("deal_global_center_path: start-end distance (%.3fm) < threshold, already closed", start_end_dis);
            return;
        }

        // 4. 计算插值步长（自适应原路径密度）
        double total_path_dis = 0.0;
        for (size_t i = 1; i < global_center_path.poses.size(); ++i) {
            const auto& prev = global_center_path.poses[i-1].pose.position;
            const auto& curr = global_center_path.poses[i].pose.position;
            total_path_dis += hypot(curr.x - prev.x, curr.y - prev.y);
        }
        const double avg_step = total_path_dis / (global_center_path.poses.size() - 1); // 原路径平均步长
        const double interp_step = std::max(avg_step, 0.2); // 最小步长（避免点过密，可调整）
        const int interp_count = static_cast<int>(start_end_dis / interp_step); // 插值点数
        if (interp_count < 2) {
            ROS_WARN("deal_global_center_path: insufficient interpolation points (%d), skip", interp_count);
            return;
        }

        // 5. 计算关键方向角（用于判断是否为直线）
        // 5.1 起点切线方向角（P0→P0_next）
        const auto& P0_next = global_center_path.poses[1].pose.position;
        const double yaw_start = atan2(P0_next.y - y0, P0_next.x - x0);
        // 5.2 终点切线方向角（Pend_prev→Pend）
        const auto& Pend_prev = global_center_path.poses[global_center_path.poses.size() - 2].pose.position;
        const double yaw_end = atan2(y_end - Pend_prev.y, x_end - Pend_prev.x);
        // 5.3 起点-终点连线方向角（Pend→P0）
        const double yaw_line = atan2(y0 - y_end, x0 - x_end);

        // 6. 判断是否为直线场景（核心优化）
        const double angle_threshold = M_PI / 36; // 直线判断阈值：5度（转换为弧度，可调整）
        // 计算三个方向角的差值（取最小角度差，范围0~π）
        const double diff_start_line = fabs(remainder(yaw_start - yaw_line, 2 * M_PI));
        const double diff_end_line = fabs(remainder(yaw_end - yaw_line, 2 * M_PI));
        const bool is_straight = (diff_start_line < angle_threshold) && (diff_end_line < angle_threshold);

        // 7. 生成插值点（直线→线性插值，非直线→三次样条）
        std::vector<geometry_msgs::PoseStamped> interp_points;
        for (int i = 1; i < interp_count; ++i) { // 跳过Pend和P0本身
            const double t = static_cast<double>(i) / interp_count; // 插值参数（0→1：Pend→P0）
            geometry_msgs::PoseStamped interp_pose;
            interp_pose.header.frame_id = frame_id;
            interp_pose.header.stamp = ros::Time::now();
            interp_pose.pose.position.z = 0.0;

            if (is_straight) {
                // 7.1 直线场景：线性插值（保持两点间直线）
                interp_pose.pose.position.x = x_end + t * (x0 - x_end);
                interp_pose.pose.position.y = y_end + t * (y0 - y_end);
            } else {
                // 7.2 非直线场景：三次样条插值（平滑过渡）
                // 计算边界斜率（单位向量，保证方向一致）
                double k_end_x = cos(yaw_end);
                double k_end_y = sin(yaw_end);
                double k_start_x = cos(yaw_start);
                double k_start_y = sin(yaw_start);

                // Hermite三次样条插值公式（带一阶导数约束）
                const double t2 = t * t;
                const double t3 = t2 * t;
                const double a = 2*t3 - 3*t2 + 1;
                const double b = t3 - 2*t2 + t;
                const double c = -2*t3 + 3*t2;
                const double d = t3 - t2;
                const double h = start_end_dis; // 插值区间长度

                interp_pose.pose.position.x = a*x_end + b*h*k_end_x + c*x0 + d*h*k_start_x;
                interp_pose.pose.position.y = a*y_end + b*h*k_end_y + c*y0 + d*h*k_start_y;
            }

            interp_points.push_back(interp_pose);
        }

        // 8. 为插值点计算朝向（与原路径逻辑一致）
        for (size_t i = 0; i < interp_points.size(); ++i) {
            auto& curr_pose = interp_points[i];
            geometry_msgs::Point32 curr_pt, next_pt;
            curr_pt.x = curr_pose.pose.position.x;
            curr_pt.y = curr_pose.pose.position.y;

            // 最后一个插值点的下一个点是起点P0
            if (i == interp_points.size() - 1) {
                next_pt.x = x0;
                next_pt.y = y0;
            } else {
                next_pt.x = interp_points[i+1].pose.position.x;
                next_pt.y = interp_points[i+1].pose.position.y;
            }

            // 计算yaw角并设置四元数
            double yaw = atan2(next_pt.y - curr_pt.y, next_pt.x - curr_pt.x);
            curr_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
        }

        // 9. 将插值点插入原路径，完成闭合
        global_center_path.poses.insert(global_center_path.poses.end(), interp_points.begin(), interp_points.end());
        global_center_path.header.stamp = ros::Time::now();

        // 日志输出（明确当前插值方式）
        ROS_INFO("deal_global_center_path: closed successfully! "
                "Interpolation type: %s, Added %lu points, Distance: %.3fm, Step: %.3fm",
                is_straight ? "Linear" : "Cubic Spline",
                interp_points.size(), start_end_dis, interp_step);
    }
    
    void FrenetCoordinateHandle::global_center_path_TO_visiual_marker_array_CUBE()
    {
        for(const auto & g_c_p : global_center_path.poses)
        {
            visiual_marker_array_CUBE(marker_array_center_line,"world","g", g_c_p.pose.position.x, g_c_p.pose.position.y, 0);
        }
    }

    void FrenetCoordinateHandle::LinearEquations4(double x1, double x2, double y1, double y2, double k1, double k2, double & a, double & b, double & c, double & d)
    {
        const int num = 4;
        //增广矩阵
        double AugmentedMatrix[num][num+1] = {pow(x1,3), pow(x1,2), x1, 1, y1, 
                                              pow(x2,3), pow(x2,2), x2, 1, y2, 
                                              3 * pow(x1,2), 2 * x1, 1, 0, k1, 
                                              3 * pow(x2,2), 2 * x2, 1, 0, k2};

        double abcde[num] = {}, delta = 0.0;
        for (size_t i = 0; i < num; i++)
        {
            for (size_t j = i+1; j < num; j++)
            {
                double temp = AugmentedMatrix[j][i];
                for (size_t k = i; k < num+1; k++)
                {
                    if (k == i)
                    {
                        AugmentedMatrix[j][k] = 0.0;
                        continue;
                    }
                    
                    AugmentedMatrix[j][k] = AugmentedMatrix[j][k] * AugmentedMatrix[i][i] / temp - AugmentedMatrix[i][k];
                }
            }
        }

        //顶不住了，一条一条写吧
        abcde[3] = AugmentedMatrix[3][4] / AugmentedMatrix[3][3];
        abcde[2] = (AugmentedMatrix[2][4] - AugmentedMatrix[2][3] * abcde[3]) / AugmentedMatrix[2][2];
        abcde[1] = (AugmentedMatrix[1][4] - AugmentedMatrix[1][3] * abcde[3] - AugmentedMatrix[1][2] * abcde[2]) / AugmentedMatrix[1][1];
        abcde[0] = (AugmentedMatrix[0][4] - AugmentedMatrix[0][3] * abcde[3] - AugmentedMatrix[0][2] * abcde[2] - AugmentedMatrix[0][1] * abcde[1]) / AugmentedMatrix[0][0];
        
        //a b c d 为系数
        a = abcde[0];
        b = abcde[1];
        c = abcde[2];
        d = abcde[3];

    }

    void FrenetCoordinateHandle::VariableSlopeInterpolation(double x1, double x2, double y1, double y2, double k_counts[][2], double add_points[][2], int add_points_num, double reference_dis, int line_choose_what)
    {
        nav_msgs::Path& temp_line = (line_choose_what == 0) ? gps_path_filtered : global_center_path;
        
        double temp_x = x2, temp_y = y2;

        for(size_t i = 0; i < add_points_num; i++)
        {
            double cos_theda = (k_counts[i][0] * (x1 - x2) + k_counts[i][1] * (y1 - y2)) / 
                               (sqrt(pow(k_counts[i][0],2) + pow(k_counts[i][1],2)) * sqrt(pow((x1 - x2),2) + pow((y1 - y2),2)));
            double radian = change_reference_rate_ * reference_dis / cos_theda;
            std::cout << "cos_theda: " << cos_theda << ", radian: " << radian << std::endl;

            add_points[i][0] = temp_x + k_counts[i][0] / sqrt(k_counts[i][0] * k_counts[i][0] + k_counts[i][1] * k_counts[i][1]) * radian;
            add_points[i][1] = temp_y + k_counts[i][1] / sqrt(k_counts[i][0] * k_counts[i][0] + k_counts[i][1] * k_counts[i][1]) * radian;
            
            geometry_msgs::PoseStamped p;
            p.pose.position.x = add_points[i][0];
            p.pose.position.y = add_points[i][1];
            temp_line.poses.push_back(p);

            temp_x = add_points[i][0];
            temp_y = add_points[i][1];
        }
    }

    double FrenetCoordinateHandle::CurveFunction(double a, double b, double c, double d, double x)
    {
        double result = a * pow(x,3) + b * pow(x,2) + c * x + d;
        return result;
    }

    double FrenetCoordinateHandle::DiffFunction(double a, double b, double c, double d, double x)
    {
        double result = 3 * a * pow(x,2) + 2 * b * x + c;
        return result;
    }

    // 从文件加载锥桶地图
    void FrenetCoordinateHandle::loadConeMapFromFile()
    {
        // 构建锥桶文件完整路径
        std::string cone_file_path = center_line_save_path_;
        ROS_ERROR_STREAM("center_line_save_path_: " << center_line_save_path_);

        if (!cone_file_path.empty() && cone_file_path.back() != '/') {
            cone_file_path += "/";
        }
        cone_file_path += "cone_map.txt";

        std::ifstream file(cone_file_path);

        if (!file.is_open()) {
            ROS_ERROR_STREAM("Failed to open cone map file: " << cone_file_path);
            return;
        }

        std::string line;
        double x, y;
        char color;
        int red_count = 0, blue_count = 0;

        while (std::getline(file, line)) {
            // 跳过空行
            if (line.empty()) continue;

            std::istringstream iss(line);
            if (!(iss >> x >> y >> color)) {
                ROS_WARN_STREAM("Invalid format in cone file: " << line);
                continue;
            }

            // 构建锥桶数据
            fsd_common_msgs::Cone cone;
            cone.position.x = x;
            cone.position.y = y;
            cone.position.z = 0.0;

            // 添加到全局无序坐标列表
            geometry_msgs::Point32 point;
            point.x = x;
            point.y = y;
            cone_global_disorder_coordinate.polygon.points.push_back(point);

            // 按颜色分类
            if (color == 'r') {
                map_from_slam_left.cone_detections.push_back(cone);
                red_count++;
            } else if (color == 'b') {
                map_from_slam_right.cone_detections.push_back(cone);
                blue_count++;
            } else {
                ROS_WARN_STREAM("Unknown cone color '" << color << "' at (" << x << "," << y << ")");
            }

            // 添加到总地图
            map_from_slam.cone_detections.push_back(cone);
        }

        file.close();
        // ROS_INFO_STREAM("Loaded cone map from file: " << red_count << " red, " << blue_count << " blue cones");

        // 设置锥桶数据就绪标志
        if (red_count > 0 && blue_count > 0) {
            get_cone = true;
        } else {
            ROS_WARN("No valid red/blue cones loaded from file");
        }
    }

    // 从文件加载GPS轨迹
    void FrenetCoordinateHandle::loadGpsMapFromFile()
    {
        // 构建轨迹文件完整路径
        std::string gps_file_path = center_line_save_path_;
        if (!gps_file_path.empty() && gps_file_path.back() != '/') {
            gps_file_path += "/";
        }
        gps_file_path += "gps_map.txt";

        std::ifstream file(gps_file_path);
        if (!file.is_open()) {
            ROS_ERROR_STREAM("Failed to open GPS map file: " << gps_file_path);
            return;
        }

        gps_path_filtered.header.frame_id = "world";
        gps_path_filtered.poses.clear();
        odom_dis = 0.0;
        flag_gps_num = 0;

        std::string line;
        double x, y;
        geometry_msgs::PoseStamped pose;

        while (std::getline(file, line)) {
            // 跳过空行
            if (line.empty()) continue;

            std::istringstream iss(line);
            if (!(iss >> x >> y)) { // 只读取前两列
                ROS_WARN_STREAM("Invalid format in GPS file: " << line);
                continue;
            }

            if (flag_gps_num == 0) {
                // 初始化第一个点
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                gps_path_filtered.poses.push_back(pose);
                flag_gps_num = 1;
            } else {
                // 计算与上一点的距离
                auto& last_pose = gps_path_filtered.poses.back().pose.position;
                double delta_dis = hypot(x - last_pose.x, y - last_pose.y);

                // 过滤过近的点（保持与原有逻辑一致）
                if (delta_dis >= record_dis_) {
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    gps_path_filtered.poses.push_back(pose);
                    odom_dis += delta_dis;
                }
            }
        }

        file.close();
        int path_size = gps_path_filtered.poses.size();
        // ROS_INFO_STREAM("Loaded GPS path from file: " << path_size << " points, total distance: " << odom_dis);

        if (path_size >= 2) {
            // 处理轨迹朝向（与原有逻辑一致）
            gps_center_path = gps_path_filtered;
            for (size_t i = 0; i < gps_center_path.poses.size() - 1; ++i) {
                double x1 = gps_center_path.poses[i].pose.position.x;
                double y1 = gps_center_path.poses[i].pose.position.y;
                double x2 = gps_center_path.poses[i+1].pose.position.x;
                double y2 = gps_center_path.poses[i+1].pose.position.y;
                double yaw = atan2(y2 - y1, x2 - x1);
                gps_center_path.poses[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
            }
            // 最后一个点沿用前一个点的朝向
            gps_center_path.poses.back().pose.orientation = gps_center_path.poses[path_size - 2].pose.orientation;

            // 设置轨迹数据就绪标志
            get_gps = true;
        } else {
            ROS_WARN("Insufficient GPS points (need at least 2)");
        }
    }
    
    // 保存左右Frenet锥桶和auto_color_map到文件
    void FrenetCoordinateHandle::saveFrenetConesAndColorMap()
    {
        // 保存左侧Frenet锥桶
        std::ofstream left_file(center_line_save_path_ + "left_frenet_cones.txt", std::ofstream::out | std::ofstream::trunc);
        if (left_file.is_open())
        {
            int index = 1;
            for (const auto& cone : left_cone_frenet)
            {
                left_file << std::fixed << std::setprecision(4)
                        << index << " "                  // 序号(行号)
                        << cone.first << " "             // s值
                        << cone.second.d << " "          // d值
                        << cone.second.rx << " "          // x坐标
                        << cone.second.ry << std::endl;   // y坐标
                index++;
            }
            left_file.close();
            ROS_INFO("Left Frenet cones saved successfully");
        }
        else
        {
            ROS_ERROR("Failed to open left_frenet_cones.txt for writing");
        }

        // 保存右侧Frenet锥桶
        std::ofstream right_file(center_line_save_path_ + "right_frenet_cones.txt", std::ofstream::out | std::ofstream::trunc);
        if (right_file.is_open())
        {
            int index = 1;
            for (const auto& cone : right_cone_frenet)
            {
                right_file << std::fixed << std::setprecision(4)
                        << index << " "                  // 序号(行号)
                        << cone.first << " "             // s值
                        << cone.second.d << " "          // d值
                        << cone.second.rx << " "          // x坐标
                        << cone.second.ry << std::endl;   // y坐标
                index++;
            }
            right_file.close();
            ROS_INFO("Right Frenet cones saved successfully");
        }
        else
        {
            ROS_ERROR("Failed to open right_frenet_cones.txt for writing");
        }

        // 保存auto_color_map（包含序号信息）
        std::ofstream color_file(center_line_save_path_ + "auto_color_map.txt", std::ofstream::out | std::ofstream::trunc);
        if (color_file.is_open())
        {
            int left_index = 1, right_index = 1;
            // 保存左侧锥桶颜色信息
            for (const auto& cone : left_cone_frenet)
            {
                color_file << std::fixed << std::setprecision(4)
                        << "left " << left_index << " "  // 类型和序号
                        << cone.second.rx << " "          // x坐标
                        << cone.second.ry << " "          // y坐标
                        << "r" << std::endl;             // 颜色
                left_index++;
            }
            // 保存右侧锥桶颜色信息
            for (const auto& cone : right_cone_frenet)
            {
                color_file << std::fixed << std::setprecision(4)
                        << "right " << right_index << " " // 类型和序号
                        << cone.second.rx << " "          // x坐标
                        << cone.second.ry << " "          // y坐标
                        << "b" << std::endl;             // 颜色
                right_index++;
            }
            color_file.close();
            ROS_INFO("auto_color_map saved successfully");
        }
        else
        {
            ROS_ERROR("Failed to open auto_color_map.txt for writing");
        }
    }

    // 从文件加载Frenet锥桶
    bool FrenetCoordinateHandle::loadFrenetConesFromFile()
    {
        left_cone_frenet.clear();
        right_cone_frenet.clear();
        auto_color_map_.markers.clear();
        
        // 加载左侧锥桶
        std::ifstream left_file(center_line_save_path_ + "left_frenet_cones.txt");
        if (!left_file.is_open())
        {
            ROS_ERROR("Failed to open left_frenet_cones.txt for reading");
            return false;
        }
        
        int index;
        double s, d, x, y;
        while (left_file >> index >> s >> d >> x >> y)
        {
            left_cone_frenet.insert(std::pair<double, frenetcone>(s, frenetcone(x, y, d)));
            visiual_marker_array_CUBE(auto_color_map_, frame_id_, "r", x, y, 0);
            
            // 添加序号标记
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = frame_id_;
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "cone_labels";
            text_marker.id = auto_color_map_.markers.size() + 1;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position.x = x + 0.1;
            text_marker.pose.position.y = y + 0.1;
            text_marker.pose.position.z = 0.2;
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.15;
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0;
            text_marker.text = std::to_string(index);
            auto_color_map_.markers.push_back(text_marker);
        }
        left_file.close();
        
        // 加载右侧锥桶
        std::ifstream right_file(center_line_save_path_ + "right_frenet_cones.txt");
        if (!right_file.is_open())
        {
            ROS_ERROR("Failed to open right_frenet_cones.txt for reading");
            return false;
        }
        
        while (right_file >> index >> s >> d >> x >> y)
        {
            right_cone_frenet.insert(std::pair<double, frenetcone>(s, frenetcone(x, y, d)));
            visiual_marker_array_CUBE(auto_color_map_, frame_id_, "b", x, y, 0);
            
            // 添加序号标记
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = frame_id_;
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "cone_labels";
            text_marker.id = auto_color_map_.markers.size() + 1;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position.x = x + 0.1;
            text_marker.pose.position.y = y + 0.1;
            text_marker.pose.position.z = 0.2;
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.15;
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0;
            text_marker.text = std::to_string(index);
            auto_color_map_.markers.push_back(text_marker);
        }
        right_file.close();
        
        ROS_INFO("Loaded Frenet cones from files successfully");
        return true;
    }
}

    

