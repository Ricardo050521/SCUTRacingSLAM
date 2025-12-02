#include "mapping.h"

using namespace racingslam;

Mapping::Mapping(std::string ns)
{
    node_ = ros::NodeHandle(ns);
    delta_dis_ = 0;
    odom_ = 0;
    loop_detection_timer_ = -1;
    cone_run_count_ = 0;
    res_state_ = 0;
    track_fix_start_time_ = 0;
    skid_start_time_ = 0;
    is_state_init_ = false;
    is_track_line_init_ = false;
    is_loop_closed_ = false;
    is_track_fix_ = false;
    is_acc_end_ = false;
    is_skid_start_ = false;
    is_skid_end_ = false;
    is_map_saved_ = false;
    is_map_load_ = false;
    is_center_load_ = false;
    is_acc_track_fit = false;
    is_skid_dealt = false;
}

/***
 * @brief 该函数用于加载参数服务器的参数
*/
void Mapping::loadParam()
{
    if(!node_.param("mapping/odom_track_switch", odom_track_switch_, false))
    {
        ROS_WARN_STREAM("Did not load mapping/odom_track_switch. Standard value is: " << odom_track_switch_);
    }
    if(!node_.param("mapping/map_load_switch", map_load_switch_, false))
    {
        ROS_WARN_STREAM("Did not load mapping/map_load_switch. Standard value is: " << map_load_switch_);
    }
    if(!node_.param("mapping/center_load_switch", center_load_switch_, false))
    {
        ROS_WARN_STREAM("Did not load mapping/center_load_switch. Standard value is: " << center_load_switch_);
    }
    if(!node_.param<std::string>("mapping/res_sub_topic", res_sub_topic_, "/res_and_ami"))
    {
        ROS_WARN_STREAM("Did not load mapping/res_sub_topic. Standard value is: " << res_sub_topic_);
    }
    if (!node_.param<std::string>("mapping/cone_sub_topic", cone_sub_topic_, "/perception/lidar/cone_side")) 
    {
        ROS_WARN_STREAM("Did not load mapping/cone_sub_topic. Standard value is: " << cone_sub_topic_);
    }
    if(!node_.param<std::string>("mapping/state_sub_topic", state_sub_topic_, "/estimation/slam/state"))
    {
        ROS_WARN_STREAM("Did not load mapping/state_sub_topic. Standard value is: " << state_sub_topic_);
    }
    if(!node_.param<std::string>("odometry/odom_track_topic", track_sub_topic_, "/racingslam/odom_track"))
    {
        ROS_WARN_STREAM("Did not load odometry/odom_track_topic. Standard value is: " << track_sub_topic_);
    }
    if(!node_.param<std::string>("mapping/map_pub_topic", map_pub_topic_, "/estimation/slam/map"))
    {
        ROS_WARN_STREAM("Did not load mapping/map_pub_topic. Standard value is: " << map_pub_topic_);
    }
    if(!node_.param<std::string>("mapping/gps_line_pub_topic", gps_line_pub_topic_, "/estimation/slam/gps_line"))
    {
        ROS_WARN_STREAM("Did not load mapping/gps_line_pub_topic. Standard value is: " << gps_line_pub_topic_);
    }
    if(!node_.param<std::string>("mapping/global_gps_line_pub_topic", global_gps_line_pub_topic_, "/estimation/slam/global_gps_line"))
    {
        ROS_WARN_STREAM("Did not load mapping/global_gps_line_pub_topic. Standard value is: " << global_gps_line_pub_topic_);
    }
    if(!node_.param<std::string>("mapping/fix_acc_track_pub_topic", fix_acc_track_pub_topic_, "/estimation/slam/fix_acc_track"))
    {
        ROS_WARN_STREAM("Did not load mapping/fix_acc_track_pub_topic. Standard value is: " << fix_acc_track_pub_topic_);
    }
    if(!node_.param<std::string>("mapping/map_visualization_pub_topic", map_visualization_pub_topic_, "/visualization/slam/map"))
    {
        ROS_WARN_STREAM("Did not load mapping/map_visualization_pub_topic. Standard value is: " << map_visualization_pub_topic_);
    }
    if(!node_.param<std::string>("mapping/track_visualization_pub_topic", track_visualization_pub_topic_, "/visualization/slam/track"))
    {
        ROS_WARN_STREAM("Did not load mapping/track_visualization_pub_topic. Stand value is: " << track_visualization_pub_topic_);
    }
    if(!node_.param<std::string>("mapping/current_visual_cone_detections_pub_topic", current_visual_cone_detections_pub_topic_, "/visualization/slam/cone_detections"))
    {
        ROS_WARN_STREAM("Did not load mapping/current_visual_cone_detections_pub_topic. Stand value is: " << current_visual_cone_detections_pub_topic_);
    }
    if(!node_.param<std::string>("mapping/realtime_map_pub_topic", realtime_map_pub_topic_, "/estimation/slam/realtime_map"))
    {
        ROS_WARN_STREAM("Did not load mapping/realtime_map_pub_topic. Standard value is: " << realtime_map_pub_topic_);
    }
    if(!node_.param<std::string>("mapping/skid_path_pub_topic", skid_path_pub_topic_, "/control/skidpad"))
    {
        ROS_WARN_STREAM("Did not load mapping/skid_path_pub_topic. Standard value is: " << skid_path_pub_topic_);
    }
    if(!node_.param<std::string>("mapping/map_save_path", map_save_path_, "/home/ricardo/map/"))
    {
        ROS_WARN_STREAM("Did not load mapping/map_save_path. Standard value is: " << map_save_path_);
    }
    if(!node_.param("mapping/reg_range", reg_range_, 102.25))
    {
        ROS_WARN_STREAM("Did not load mapping/reg_range. Standard value is: " << reg_range_);
    }
    if(!node_.param("mapping/min_update_count", min_update_count_, 3))
    {
        ROS_WARN_STREAM("Did not load mapping/min_update_count. Standard value is: " << min_update_count_);
    }
    if(!node_.param("mapping/max_reg_dis", max_reg_dis_, 1.5))
    {
        ROS_WARN_STREAM("Did not load mapping/max_reg_dis. Standard valus is: " << max_reg_dis_);
    }
    if(!node_.param("mapping/min_cone_distance", min_cone_distance_, 1.5))
    {
        ROS_WARN_STREAM("Did noe load mapping/min_cone_distance. Standard value is: " << min_cone_distance_);
    }
    if(!node_.param("mapping/loop_detection_timer_count", loop_detection_timer_count_, 10))
    {
        ROS_WARN_STREAM("Did not load mapping/loop_detection_time_count. Standard value is: " << loop_detection_timer_count_);
    }
    if(!node_.param("mapping/loop_detection_dis", loop_detection_dis_, 2.0))
    {
        ROS_WARN_STREAM("Did not load mapping/loop_detection_dis. Stand value is: " << loop_detection_dis_);
    }
    if(!node_.param("mapping/acc_distance", acc_distance_, 80.0))
    {
        ROS_WARN_STREAM("Did not load mapping/acc_distance. Stand value is: " << acc_distance_);
    }
    if(!node_.param("mapping/fix_acc_track_length", fit_acc_track_length_, 80.0))
    {
        ROS_WARN_STREAM("Did not load mapping/fix_acc_track_length. Stand value is: " << fit_acc_track_length_);
    }
    if(!node_.param<std::string>("mapping/visualization_frame_id", frame_id_, "world"))
    {
        ROS_WARN_STREAM("Did not load mapping/visualization_frame_id. Standard value is: " << frame_id_);
    }
    if(!node_.param("mapping/lidar2cog_length", lidar2cog_length_, 1.79))
    {
        ROS_WARN_STREAM("Did not load mapping/lidar2cog_length. Stand value is: " << lidar2cog_length_);
    }
    if(!node_.param<std::string>("mapping/running_mode", running_mode_, "track"))
    {
        ROS_WARN_STREAM("Did not load mapping/running_mode. Standard value is: " << running_mode_);
    }

    if(!node_.param("mapping/dis_to_erase", dis_to_erase_, 3.0))
    {
        ROS_WARN_STREAM("Did not load mapping/dis_to_erase. Standard value is: " << dis_to_erase_);
    }
    if(!node_.param("mapping/time_match_tolerance", time_match_tolerance_, 0.02))
    {
        ROS_WARN_STREAM("Did not load mapping/time_match_tolerance. Standard value is: " << time_match_tolerance_);
    }
    if(!node_.param("mapping/max_cache_frames", max_cache_frames_, 100))
    {
        ROS_WARN_STREAM("Did not load mapping/max_cache_frames. Standard value is: " << max_cache_frames_);
    }
}

/***
 * @brief 该函数用于接收与注册话题
*/
void Mapping::regTopic()
{
    cone_sub_ = node_.subscribe<fsd_common_msgs::ConeDetections>(cone_sub_topic_, 1, &Mapping::coneCallback, this);
    radar_sub_ = node_.subscribe<fsd_common_msgs::ConeDetections>("/perception/lidar/cone_detections", 10, &Mapping::radarCallback, this);
    state_sub_ = node_.subscribe(state_sub_topic_, 1, &Mapping::stateCallback, this);
    //res_sub_ = node_.subscribe(res_sub_topic_, 1, &Mapping::resCallback, this);
    if(odom_track_switch_) track_sub_ = node_.subscribe<racingslam::CarTrack>(track_sub_topic_, 1, &Mapping::trackCallback, this); //开关控制是否使用里程计
    
    map_pub_ = node_.advertise<fsd_common_msgs::Map>(map_pub_topic_, 10, true); //单次全局地图
    realtime_map_pub_ = node_.advertise<fsd_common_msgs::ConeDetections>(realtime_map_pub_topic_, 10, true); //实时全局地图
    gps_line_pub_ = node_.advertise<nav_msgs::Path>(gps_line_pub_topic_, 10, true);
    global_gps_line_pub_ = node_.advertise<nav_msgs::Path>(global_gps_line_pub_topic_, 10, true);
    fix_acc_track_pub_ = node_.advertise<nav_msgs::Path>(fix_acc_track_pub_topic_, 10, true);
    skid_path_pub_ = node_.advertise<fsd_common_msgs::SkidpadGlobalCenterLine>(skid_path_pub_topic_, 10, true);

    map_visualization_pub_ = node_.advertise<visualization_msgs::MarkerArray>(map_visualization_pub_topic_, 10, true);//全局地图可视化
    current_visual_cone_detections_pub_ = node_.advertise<visualization_msgs::MarkerArray>(current_visual_cone_detections_pub_topic_, 10, true);//当前椎桶可视化
    realtime_map_visualization_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/visualization/slam/newrealtime_map", 10, true);//实时地图可视化（局部坐标系）
}

/**
 * @brief 基于Eigen库的三次样条插值实现
 * @param x 原始轨迹点x坐标数组
 * @param y 原始轨迹点y坐标数组
 * @param num_points 插值后需要的总点数
 * @return 平滑插值后的点集
 */
std::vector<geometry_msgs::Point> Mapping::cubicSplineInterpolation(const std::vector<double>& x, const std::vector<double>& y, int num_points) {
    std::vector<geometry_msgs::Point> result;
    int n = x.size();
    if (n < 2) return result;

    // 1. 计算自然参数t（累计距离）
    Eigen::VectorXd t(n);  // 显式使用Eigen::前缀
    t[0] = 0.0;
    for (int i = 1; i < n; ++i) {
        double dx = x[i] - x[i-1];
        double dy = y[i] - y[i-1];
        t[i] = t[i-1] + sqrt(dx*dx + dy*dy);
    }
    double total_length = t[n-1];
    if (total_length < 1e-6) return result;

    // 2. 计算相邻点距离h
    Eigen::VectorXd h = t.tail(n-1) - t.head(n-1);

    // 3. 样条系数计算函数（x和y方向通用）
    auto calculateSplineCoeffs = [&](const Eigen::VectorXd& data) -> std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);  // 显式命名空间
        Eigen::VectorXd B = Eigen::VectorXd::Zero(n);

        // 填充内部点
        for (int i = 1; i < n-1; ++i) {
            A(i, i-1) = h[i-1];
            A(i, i) = 2 * (h[i-1] + h[i]);
            A(i, i+1) = h[i];
            B(i) = 3 * ((data[i+1] - data[i])/h[i] - (data[i] - data[i-1])/h[i-1]);
        }

        // 自然边界条件
        A(0, 0) = 1.0;
        A(n-1, n-1) = 1.0;
        B(0) = 0.0;
        B(n-1) = 0.0;

        // 求解线性方程组（显式调用LU分解）
        Eigen::VectorXd c = A.lu().solve(B);

        // 计算b和d系数
        Eigen::VectorXd b(n-1), d(n-1);
        for (int i = 0; i < n-1; ++i) {
            b[i] = (data[i+1] - data[i])/h[i] - h[i]*(c[i+1] + 2*c[i])/3;
            d[i] = (c[i+1] - c[i])/(3*h[i]);
        }

        return {b, c, d};
    };

    // 4. 转换数据并计算系数（显式使用Eigen::Map）
    Eigen::VectorXd x_data = Eigen::Map<const Eigen::VectorXd>(x.data(), n);
    Eigen::VectorXd y_data = Eigen::Map<const Eigen::VectorXd>(y.data(), n);
    auto [b_x, c_x, d_x] = calculateSplineCoeffs(x_data);
    auto [b_y, c_y, d_y] = calculateSplineCoeffs(y_data);

    // 5. 生成插值点
    double step = total_length / (num_points - 1);
    for (int k = 0; k < num_points; ++k) {
        double t_val = k * step;

        int i = 0;
        while (i < n-1 && t[i+1] < t_val) ++i;
        double dt = t_val - t[i];

        // 计算插值坐标
        double x_interp = x_data[i] + b_x[i]*dt + c_x[i]*dt*dt + d_x[i]*dt*dt*dt;
        double y_interp = y_data[i] + b_y[i]*dt + c_y[i]*dt*dt + d_y[i]*dt*dt*dt;

        geometry_msgs::Point p;
        p.x = x_interp;
        p.y = y_interp;
        p.z = 0.0;
        result.push_back(p);
    }

    return result;
}



void Mapping::resCallback(const fsd_common_msgs::ResAndAmi& msg)
{
    res_state_ = msg.resState;
}

void Mapping::radarCallback(const fsd_common_msgs::ConeDetections::ConstPtr& radar_msg)
{
    if(map_load_switch_) return;

    latest_radar_stamp_ = radar_msg->header.stamp;
    ROS_INFO("Updated latest radar timestamp: %.6f", latest_radar_stamp_.toSec());

}

/***
 * @brief 椎桶的回调函数
*/
void Mapping::coneCallback(const fsd_common_msgs::ConeDetections::ConstPtr& msg)
{   
    //若使用地图加载，则不使用原椎桶回调函数
    if(map_load_switch_)
    {
        std::cout << "test cone" << std::endl;
        return;
    }
    
    cone_run_count_++;

    fsd_common_msgs::ConeDetections temp;
    temp.cone_detections = msg->cone_detections;
    temp.header.stamp = latest_radar_stamp_;
    ROS_INFO("=====================stamp================================= %.2f", latest_radar_stamp_.toSec());
    temp.header.frame_id = msg->header.frame_id;
    // temp.header = msg->header;
    cone_buffer_.push_back(temp);
}

// 根据目标时间戳，在映射表中查找误差范围内最接近的pose_A
bool Mapping::findMatchedPose(double target_time, fsd_common_msgs::CarState& out_pose) {

    if (cone_time_2pose_map_.empty()) {
        ROS_WARN("cone_time_2pose_map_ is empty, no poses to match");
        return false;
    }

    // 找到第一个 >= target_time的迭代器
    auto it = cone_time_2pose_map_.lower_bound(target_time);
    double min_diff = time_match_tolerance_ + 0.001;
    bool found = false;
    
    // 检查当前迭代器和前一个迭代器
    if (it != cone_time_2pose_map_.end()) {
        double diff = fabs(it->first - target_time);
        if (diff <= time_match_tolerance_ && diff < min_diff) {
            min_diff = diff;
            out_pose = it->second;
            found = true;
        }
    }
    if (it != cone_time_2pose_map_.begin()) {
        --it;
        double diff = fabs(it->first - target_time);
        if (diff <= time_match_tolerance_ && diff < min_diff) {
            min_diff = diff;
            out_pose = it->second;
            found = true;
        }
    }

    // 日志输出
    if (found) ROS_INFO("Matched pose found: target_time=%.6f, min_diff=%.6f", target_time, min_diff);
    else ROS_ERROR("No matched pose! target_time=%.6f (tolerance=%.6f)", target_time, time_match_tolerance_);

    return found;
}

/***
 * @brief 该函数是车辆位置的回调函数，用于处理接受到的车辆的位姿信息
*/
void Mapping::stateCallback(const fsd_common_msgs::CarState &msg)
{    
    //若使用地图加载，则不运行该回调函数
    if(map_load_switch_)
    {
        return;
    }

    //以下代码用于记录轨迹

    //未完成回环记录点坐标
    if(!odom_track_switch_ && !is_loop_closed_ && !is_acc_end_ && !is_skid_end_) //任意一项完成停止记录
    {
        geometry_msgs::PoseStamped temp_pose;
        double delta_dis = 0.0;
        if(!is_track_line_init_)
        {
            track_line_.header.frame_id = frame_id_;

            temp_pose.header = msg.header;
            temp_pose.pose.position.x = msg.car_state.x;
            temp_pose.pose.position.y = msg.car_state.y;
            temp_pose.pose.orientation.z = msg.car_state.theta;
            track_line_.poses.push_back(temp_pose);
            gps_line_pub_.publish(track_line_);
            
            is_track_line_init_ = true;

            return;
        }

        geometry_msgs::PoseStamped back_pose;
        back_pose = track_line_.poses.back();
        // ROS_INFO("stamp =========================================== %.2f", back_pose.header.stamp);
        delta_dis = sqrt(pow(msg.car_state.x - back_pose.pose.position.x, 2) + pow(msg.car_state.y - back_pose.pose.position.y, 2)); //计算当前点与上一点之间的位移，若过小则不计入track_line_
        
        //控制要求点之间的间隔0.1左右，对不满足的点进行差值处理
        if(is_track_line_init_ && delta_dis > 0.1)
        {
            if(delta_dis > 0.15)
            {
                double delta_time = msg.header.stamp.toSec() - back_pose.header.stamp.toSec();
                double delta_x = msg.car_state.x - back_pose.pose.position.x;
                double delta_y = msg.car_state.y - back_pose.pose.position.y;
                int add_poses_numbers;
                add_poses_numbers = (int)(delta_dis / 0.1);

                for(int i = 1; i < add_poses_numbers + 1; i++)
                {
                    geometry_msgs::PoseStamped add_pose;
                    add_pose.header.stamp = ros::Time(back_pose.header.stamp.toSec() + (delta_time * (i) / (add_poses_numbers + 1)));
                    add_pose.pose.position.x = back_pose.pose.position.x + (delta_x * (i) / (add_poses_numbers + 1));
                    add_pose.pose.position.y = back_pose.pose.position.y + (delta_y * (i) / (add_poses_numbers + 1));
                    add_pose.pose.orientation.z = back_pose.pose.orientation.z; //角度暂不差值，如需差值需要复杂的判读条件
                    track_line_.poses.push_back(add_pose);
                }
            }

            temp_pose.header = msg.header;
            temp_pose.pose.position.x = msg.car_state.x;
            temp_pose.pose.position.y = msg.car_state.y;
            temp_pose.pose.orientation.z = msg.car_state.theta;
            track_line_.poses.push_back(temp_pose);
            gps_line_pub_.publish(track_line_);
        }

    }
    // 已完成回环补偿空缺点
    if(!odom_track_switch_ && is_loop_closed_)
    {
        geometry_msgs::PoseStamped start_pose = track_line_.poses.front();
        geometry_msgs::PoseStamped end_pose = track_line_.poses.back();
        double delta_dis = sqrt(pow(end_pose.pose.position.x - start_pose.pose.position.x, 2) + 
                            pow(end_pose.pose.position.y - start_pose.pose.position.y, 2));

        // 准备用于三次样条插值的原始点
        std::vector<double> x, y;
        x.push_back(end_pose.pose.position.x);
        y.push_back(end_pose.pose.position.y);
        x.push_back(start_pose.pose.position.x);
        y.push_back(start_pose.pose.position.y);

        // 计算需要的插值点数（0.1）
        int add_poses_numbers = (int)(delta_dis / 0.1);
        int num_points = add_poses_numbers + 1;  // 包含起点和终点

        // 使用三次样条插值生成平滑路径
        std::vector<geometry_msgs::Point> spline_points = cubicSplineInterpolation(x, y, num_points);

        // 将插值点添加到轨迹中（跳过第一个点，避免与end_pose重复）
        for(size_t i = 1; i < spline_points.size(); ++i)
        {
            geometry_msgs::PoseStamped add_pose;
            add_pose.pose.position = spline_points[i];
            add_pose.pose.orientation.z = end_pose.pose.orientation.z;  // 角度暂不插值
            track_line_.poses.push_back(add_pose);
        }

        is_track_fix_ = true;
    }
    // // 已完成回环补偿空缺点
    // if(!odom_track_switch_ && is_loop_closed_)
    // {
    //     geometry_msgs::PoseStamped start_pose;
    //     geometry_msgs::PoseStamped end_pose;
    //     double delta_dis;
    //     start_pose = track_line_.poses.front();
    //     end_pose = track_line_.poses.back();
    //     delta_dis = sqrt(pow(end_pose.pose.position.x - start_pose.pose.position.x, 2) + 
    //                      pow(end_pose.pose.position.y - start_pose.pose.position.y, 2));

    //     double delta_x = start_pose.pose.position.x - end_pose.pose.position.x;
    //     double delta_y = start_pose.pose.position.y - end_pose.pose.position.y;
    //     int add_poses_numbers;
    //     add_poses_numbers = (int)(delta_dis / 0.1);

    //     for(int i = 1; i < add_poses_numbers + 1; i++)
    //     {
    //         geometry_msgs::PoseStamped add_pose;
            
    //         add_pose.pose.position.x = end_pose.pose.position.x + (delta_x * (i) / (add_poses_numbers + 1));
    //         add_pose.pose.position.y = end_pose.pose.position.y + (delta_y * (i) / (add_poses_numbers + 1));
    //         add_pose.pose.orientation.z = end_pose.pose.orientation.z;  //角度暂不差值，如需差值需要复杂的判读条件
    //         track_line_.poses.push_back(add_pose);
    //     }

    //     is_track_fix_ = true;
    // }

    ROS_INFO("last car track x: %f, y: %f", (track_line_.poses.end() - 1) -> pose.position.x, (track_line_.poses.end() - 1) -> pose.position.y);
    
    //以下代码用于回环检测

    if(!is_state_init_)
    {
        init_state_.car_state.x     = msg.car_state.x + 1.0*cos(msg.car_state.theta);
        init_state_.car_state.y     = msg.car_state.y + 1.0*sin(msg.car_state.theta);
        init_state_.car_state.theta = 0 * msg.car_state.theta;
        init_state_.header = msg.header;

        current_state_ = init_state_;
        last_state_ = init_state_;

        is_state_init_ = true;
    }

    if(is_state_init_)
    {
        last_state_ = current_state_;

        current_state_.car_state.x     = msg.car_state.x + 1.0*cos(msg.car_state.theta); //将车辆质心位置转换到雷达坐标位置
        current_state_.car_state.y     = msg.car_state.y + 1.0*sin(msg.car_state.theta);
        current_state_.car_state.theta = msg.car_state.theta;
        current_state_.header = msg.header;

        // ROS_INFO("last car state x = %f, y = %f, theta = %f", last_state_.car_state.x, last_state_.car_state.y, last_state_.car_state.theta);
        // ROS_INFO("current car state x = %f, y = %f, theta = %f", current_state_.car_state.x, current_state_.car_state.y, current_state_.car_state.theta);

        odom_ += getDeltaDis();
        //ROS_INFO("odom = %f", odom_);

        if(running_mode_ == "track")
        {
            Mapping::loopDetect();
        }
        if(running_mode_ == "acc")
        {
            double current_dis;
            current_dis = sqrt((current_state_.car_state.x-init_state_.car_state.x)*(current_state_.car_state.x-init_state_.car_state.x) + 
                               (current_state_.car_state.y-init_state_.car_state.y)*(current_state_.car_state.y-init_state_.car_state.y));
            if((current_dis >= acc_distance_) && (is_acc_end_ == false)) is_acc_end_ = true;
        }
        if(running_mode_ == "skid")
        {
            double current_v = sqrt(pow(msg.car_state_dt.car_state_dt.x, 2) + pow(msg.car_state_dt.car_state_dt.y, 2));
            if(current_v > 0.5 && !is_skid_start_) is_skid_start_ = true; //速度大于0.5认为开始
            if(skid_start_time_ == 0 && is_skid_start_) skid_start_time_ = ros::Time::now().toSec(); //开始运动后开始计时
            double duration_time;
            duration_time = ros::Time::now().toSec() - skid_start_time_;
            if(duration_time > 2 && current_v < 0.5) is_skid_end_ = true; //当速度再次小于0.5且启动时间大于2时认为结束
        }
    }

    if(!is_loop_closed_ && running_mode_ == "track")
    {
        ROS_WARN("is loop close: false");
    }
    if(is_loop_closed_ && running_mode_ == "track")
    {
        ROS_INFO("is loop close: true");
    }
    if(!is_acc_end_ && running_mode_ == "acc")
    {
        ROS_WARN("is acc end: false");
    }
    if(is_acc_end_ && running_mode_ == "acc")
    {
        ROS_WARN("is acc end: true");
    }
}

/***
 * @brief 轨迹回调函数，用于接收轨迹
*/
void Mapping::trackCallback(const racingslam::CarTrack::ConstPtr& msg)
{    
    //若使用地图加载，则不运行该回调函数
    if(map_load_switch_)
    {
        return;
    }
    track_ = *msg;
}

/***
 * @brief 将椎桶从雷达坐标系转换到世界坐标系
*/
fsd_common_msgs::ConeDetections Mapping::translateConeState(const fsd_common_msgs::ConeDetections& original_cones, fsd_common_msgs::CarState& car_state)
{
    fsd_common_msgs::ConeDetections target_cones;

    for(auto i : original_cones.cone_detections)
    {
        if(i.position.x*i.position.x + i.position.y*i.position.y > reg_range_) continue; //限制椎桶数量为前3对椎桶

        fsd_common_msgs::Cone temp;
        temp.color = i.color;
        temp.colorConfidence = i.colorConfidence;
        temp.poseConfidence = i.poseConfidence;
        temp.position.x = car_state.car_state.x + (i.position.x + lidar2cog_length_)*cos(car_state.car_state.theta) - (i.position.y)*sin(car_state.car_state.theta);
        temp.position.y = car_state.car_state.y + (i.position.y)*cos(car_state.car_state.theta) + (i.position.x + lidar2cog_length_)*sin(car_state.car_state.theta);

        // ROS_INFO_STREAM("before translate(" << i.position.x << "," << i.position.y << ") -> after translate(" << temp.position.x << "," << temp.position.y << ")");
        target_cones.cone_detections.push_back(temp);
    }

    return target_cones;
}

fsd_common_msgs::ConeDetections Mapping::translate2Local(const std::vector<std::pair<ConeInSlam, double>>& original_cones_with_time, fsd_common_msgs::CarState& car_state)
{

    fsd_common_msgs::ConeDetections target_cones;

    for(auto i : original_cones_with_time)
    {

        fsd_common_msgs::Cone original_cone = i.first.getCone();
        double match_time = i.second;
        fsd_common_msgs::CarState match_state;

        // 通过时间戳匹配对应轨迹点
        if (!findMatchedPose(match_time, match_state)) {
            ROS_WARN("===============ERRORRRRRRRRRRRRRR=========================");
            // match_state = car_state;
          }

        fsd_common_msgs::Cone temp;
        temp.color = original_cone.color;
        temp.colorConfidence = original_cone.colorConfidence;
        temp.poseConfidence = original_cone.poseConfidence;

        temp.position.x = (original_cone.position.x - match_state.car_state.x)*cos(-match_state.car_state.theta)- (original_cone.position.y - match_state.car_state.y)*sin(-match_state.car_state.theta) - lidar2cog_length_;
        temp.position.y = (original_cone.position.x - match_state.car_state.x)*sin(-match_state.car_state.theta) + (original_cone.position.y - match_state.car_state.y)*cos(-match_state.car_state.theta);

        target_cones.cone_detections.push_back(temp);
    }
    return target_cones;
}

/***
 * @brief 该函数根据轨迹对椎桶进行坐标转换，得到轨迹时进行一次建图
*/
void Mapping::regCones()
{
    if(!is_state_init_) return;
    if(cone_buffer_.size() < 3) return;

    for(auto i = (cone_buffer_.end() - 3); i != cone_buffer_.end(); i++)
    {
        double time_cone = i->header.stamp.toSec();
        double min_delta_time = std::numeric_limits<double>::max();
        std::vector<geometry_msgs::PoseStamped>::iterator best_match_pose; //记录选中的轨迹中的点

        for(auto j = track_line_.poses.begin(); j != track_line_.poses.end(); j++)
        {
            double time_track = j->header.stamp.toSec();
            double delta_time = std::abs(time_cone - time_track);
            if(delta_time < min_delta_time)
            {
                min_delta_time = delta_time;
                best_match_pose = j;
            }
        }

        // 检查是否找到有效轨迹点（避免轨迹为空时的错误）
        if(min_delta_time == std::numeric_limits<double>::max())
        {
            ROS_WARN("No valid track points found for cone data, skipping...");
            continue;
        }


        fsd_common_msgs::CarState this_state;
        this_state.car_state.x = best_match_pose->pose.position.x;
        this_state.car_state.y = best_match_pose->pose.position.y;
        this_state.car_state.theta = best_match_pose->pose.orientation.z;

        // 将「帧时间戳→pose_A」存入映射表
        cone_time_2pose_map_[time_cone] = this_state;

        // 清理过期缓存
        if (cone_time_2pose_map_.size() > max_cache_frames_) {
            auto oldest_it = cone_time_2pose_map_.begin();
            ROS_INFO("Erased old entry: time=%.6f", oldest_it->first);
            cone_time_2pose_map_.erase(oldest_it); // O(log n) 复杂度
        }

        fsd_common_msgs::ConeDetections temp_cone_detections = translateConeState(*i, this_state);

        // 继承时间戳[融合--雷达--slam]
        temp_cone_detections.header.stamp = i->header.stamp;
        
        //将椎桶送入cone_2time_map_
        if(cone_2time_map_.empty())
        {
            // 首次全部加入
            for(auto j: temp_cone_detections.cone_detections)
            {
                if(j.poseConfidence.data > 0.8)
                {
                    ConeInSlam temp(j);
                    temp.setTime(temp_cone_detections.header.stamp.toSec());
                    cone_2time_map_.emplace_back(temp, temp.getTime());
                }
            }
        }
        else
        {
            for(auto j: temp_cone_detections.cone_detections)
            {
                if(j.poseConfidence.data > 0.8)
                {
                    double min_dis = std::numeric_limits<double>::max();

                    std::vector<std::pair<ConeInSlam, double>>::iterator cone_to_update;
                    // std::vector<ConeInSlam>::iterator cone_to_update;

                    for(auto k = cone_2time_map_.begin(); k != cone_2time_map_.end(); k++)
                    {
                        fsd_common_msgs::Cone temp = k->first.getCone();
                        double dis = Mapping::dis(j,temp);
                        if(dis < min_dis)
                        {
                            min_dis = dis;
                            cone_to_update = k;
                        }
                    }
                    if(min_dis <= max_reg_dis_)
                    {
                        // 更新位置和时间戳
                        cone_to_update -> first.update(j);
                        cone_to_update->second = temp_cone_detections.header.stamp.toSec();
                    }
                    else
                    {   
                        // 新增
                        ConeInSlam temp(j);
                        temp.setTime(temp_cone_detections.header.stamp.toSec());
                        cone_2time_map_.emplace_back(temp, temp.getTime());
                    }
                }
            }
        }
    }
        
    
}

/**
 * @brief 对椎桶进行滤波，将两个距离过近的椎桶取平均值之后合并
 * 
 */
void Mapping::coneFilter() 
{
    auto cone_map_end = cone_2time_map_.end();

    for(auto i = cone_2time_map_.begin(); i != cone_map_end; i++) {

        double min_dis = std::numeric_limits<double>::max();
        std::vector<std::pair<ConeInSlam, double>>::iterator cone_to_del;

        for(auto j = cone_2time_map_.begin(); j != cone_2time_map_.end(); j++) {

            if(i == j) continue;

            double current_dis = dis(i->first, j->first);
            // double current_dis = dis(*i, *j);
            if(current_dis < min_dis && current_dis <= min_cone_distance_ && i->first.getCone().color.data == j->first.getCone().color.data) {
                min_dis = current_dis;
                cone_to_del = j;
            }
        }

        if(min_dis == std::numeric_limits<double>::max()) continue;

        geometry_msgs::Point i_pos = i->first.getPosition();
        geometry_msgs::Point other_pos = cone_to_del->first.getPosition();
        i->first.setPosition((i_pos.x+other_pos.x)/2, (i_pos.y+other_pos.y)/2);
        cone_2time_map_.erase(cone_to_del);
    }
}

/***
 * @brief 该函数用于进行回环检测，若检测到已经完成回环，则发送真值
*/
void Mapping::loopDetect()
{    
    double current_state_dis = dis(current_state_);
    // double last_state_dis = dis(last_state_);
    // delta_dis_ = (delta_dis_ + (current_state_dis - last_state_dis)) / 2;

    if(current_state_dis > loop_detection_dis_ && loop_detection_timer_ == -1) loop_detection_timer_ = 0; //启动回环检测
    if(current_state_dis < loop_detection_dis_ && loop_detection_timer_ == 0)  loop_detection_timer_ = 1; //回环检测完成

    if(loop_detection_timer_ == 1){
        is_loop_closed_ = true;
        filterOutlierCones();
    } 


    //ROS_INFO("current dis: %f; last dis: %f, delta: %f", current_state_dis, last_state_dis, delta_dis_);
    // if(current_state_dis <= loop_detection_dis_ && odom_ >= 20) 
    // {
    //     if(delta_dis_ > 0 && loop_detection_timer_ < 0)
    //     {
    //         ROS_INFO("TIMER ACTIVE! ");
    //         loop_detection_timer_ = 0;
    //     }
    //     else if(delta_dis_ > 0 && loop_detection_timer_ >= 0) //距离增大增加计数器
    //     {
    //         loop_detection_timer_++;
    //     }
    //     else if(delta_dis_ < 0 && loop_detection_timer_ >= 0) //距离减小减少计数器
    //     {
    //         ROS_INFO("TIMER_INACTIVE! ");
    //         loop_detection_timer_ = -1;
    //     }

    //     if(loop_detection_timer_ >= loop_detection_timer_count_)
    //     {
    //         is_loop_closed_ = true;
    //     }

    // }
    // else 
    // {
    //     if(loop_detection_timer_ >= 0)
    //     {
    //         ROS_INFO("TIMER_INACTIVE! ");
    //         loop_detection_timer_ = -1;
    //     }
    // }
}

/***
 * @brief 该函数用于将已经建好的椎桶地图以及行使过的gps轨迹保存到txt文件中
*/
void Mapping::saveMap()
{
    //保存椎桶地图
    std::ofstream cone_map(map_save_path_ + "cone_map.txt", std::ofstream::out | std::ofstream::trunc);

    if (!cone_map.is_open())
    {
        std::cout << "Failed to open cone map!" << std::endl;
        return;
    }

    for(auto i: cone_2time_map_)
    {
        if(i.first.getUpdateCount() >= min_update_count_)
        {
            double temp_x = i.first.getCone().position.x;
            double temp_y = i.first.getCone().position.y;
            std::string temp_color = i.first.getCone().color.data;

            cone_map << std::fixed << std::setprecision(4) << temp_x << " " << temp_y << " " << temp_color << std::endl;
        }
    }

    cone_map.close();

    //保存gps轨迹地图
    std::ofstream gps_map(map_save_path_ + "gps_map.txt", std::ofstream::out | std::ofstream::trunc);

    if (!gps_map.is_open())
    {
        std::cout << "Failed to open gps map!" << std::endl;
        return;
    }

    for(auto j: track_line_.poses)
    {
        double temp_x = j.pose.position.x;
        double temp_y = j.pose.position.y;
        double temp_theta = j.pose.orientation.z;

        gps_map << std::fixed << std::setprecision(4) << temp_x << " " << temp_y << " " << temp_theta << std::endl;
        //发与控制生成地图用
        // gps_map << std::fixed << std::setprecision(4) << temp_x << std::endl;
        // gps_map << std::fixed << std::setprecision(4) << temp_y << std::endl;

    }

    gps_map.close();
}

/***
 * @brief 该函数将保存的地图信息加载，以便于后续跑动能够直接进行全局跑动
*/
void Mapping::loadMap()
{
    //检验地图文件是否为空
    std::ifstream check_cone_map(map_save_path_ + "cone_map.txt", std::ios::binary | std::ios::ate); // 打开文件并将指针移到末尾
    if (!check_cone_map.is_open()) 
    {
        std::cerr << "Could not open the file: " << map_save_path_ + "cone_map.txt" << std::endl;
        return;
    }

    std::streampos check_cone_map_size = check_cone_map.tellg(); // 获取文件大小
    check_cone_map.close();
    if(check_cone_map_size == 0) ROS_WARN("cone map is empty");  //学长写的enpty笑死我了，小登笑了学长就不准笑我了哦


    std::ifstream check_gps_map(map_save_path_ + "gps_map.txt", std::ios::binary | std::ios::ate);
    if (!check_gps_map.is_open()) 
    {
        std::cerr << "Could not open the file: " << map_save_path_ + "gps_map.txt" << std::endl;
        return;
    }

    std::streampos check_gps_map_size = check_gps_map.tellg(); // 获取文件大小
    check_gps_map.close();
    if(check_gps_map_size == 0) ROS_WARN("gps map is empty");

    //加载椎桶地图文件
    std::ifstream cone_map(map_save_path_ + "cone_map.txt");

    if (!cone_map.is_open()) 
    {
        std::cerr << "cone map to open file!" << std::endl;
        return;
    }

    std::string cone_map_line;
    while(std::getline(cone_map, cone_map_line))
    {
        double temp_x, temp_y; std::string temp_color;
        std::istringstream cone_iss(cone_map_line);

        if (cone_iss >> temp_x >> temp_y >> temp_color) 
        {
            fsd_common_msgs::Cone temp_cone;
            temp_cone.color.data = temp_color;
            temp_cone.position.x = temp_x;
            temp_cone.position.y = temp_y;
            ConeInSlam temp_cone_in_slam(temp_cone);
            temp_cone_in_slam.setUpdateCount(999);  // 手动更新这一块

            cone_2time_map_.emplace_back(temp_cone_in_slam, 0.0);   // 启用地图加载的时间戳全设0
            // map_buffer_.push_back(temp_cone_in_slam);
            // std::cout << "Read values: " << temp_x << ", " << temp_y << ", " << temp_color << std::endl;
        } 
        else 
        {
            std::cerr << "Error parsing line!" << std::endl;
        }
    }

    //加载gps地图文件
    if(center_load_switch_){
        std::ifstream gps_map(map_save_path_ + "center_map.txt");
        if (!gps_map.is_open()) 
        {
            std::cerr << "gps map to open file!" << std::endl;
            return;
        }
    
        std::string gps_map_line;
        while(std::getline(gps_map, gps_map_line))
        {
            double temp_x, temp_y;
            double temp_theta = 0.0;
            std::istringstream gps_iss(gps_map_line);
    
            if (gps_iss >> temp_x >> temp_y)
            {
                std::string dummy;
                gps_iss >> dummy;
                geometry_msgs::PoseStamped temp_state;
                temp_state.pose.position.x = temp_x;
                temp_state.pose.position.y = temp_y;
                temp_state.pose.orientation.z = temp_theta;
                track_line_.poses.push_back(temp_state);
                // std::cout << "Read values: " << temp_x << ", " << temp_y << ", " << temp_theta << std::endl;
            }
            else 
            {
                std::cerr << "Error parsing line!" << std::endl;
            }
        }
        is_center_load_ = true;
    }else{
        std::ifstream gps_map(map_save_path_ + "gps_map.txt");
        if (!gps_map.is_open()) 
        {
            std::cerr << "gps map to open file!" << std::endl;
            return;
        }
    
        std::string gps_map_line;
        while(std::getline(gps_map, gps_map_line))
        {
            double temp_x, temp_y, temp_theta;
            std::istringstream gps_iss(gps_map_line);
    
            if (gps_iss >> temp_x >> temp_y >> temp_theta)
            {
                geometry_msgs::PoseStamped temp_state;
                temp_state.pose.position.x = temp_x;
                temp_state.pose.position.y = temp_y;
                temp_state.pose.orientation.z = temp_theta;
                track_line_.poses.push_back(temp_state);
                // std::cout << "Read values: " << temp_x << ", " << temp_y << ", " << temp_theta << std::endl;
            }
            else 
            {
                std::cerr << "Error parsing line!" << std::endl;
            }
        }
    }

}

/***
 * @brief 该函数将slam中使用的CarTrack类型数据转换为规划使用的Path类型数据
*/
nav_msgs::Path Mapping::carstateTrack2Path()
{
    nav_msgs::Path temp_path;
    for(auto i: track_.track)
    {
        geometry_msgs::PoseStamped p;
        p.pose.position.x = i.car_state.x;
        p.pose.position.y = i.car_state.y;
        temp_path.poses.push_back(p);
    }

    return temp_path;
}

/***
 * @brief 该函数用于得到车辆两位置之间的距离
*/
double Mapping::getDeltaDis()
{
    double delta_x = current_state_.car_state.x - last_state_.car_state.x;
    double delta_y = current_state_.car_state.y - last_state_.car_state.y;
    double res = sqrt(delta_x*delta_x + delta_y*delta_y);
    return res;
}

/***
 * @brief 该函数用于将buffer中符合条件的椎桶提取出来生成地图
*/
std::vector<std::pair<ConeInSlam, double>> Mapping::getMapWithTime()
{

    std::vector<std::pair<ConeInSlam, double>> map_with_time;

    for(auto i: cone_2time_map_)
    {
        if(i.first.getUpdateCount() >= min_update_count_)
        {
            map_with_time.emplace_back(i);
        }
    }
    return map_with_time;
}

fsd_common_msgs::ConeDetections Mapping::getMap()
{
    fsd_common_msgs::ConeDetections map;

    for(auto i : cone_2time_map_)
    {
        if(i.first.getUpdateCount() >= min_update_count_)
        {
            map.cone_detections.push_back(i.first.getCone());
        }
    }
    return map;
}


/***
 * @brief 该函数用于将buffer中符合条件的椎桶提取出来生成地图，有颜色信息
*/
fsd_common_msgs::Map Mapping::getMapWithColor()
{
    fsd_common_msgs::Map map;
    for(auto i = cone_2time_map_.begin(); i != cone_2time_map_.end(); i++)
    {
        if(i->first.getUpdateCount() >= min_update_count_)
        {
            fsd_common_msgs::Cone temp = i->first.getCone();
            if(temp.color.data == "r") map.cone_red.push_back(temp);
            if(temp.color.data == "b") map.cone_blue.push_back(temp);
        }
    }
    
    return map;
}

/***
 * @brief 该函数根据椎桶与轨迹点的最短距离判断是否为离群点
 * @param cone 待判断的锥桶
 * @return true: 保留（距离<=阈值）；false: 过滤（距离>阈值）
 */
bool Mapping::dbscan(ConeInSlam cone)
{
    if(track_line_.poses.empty())
    {
        ROS_WARN("Track line is empty, cannot check cone distance.");
        return false;  // 轨迹为空时默认过滤
    }

    double min_dis_bwt_cone_track = std::numeric_limits<double>::max();  // 初始化最短距离为最大值
    // 遍历所有轨迹点，找锥桶到轨迹的最短距离
    for(const auto& track_point : track_line_.poses)
    {
        double current_dis = sqrt(
            pow(cone.getPosition().x - track_point.pose.position.x, 2) + 
            pow(cone.getPosition().y - track_point.pose.position.y, 2)
        );
        // 更新最短距离
        if(current_dis < min_dis_bwt_cone_track)
        {
            min_dis_bwt_cone_track = current_dis;
        }
    }

    // 距离 <= 阈值则保留，否则过滤
    return min_dis_bwt_cone_track <= dis_to_erase_;
}

/***
 * @brief 该函数使用最小二乘法拟合轨迹
 * @param path_to_fit 需要被拟合的线段
 * @param fit_length 拟合的长度
 */
nav_msgs::Path Mapping::fitLine(const nav_msgs::Path& path_to_fit, double fit_length)
{
    nav_msgs::Path result_path;
    
    int n = path_to_fit.poses.size();
    if(n == 0) return result_path;

    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for(const auto& i: path_to_fit.poses)
    {
        sumX += i.pose.position.x;
        sumY += i.pose.position.y;
        sumXY += i.pose.position.x * i.pose.position.y;
        sumX2 += i.pose.position.x * i.pose.position.x;
    }

    double slope;//斜率
    double intercept;//截距
    slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    intercept = (sumY - slope * sumX) / n;

    double current_fix_length = 0; //已拟合长度
    double step_length = abs(sqrt((0.1*0.1) / (1 + slope*slope)));//计算步长 x轴上

    bool direction_flag; //判断步长增加方向 true为递增 false为递减
    if(path_to_fit.poses.front().pose.position.x < path_to_fit.poses.back().pose.position.x) direction_flag = true;
    if(path_to_fit.poses.front().pose.position.x > path_to_fit.poses.back().pose.position.x) direction_flag = false;
    //相当特殊情况，几乎不可能，但是仍要做好预备,直接增加y方向长度
    if(path_to_fit.poses.front().pose.position.x == path_to_fit.poses.back().pose.position.x)
    {
        for(int i=0; i<750; i++)
        {
            double x = path_to_fit.poses.front().pose.position.x;
            double y = path_to_fit.poses.front().pose.position.y + 0.1*i;

            geometry_msgs::PoseStamped temp;
            temp.pose.position.x = x;
            temp.pose.position.y = y;

            result_path.poses.push_back(temp);
        }

        return result_path;
    }

    if(direction_flag)
    {
        for(int i=0; current_fix_length < fit_length; i++)
        {
            double x = path_to_fit.poses.front().pose.position.x + step_length*i;
            double y = slope*x + intercept;

            geometry_msgs::PoseStamped temp;
            temp.pose.position.x = x;
            temp.pose.position.y = y;

            result_path.poses.push_back(temp);

            current_fix_length = sqrt(pow(x - path_to_fit.poses.front().pose.position.x, 2) + pow(y - track_line_.poses.front().pose.position.y, 2));
        }
    }
    else
    {
        for(int i=0; current_fix_length < fit_length; i++)
        {
            double x = path_to_fit.poses.front().pose.position.x - step_length*i;
            double y = slope*x + intercept;

            geometry_msgs::PoseStamped temp;
            temp.pose.position.x = x;
            temp.pose.position.y = y;

            result_path.poses.push_back(temp);

            current_fix_length = sqrt(pow(x - path_to_fit.poses.front().pose.position.x, 2) + pow(y - track_line_.poses.front().pose.position.y, 2));
        }
    }

    return result_path;
}

/***
 * @brief 该函数用于处理八字绕环最后一段直线较短问题，帮助控制解决丢失规划线问题，同时，将类型转换为八字绕环需要的类型
 */
fsd_common_msgs::SkidpadGlobalCenterLine Mapping::dealSkid()
{
    nav_msgs::Path temp_path; //记录八字最后直线处轨迹
    nav_msgs::Path fit_temp_path; //拟合后的八字最后直线处轨迹

    for(int i = 10; i >= 1; i--)
    {
        geometry_msgs::PoseStamped temp_pose;

        temp_pose = *(track_line_.poses.end() - i);

        temp_path.poses.push_back(temp_pose);
    }

    fit_temp_path = fitLine(temp_path, 5.0);

    for(auto i: fit_temp_path.poses)
    {
        track_line_.poses.push_back(i);
    }

    fsd_common_msgs::SkidpadGlobalCenterLine result_path;
    result_path.isReachMid = true;

    for(auto i: track_line_.poses)
    {
        result_path.global_path.poses.push_back(i);
    }

    return result_path;
}

/***
 * @brief 该函数用于开始进行slam
*/
void Mapping::startSlam()
{    
    //std::cout << "-------------------------------------------------------------" << std::endl;

    if(map_load_switch_)
    {
        if(!is_map_load_)
        {
            loadMap();
            is_map_load_ = true;
        }
        fsd_common_msgs::Map map;
        map = Mapping::getMapWithColor();
        map_pub_.publish(map);
        ROS_WARN("================== map have been published ==================");
        ROS_WARN("============ global gps line have been published ============");
        Mapping::sendMapVisualization(frame_id_);
        track_line_.header.frame_id = frame_id_;
        global_gps_line_pub_.publish(track_line_);

        if(running_mode_ == "acc")
        {
            if(!is_acc_track_fit)
            {
                fix_acc_track_ = fitLine(track_line_, fit_acc_track_length_);
                is_acc_track_fit = true;
            }

            fix_acc_track_.header.frame_id = frame_id_;
            fix_acc_track_pub_.publish(fix_acc_track_);
            ROS_WARN("============ fix acc line have been published ============");
        }
        if(running_mode_ == "skid")
        {
            if(!is_skid_dealt)
            {
                skid_path_ = dealSkid();
                is_skid_dealt = true;
            }

            skid_path_pub_.publish(skid_path_);
        }
        
        return;
    }
    
    if((is_track_fix_) && (running_mode_ == "track") && (!map_load_switch_))
    {
        //为防止进入全局时轨迹出现问题，得到全局线之后延时2s发送
        if(track_fix_start_time_ == 0) track_fix_start_time_ = ros::Time::now().toSec();
        double current_time;
        current_time = ros::Time::now().toSec();
        if(current_time - track_fix_start_time_ <= 2) return;
        
        map_pub_.publish(getMapWithColor());
        ROS_WARN("================== map have been published ==================");

        global_gps_line_pub_.publish(track_line_);
        ROS_WARN("============ global gps line have been published ============");

        if(!is_map_saved_)
        {
            saveMap();
            is_map_saved_ = true;
        }
        ROS_WARN("==================== map have been saved ====================");

        return;
    }
    if((is_acc_end_) && (running_mode_ == "acc") && (!map_load_switch_))
    {
        map_pub_.publish(getMapWithColor());
        ROS_WARN("================== map have been published ==================");
        
        global_gps_line_pub_.publish(track_line_);
        ROS_WARN("============ global gps line have been published ============");

        if(!is_map_saved_)
        {
            saveMap();
            is_map_saved_ = true;
        }
        ROS_WARN("==================== map have been saved ====================");

        return;
    }
    if((is_skid_end_) && (running_mode_ == "skid") && (!map_load_switch_))
    {
        map_pub_.publish(getMapWithColor());
        ROS_WARN("================== map have been published ==================");
        
        global_gps_line_pub_.publish(track_line_);
        ROS_WARN("============ global gps line have been published ============");

        if(!is_map_saved_)
        {
            saveMap();
            is_map_saved_ = true;
        }
        ROS_WARN("==================== map have been saved ====================");

        return;
    }
    
    ros::Time start_time = ros::Time::now();

    regCones();

    coneFilter();

    ros::Time end_time = ros::Time::now();
    
    double mapping_delay = end_time.toSec() - start_time.toSec();
    ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++mapping delay = %f ms", mapping_delay*1000);

    // fsd_common_msgs::ConeDetections realtime_map;
    std::vector<std::pair<ConeInSlam, double>> realtime_map_with_time;
    realtime_map_with_time = Mapping::getMapWithTime();

    fsd_common_msgs::ConeDetections realtime_map_local = translate2Local(realtime_map_with_time, current_state_);

    realtime_map_local.header.frame_id = "rslidar"; // 设置局部坐标系
    realtime_map_local.header.stamp = ros::Time::now();
    realtime_map_pub_.publish(realtime_map_local);
    sendRealtimeMapVisualization(realtime_map_local, "rslidar");

    Mapping::sendMapVisualization(frame_id_); //地图可视化

}

/***
 * @brief 该函数用来发送可视化椎桶地图话题信息
*/
void Mapping::sendMapVisualization(std::string frame_id)
{
    visualization_msgs::Marker cone_marker;
    visualization_msgs::MarkerArray cone_maker_array;

    //Set the marker type
    cone_marker.type = visualization_msgs::Marker::CUBE;
    cone_marker.action = visualization_msgs::Marker::ADD;

    cone_marker.header.stamp = ros::Time::now();
    cone_marker.header.frame_id = frame_id;
														   
    cone_marker.ns = "slam";

    cone_marker.lifetime = ros::Duration();
    cone_marker.frame_locked = true;

    //Set the color
    cone_marker.color.r = 1.0f;
    cone_marker.color.g = 0.0f;
    cone_marker.color.b = 0.0f;
    cone_marker.color.a = 0.8f;

    cone_marker.scale.x = 0.2;
    cone_marker.scale.y = 0.2;
    cone_marker.scale.z = 0.3;

    // default val to fix warn of rviz
    cone_marker.pose.orientation.x = 0;
    cone_marker.pose.orientation.y = 0;
    cone_marker.pose.orientation.z = 0;
    cone_marker.pose.orientation.w = 1;

    fsd_common_msgs::ConeDetections map;
    map = Mapping::getMap();

    for(auto i: map.cone_detections)
    {
        cone_marker.id++;
        cone_marker.pose.position.x = i.position.x;
        cone_marker.pose.position.y = i.position.y;

        if(i.color.data == "r")
        {
            cone_marker.color.r = 1.0f;
            cone_marker.color.g = 0.0f;
            cone_marker.color.b = 0.0f;
        }
        else if(i.color.data == "b")
        {
            cone_marker.color.r = 0.0f;
            cone_marker.color.g = 0.0f;
            cone_marker.color.b = 1.0f;
        }
        else
        {
            cone_marker.color.r = 0.0f;
            cone_marker.color.g = 1.0f;
            cone_marker.color.b = 0.0f;
        }

        cone_maker_array.markers.push_back(cone_marker);
    }

    map_visualization_pub_.publish(cone_maker_array);
}

/***
 * @brief 过滤全局地图中离轨迹过远的锥桶（调用dbscan判断）
 */
void Mapping::filterOutlierCones()
{
    if(!is_loop_closed_) return;


    if(cone_2time_map_.empty()) return;

    // 遍历锥桶地图，移除离群点
    std::vector<std::pair<ConeInSlam, double>>::iterator it = cone_2time_map_.begin();
    // auto it = map_buffer_.begin();
    while(it != cone_2time_map_.end())
    {
        if(!dbscan(it->first))  // dbscan返回false表示需要过滤
        {
            ROS_INFO_STREAM("Filter outlier cone (x: " << it->first.getPosition().x 
                << ", y: " << it->first.getPosition().y << "), distance: " << dis_to_erase_ << "m");
            
            it = cone_2time_map_.erase(it);  // 移除离群锥桶
        }
        else ++it;  // 保留，继续下一个
    }

    ROS_INFO_STREAM("Filtered outlier cones. Remaining cones: " << cone_2time_map_.size());
}

/***
 * @brief 该函数用来发送实时地图可视化话题信息（局部坐标系）
*/
void Mapping::sendRealtimeMapVisualization(const fsd_common_msgs::ConeDetections& realtime_map, std::string frame_id)
{
    visualization_msgs::Marker cone_marker;
    visualization_msgs::MarkerArray cone_maker_array;

    //Set the marker type
    cone_marker.type = visualization_msgs::Marker::CUBE;
    cone_marker.action = visualization_msgs::Marker::ADD;

    cone_marker.header.stamp = ros::Time::now();
    cone_marker.header.frame_id = frame_id;
														   
    cone_marker.ns = "realtime_slam";

    cone_marker.lifetime = ros::Duration(0.1); // 短暂的生命周期，只显示最新数据
    cone_marker.frame_locked = true;

    //Set the color
    cone_marker.color.r = 1.0f;
    cone_marker.color.g = 0.0f;
    cone_marker.color.b = 0.0f;
    cone_marker.color.a = 0.8f;

    cone_marker.scale.x = 0.6;
    cone_marker.scale.y = 0.6;
    cone_marker.scale.z = 0.9;

    // default val to fix warn of rviz
    cone_marker.pose.orientation.x = 0;
    cone_marker.pose.orientation.y = 0;
    cone_marker.pose.orientation.z = 0;
    cone_marker.pose.orientation.w = 1;

    for(auto i: realtime_map.cone_detections)
    {
        cone_marker.id++;
        cone_marker.pose.position.x = i.position.x;
        cone_marker.pose.position.y = i.position.y;

        if(i.color.data == "r")
        {
            cone_marker.color.r = 1.0f;
            cone_marker.color.g = 0.0f;
            cone_marker.color.b = 0.0f;
        }
        else if(i.color.data == "b")
        {
            cone_marker.color.r = 0.0f;
            cone_marker.color.g = 0.0f;
            cone_marker.color.b = 1.0f;
        }
        else
        {
            cone_marker.color.r = 0.0f;
            cone_marker.color.g = 1.0f;
            cone_marker.color.b = 0.0f;
        }

        cone_maker_array.markers.push_back(cone_marker);
    }

    realtime_map_visualization_pub_.publish(cone_maker_array);
}

