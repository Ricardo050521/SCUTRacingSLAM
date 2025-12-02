#ifndef FRENET_COORDINATE_HANDLE_HPP
#define FRENET_COORDINATE_HANDLE_HPP

#include "ros/ros.h"
#include "fsd_common_msgs/ConeDetections.h"
#include "geometry_msgs/PolygonStamped.h"
#include "frenet_coordinate_handle.hpp"
#include "cubic_spline_interpolation.hpp"
#include "fitted_curve.hpp"
#include "fsd_common_msgs/CarState.h"
#include <cmath>
#include <sys/time.h>
#include "std_msgs/Float32.h"
//#include "convert_s/serviceData.h"
#include <map>
#include "basic_.hpp"
#include "nav_msgs/Path.h"
#include <vector>
#include "Eigen/Dense"
#include "fsd_common_msgs/Map.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "frenet_coordinate/RoadWidth.h"

namespace ns_frenet_coordinate
{

  class FrenetCoordinateHandle
  {
  public:

    // Constructor
    FrenetCoordinateHandle(ros::NodeHandle &nodeHandle);

    // Getters
    int getNodeRate() const;

    // 新增文件读取函数
    void loadConeMapFromFile();
    void loadGpsMapFromFile();

    // Methods
    void loadParameters();
    void subscribeToTopics();
    void publishToTopics();
    void srvCall();
    void run();
    void runAlgorithm();
    void route(const nav_msgs::Path &gps_center_path_);
    void find_rs(double x, double y);
    void cal_center_path_frenet();
    void cartesian_to_frenet(double rs, double rx, double ry, double rtheta, double x, double y);
    void frenet_to_cartesian(double rx,double ry,double rtheta,double s_condition,double d_condition);

    //LPADD
    bool map_load_switch_;
    bool is_start_switch_;
    int flag_gps_num = 0;
    int gps_num = 0;
    double centre_evaluate_map = 0.0, dis_evaluate_map = 0.0, area_evaluate_map = 0.0, evaluate_data = 0.0, odom_dis = 0.0;
    ros::Publisher FilteredPath, Visual_Center_Line, Visual_Discrete_Map, Discrete_Map,
                   Map_Left, Map_Right, Visual_Map_Right, Visual_Map_Left, Visual_Temp_1, Visual_Temp_2,
                   auto_color_map_publisher_;
    nav_msgs::Path gps_path_filtered;
    visualization_msgs::MarkerArray marker_array_center_line, marker_array_discrete_line, marker_array_map_left, marker_array_map_right, Temp_1, Temp_2;
    fsd_common_msgs::ConeDetections map_for_ribbon, map_from_slam, 
                                    map_from_slam_left, map_from_slam_right, 
                                    map_for_ribbon_left, map_for_ribbon_right,
                                    map_from_slam_left_ordered, map_from_slam_right_ordered,
                                    map_for_publish_left,map_for_publish_right;
    void discrete_center_line();
    void visiual_marker_array_CUBE(visualization_msgs::MarkerArray &marker_array, std::string frame_id_, std::string color, double x, double y, double z);
    void visiual_marker_array_ARROW(visualization_msgs::MarkerArray &marker_array, std::string frame_id_, std::string color, double x1, double y1, double z1, double x2, double y2, double z2);
    double evaluate_discrete_map();
    void calculate_center(fsd_common_msgs::ConeDetections & map, int map_size, double in_or_out, double flag_which, double & cx, double & cy, double & A);
    // void deal_global_center_path(int flag_choose_what, int line_choose_what);
    void deal_global_center_path();
    void global_center_path_TO_visiual_marker_array_CUBE();
    void LinearEquations4(double x1, double x2, double y1, double y2, double k1, double k2, double & a, double & b, double & c, double & d);
    void VariableSlopeInterpolation(double x1, double x2, double y1, double y2, double k_counts[][2], double add_points[][2], int add_points_num, double reference_dis, int line_choose_what);
    double CurveFunction(double a, double b, double c, double d, double x);
    double DiffFunction(double a, double b, double c, double d, double x);

  private:
    ros::NodeHandle nodeHandle_;
    geometry_msgs::PolygonStamped cone_global_disorder_coordinate; //此类型包含多边形及其相关信息，多边形的定点为geometry_msgs/Point32类型
    geometry_msgs::PolygonStamped cone_global_disorder_coordinate2;
    geometry_msgs::PolygonStamped dense_center_line;

    // geometry_msgs::PolygonStamped gps_center_line;
    geometry_msgs::Point32 head_pp;
    nav_msgs::Path gps_center_path; //笛卡尔坐标下走过的路径，其中z轴存储的是已经走过的距离
    ros::Subscriber globalMapSubscriber;
    ros::Subscriber GpsSubscriber;
    ros::Subscriber slamStateSubscriber_;
    // fssim
    ros::Subscriber slamMapSubscriber_;

    ros::Publisher FrenetDetectionsPublisher;
    ros::Publisher frenet_coordinate_compute_time_Publisher;
    ros::Publisher width_pub;


    void globalMapCallback(const fsd_common_msgs::Map &msg);
    void gpscenterlineCallback(const nav_msgs::Path &gpscenterpath);
    void gpscenterlineCallback_new(const fsd_common_msgs::CarState &gpscenterpath);
    void localMapCallback(const fsd_common_msgs::ConeDetections &msg);
    void slamStateCallback(const fsd_common_msgs::CarState &state);
    void setState(const fsd_common_msgs::CarState &state);
    // fssim
    void slamMapCallback(const fsd_common_msgs::Map &map);

    std::string global_map_topic_name_;
    std::string gps_centerline_topic_name_;
    std::string frame_id_;
    std::string slam_state_topic_name_;
    std::string frenet_coordinate_topic_name_;
    std::string frenet_coordinate_compute_time_topic_name_;
    // fssim
    std::string slam_map_topic_name_;

    int node_rate_;
    bool get_gps = false;
    bool get_cone = false;
    bool sign_finish = false;

    bool cone_map_loaded_ = false;
    bool gps_map_loaded_ = false; 

    int iter_count = 0;
    fsd_common_msgs::CarState  state_;

    std::map <int, frenetbase> gps_map;
    std::map <double, frenetcone> left_cone_frenet; //x,y为笛卡尔坐标   cone_s_, cone_d_属于fernet坐标
    std::map <double, frenetcone> right_cone_frenet; //x,y为笛卡尔坐标   cone_s_, cone_d_属于fernet坐标

    geometry_msgs::PolygonStamped left_cone_; //left_cone_存储左侧椎桶的全部frenet坐标
    geometry_msgs::PolygonStamped right_cone_; //right_cone_存储右侧椎桶的全部frenet坐标
    geometry_msgs::PolygonStamped left_cone_sp; //使用样条插值之后的点
    geometry_msgs::PolygonStamped right_cone_sp;
    geometry_msgs::PolygonStamped global_center_line_frenet; //frenet坐标下的全局中心线
    frenet_coordinate::RoadWidth road_width;

    visualization_msgs::MarkerArray auto_color_map_;

    nav_msgs::Path global_center_path; //笛卡尔坐标系下的全局中心线

    double s_start;
    double s_end;

    //for evaluate map
    double evaluation_threshold_;
    double dis_evaluation_weight_;
    double centre_evaluation_weight_;
    double discrete_dis_;
    int jump_points_;
    double record_dis_;
    double start_dis_;
    double end_dis_;
    double change_angle_rate_;
    double change_reference_rate_;

    struct timeval start_time, end_time;
    std_msgs::Float32 frenet_coordinate_compute_time;

    std::string center_line_save_path_;  // 中心线保存路径
    bool save_center_line_ = false;       // 是否保存中心线开关
    void saveCenterLine();                // 保存中心线函数


    bool load_frenet_cones_from_file_;  // 从文件加载Frenet锥桶的开关
    void saveFrenetConesAndColorMap();
    bool loadFrenetConesFromFile();
    void processConesNormally();
  };
} 
#endif 
