#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <boost/bind.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <eigen3/Eigen/Dense>

//tf2
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// 消息过滤与时间同步
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// rclcpp
#include "rclcpp/rclcpp.hpp"

// 消息类型-订阅
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp" 
#include "lgsvl_msgs/msg/vehicle_odometry.hpp"
#include "lgsvl_msgs/msg/can_bus_data.hpp"
#include "nav_msgs/msg/odometry.hpp"

// 消息类型-发布
#include "lgsvl_msgs/msg/vehicle_state_data.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

#define PI 3.1415926

using namespace std::chrono_literals;
using Eigen::MatrixXd;
using std::string;
using std::vector;
using std::ifstream;
using std::istringstream;
using std::stringstream;
using std::cout;
using std::endl;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, lgsvl_msgs::msg::VehicleOdometry, 
                                                        lgsvl_msgs::msg::CanBusData, nav_msgs::msg::Odometry> MySyncPolicy;


class ChassisController: public rclcpp::Node{
    public:
        ChassisController();
        ~ChassisController();
    
    private:
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub;
        message_filters::Subscriber<lgsvl_msgs::msg::VehicleOdometry> odometry_sub;
        message_filters::Subscriber<lgsvl_msgs::msg::CanBusData> canbus_sub;
        message_filters::Subscriber<nav_msgs::msg::Odometry> gps_sub;
        
        //msgs sync
        message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>
                                                                (MySyncPolicy(10), imu_sub, odometry_sub, canbus_sub, gps_sub);

        rclcpp::Publisher<lgsvl_msgs::msg::VehicleStateData>::SharedPtr state_pub;
        rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr control_pub;

        //timer
        rclcpp::TimerBase::SharedPtr timer;
        
        //variables
        double vx;
        double vy;
        double phi;
        double phi_p;
        double x;
        double y;
        double delta;
        double ax;
        double ay;
        double steer_angle;
        double steer_control;
        vector<vector<double>> waypoint;
        
        //subscribed msgs
        sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
        lgsvl_msgs::msg::VehicleOdometry::ConstSharedPtr odometry_msg;
        lgsvl_msgs::msg::CanBusData::ConstSharedPtr canbus_msg;
        nav_msgs::msg::Odometry::ConstSharedPtr gps_msg;

        //ros2 functions
        void msg_subscriber(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
                            const lgsvl_msgs::msg::VehicleOdometry::ConstSharedPtr& odometry_msg,
                            const lgsvl_msgs::msg::CanBusData::ConstSharedPtr& canbus_msg,
                            const nav_msgs::msg::Odometry::ConstSharedPtr& gps_msg);
        void control_publisher();

        //waypoint load function
        vector<vector<double>> waypoint_loader(std::string filename);

        //quaternion to eular transform
        double quat_to_euler(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);

        //solver function
        bool riccati_solver(MatrixXd A, MatrixXd B, MatrixXd Q,
                            MatrixXd R, MatrixXd &P, const double tolerance = 1.E-2,
                            const uint iter_max = 500);

        //controller functions
        vector<double> reference_finder(vector<vector<double>> waypoint_list, double pos_x, double pos_y);
        double lateral_controller(double yaw, double yaw_rate, double pos_x, double pos_y, 
                                            double velocity_x, double velocity_y);
        double longitudinal_controller(double velocity, double acc_x);
};