#include "xcontroller.hpp"

// 控制器
ChassisController::ChassisController() : Node("mycontroller"){
    RCLCPP_INFO(this->get_logger(), "My chassis controller");

    // subscribe
    imu_sub.subscribe(this, "/simulator/sensor/imu");
    odometry_sub.subscribe(this, "/simulator/odometry");
    canbus_sub.subscribe(this, "/simulator/canbus");
    gps_sub.subscribe(this, "/simulator/nav/gps");
    sync -> registerCallback(boost::bind(&ChassisController::msg_subscriber, this, _1, _2, _3, _4));

    //load waypoint
    waypoint=waypoint_loader("./maps/pointmap/shalun.csv");

    // publish
    state_pub = this->create_publisher<lgsvl_msgs::msg::VehicleStateData>("/simulator/vehicle_state", 10);
    control_pub = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/simulator/vehicle_control", 10);
    timer = this->create_wall_timer(500ms, std::bind(&ChassisController::control_publisher, this));
    
}

ChassisController::~ChassisController()
{
    delete sync;
}

void ChassisController::msg_subscriber(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg, 
                            const lgsvl_msgs::msg::VehicleOdometry::ConstSharedPtr& odometry_msg,
                            const lgsvl_msgs::msg::CanBusData::ConstSharedPtr& canbus_msg,
                            const nav_msgs::msg::Odometry::ConstSharedPtr& gps_msg)
{
    //parameters
    vx = canbus_msg->speed_mps;
    vy = 0; //need calc
    phi_p = imu_msg->angular_velocity.z;
    phi_p = phi_p*PI/180;
    phi = quat_to_euler(imu_msg);
    x = -gps_msg->pose.pose.position.y; //to be modified
    y = gps_msg->pose.pose.position.x; //to be modified
    delta = odometry_msg->front_wheel_angle;
    ax = imu_msg->linear_acceleration.x; 
    ay = imu_msg->linear_acceleration.y;
    steer_angle = odometry_msg->front_wheel_angle;
    //RCLCPP_INFO(this->get_logger(), "Velocity: %f [m/s]", vx);
    //RCLCPP_INFO(this->get_logger(), "Front wheel angle: %f [rad]", steer_angle);
    //RCLCPP_INFO(this->get_logger(), "PositionX: %f [m], PositionY: %f [m]", x, y);
    //RCLCPP_INFO(this->get_logger(), "ACCX: %f [m/s-2], ACCY: %f [m/s-2]", ax, ay);
    //RCLCPP_INFO(this->get_logger(), "Yaw Anlge: %f [rad]", phi);
}

void ChassisController::control_publisher()
{
    steer_control=lateral_controller(phi, phi_p, x, y, vx, vy);
    auto control = lgsvl_msgs::msg::VehicleControlData();
    control.target_gear = lgsvl_msgs::msg::VehicleControlData::GEAR_DRIVE; //gear
    control.acceleration_pct = 0.05;  //acc in percentage
    control.braking_pct = 0; //brake in percentage
    control.target_wheel_angle = steer_control; //steering angle in rad
    control.target_wheel_angular_rate = 0; //steering angle velocity in rad/s

    auto state = lgsvl_msgs::msg::VehicleStateData();
    state.autonomous_mode_active = true; 
    state.vehicle_mode= lgsvl_msgs::msg::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE;

    state_pub->publish(state);
    control_pub->publish(control);
}

vector<vector<double>> ChassisController::waypoint_loader(string filename)
{
    vector<vector<double>> waypoint_container;
    ifstream fin(filename);
    string line;
    while (getline(fin, line))
    {
        istringstream sin(line);
        vector<string> Waypoints;
        string info;
        vector<double> x_y_theta_kappa;
        while(getline(sin, info, ',')){
            Waypoints.push_back(info);
        }
        string x_str=Waypoints[0];
        string y_str=Waypoints[1];
        string theta_str=Waypoints[2];
        string kappa_str=Waypoints[3];
        double waypoint_x,waypoint_y,waypoint_theta,waypoint_kappa;
        stringstream sx,sy,stheta,skappa;
        sx<<x_str;
        sy<<y_str;
        stheta<<theta_str;
        skappa<<kappa_str;
        sx>>waypoint_x;
        sy>>waypoint_y;
        stheta>>waypoint_theta;
        skappa>>waypoint_kappa;
        x_y_theta_kappa.push_back(waypoint_x);
        x_y_theta_kappa.push_back(waypoint_y);//be careful with the sign
        x_y_theta_kappa.push_back(waypoint_theta);
        x_y_theta_kappa.push_back(waypoint_kappa);
        waypoint_container.push_back(x_y_theta_kappa);

        //waypoint smoother
        vector<vector<double>> new_waypoint;
    }
    if(waypoint_container.empty()) cout<<"File is empty!!!!"<<endl;
    cout<<"Waypoints are loaded!"<<endl;
    return waypoint_container;
}

double ChassisController::quat_to_euler(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
{
    //for now only output yaw angle
    tf2::Quaternion q(
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw+PI/2;
}

vector<double> ChassisController::reference_finder(vector<vector<double>> waypoint_list, double pos_x, double pos_y)
{
    vector<double> distance_all;
    for(auto i: waypoint_list)
    {
        double dis_s = (pos_x - i[0]) * (pos_x - i[0]) + (pos_y - i[1]) * (pos_y - i[1]);
        distance_all.push_back(dis_s);
    }
    vector<double>::iterator min = std::min_element(std::begin(distance_all), std::end(distance_all));
    double index = std::distance(std::begin(distance_all), min);
    vector<double> final_point;
    final_point.push_back(waypoint_list[index][0]);//x
    final_point.push_back(waypoint_list[index][1]);//y
    final_point.push_back(waypoint_list[index][2]);//theta
    final_point.push_back(waypoint_list[index][3]);//kappa
    return final_point;
}

bool ChassisController::riccati_solver(MatrixXd A, MatrixXd B, MatrixXd Q,
                            MatrixXd R, MatrixXd &P,const double tolerance,
                            const uint iter_max)
{
    P = Q;
    double diff;
    for (uint i = 0; i < iter_max; ++i) {
    MatrixXd P_next = A.transpose() * P * A -
             A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
    if(diff<tolerance){
        break;
    }
    }
    return true;
}

double ChassisController::longitudinal_controller(double velocity_x, double acc_x)
{
    return 0; //need to create 2x1 vector which holds acc_pct and brake_pct
}

double ChassisController::lateral_controller(double yaw, double yaw_rate, double pos_x, double pos_y, 
                                            double velocity_x, double velocity_y)
{
    double ts = 0.5;
    x = x + vx * ts * cos(yaw) - vy * ts * sin(yaw);
    y = y + vy * ts * cos(yaw) + vx * ts * sin(yaw);
    yaw = yaw + yaw_rate * ts;

    vector<double> reference_pos = reference_finder(waypoint, x, y);
    double x_ref = reference_pos[0];
    double y_ref = reference_pos[1];
    double theta_ref = reference_pos[2];
    double kappa_ref = reference_pos[3];
    //Matrix
    MatrixXd tor(1,2), nor(1,2), distance(2,1);
    tor(0,0) = cos(theta_ref);
    tor(0,1) = sin(theta_ref);
    nor(0,0) = -sin(theta_ref);
    nor(0,1) = cos(theta_ref);
    distance(0,0) = pos_x - x_ref;
    distance(1,0) = pos_y - y_ref;
    double err_d = (nor * distance)(0,0);
    double err_s = (tor * distance)(0,0);

    //err_d_p
    double err_d_p = velocity_y * cos(yaw - theta_ref) + velocity_x * sin(yaw - theta_ref);
    double s_p = (velocity_x * cos(yaw - theta_ref) - velocity_y * sin(yaw - theta_ref)) / (1 - kappa_ref * err_d);

    //err_phi
    double err_phi = sin(yaw - theta_ref);

    //err_phi_p
    double err_phi_p = yaw_rate - kappa_ref * s_p;

    cout<<"err_d "<<err_d<<endl;
    cout<<"err_d_p "<<err_d_p<<endl;
    cout<<"err_phi "<<err_phi<<endl;
    cout<<"err_phi_p "<<err_phi_p<<endl;
    cout<<"Yaw: "<<yaw<<endl;
    //state matrix
    MatrixXd err_state(4,1);
    err_state(0,0) = err_d;
    err_state(1,0) = err_d_p;
    err_state(2,0) = err_phi;
    err_state(3,0) = err_phi_p;

    //vehicle parameters
    double cf = -155494.663;
    double cr = -155494.663;
    double lf = 1.3;
    double lr = 1.5;
    double m = 2080;
    double iz = 2080;
    //dynamic matrix
    MatrixXd A = MatrixXd::Zero(4,4);
    MatrixXd B = MatrixXd::Zero(4,1);
    MatrixXd P = MatrixXd::Zero(4,4);
    A(0,1) = 1;
    A(1,1) = (cf + cr) / (m * vx);
    A(1,2) = - (cf + cr) / m;
    A(1,3) =  (cf * lf - cr * lr) / (m * vx);
    A(2,3) = 1;
    A(3,1) = (cf * lf - cr * lr) / (iz * vx);
    A(3,2) = - (cf * lf - cr * lr) / iz;
    A(3,3) = (cf * lf * lf + cr * lr * lr) / (iz * vx);
    B(2,0) = -cf / m;
    B(3,0) = -cf * lf / iz;
    //discrete
    const double dt = 0.01;
    MatrixXd I = MatrixXd::Identity(4, 4);
    MatrixXd Ad;
    Ad = (I + 0.5 * dt * A) * (I - 0.5 * dt * A).inverse();
    MatrixXd Bd;
    Bd = B * dt;
    //solve lqr
    MatrixXd Q = MatrixXd::Zero(4,4);
    Q(0,0) = 10;
    Q(1,1) = 1;
    Q(2,2) = 100;
    Q(3,3) = 100;
    MatrixXd R = MatrixXd::Zero(1,1);
    R(0,0) = 100;
    MatrixXd K = MatrixXd::Zero(1,4);
    if(velocity_x > 0.001)
    {   
        riccati_solver(Ad,Bd,Q,R,P);
        K = (R + Bd.transpose() * P * Bd).inverse()* Bd.transpose() * P * Ad;
    }
    double forward_angle=kappa_ref * (lf + lr - lr * K(0,2) - (m * vx * vx / (lf + lr)) * 
                        ((lr / cf) + (lf / cr) * K(0,2) - (lf / cr)));
    double target_steer_angle = -((-1 * K * err_state)(0,0) + forward_angle);//lgsvl turn left is negative
    cout<<"steer control angle "<<target_steer_angle<<endl;
    return target_steer_angle;

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisController>());
  rclcpp::shutdown();
  return 0;
}