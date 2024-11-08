#include "../include/trajectory_generation/global_trajectory.h"

Global_traj_class::Global_traj_class(ros::NodeHandle nh, ros::NodeHandle private_nh, Eigen::MatrixXd coverage_path)
{
    private_nh_ = private_nh;
    nh_ = nh;
    traj_pub_ = private_nh_.advertise<sdv_msgs::Trajectory>("trajectory", 1);
    path_pub_ = private_nh_.advertise<nav_msgs::Path>("adapted_path", 1);
    map_sub_ = nh_.subscribe("/map", 1, &Global_traj_class::costMapCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &Global_traj_class::odomCallback, this);

////////////////////////////////////////////////////////////SET WAYPOINTS ON RVIZ////////////////////////////////
    boost::shared_ptr<geometry_msgs::PoseStamped const> shared_goal;
    ROS_INFO("Set Goal.");
    bool set_goals = true;
    geometry_msgs::Pose pose;
    Eigen::Vector2f goal_node_;
    Eigen::MatrixXd default_way_points;
    std::vector<Eigen::Vector2f> goal_nodes_;
    
    while (set_goals)
    {
        shared_goal = nullptr;
        if (shared_goal == nullptr)
        {
            if (goal_nodes_.empty())
            {
                // default_way_points.resize(2, 2);
                // default_way_points << 5.0, 0,
                //                       5.0 , 5.0;

                for (unsigned int ii = 0; ii < coverage_path.rows(); ii++)
                {
                    goal_node_ << coverage_path(ii,0), coverage_path(ii,1);
                    goal_nodes_.push_back(goal_node_);
                }
            }
            set_goals = false;
        }
        else
        {
            pose = shared_goal->pose;
            goal_node_ << pose.position.x, pose.position.y;
            goal_nodes_.push_back(goal_node_);
            std::cout << goal_node_ << std::endl;
        }
    }
    
    way_points.resize(goal_nodes_.size(), 2);
    // way_points.row(0) << 0.0, 0.0;
    for (unsigned int ii = 0; ii < goal_nodes_.size(); ii++)
    {
        way_points.row(ii) << goal_nodes_.at(ii).x(), goal_nodes_.at(ii).y();
    }

    std::cout << way_points << std::endl;
    std::cout << "Waypoints set successfully." << std::endl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    PATH_VEL_LIM = TrajectoryParameters::PATH_VEL_LIM;

}

Global_traj_class::~Global_traj_class(){}

void Global_traj_class::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_ = *msg;
}

void Global_traj_class::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_ = *msg;
}

bool Global_traj_class::isSame(double lhs, double rhs)
{
    return fabs(lhs - rhs) < EPS;
}

void Global_traj_class::euler_spiral_coefficient_generator() {

    euler_table.resize(N_EULER, 2);
    euler_table.setZero();

    double factx = 1;
    double facty = 1;

    for (unsigned int i = 0; i < N_EULER; i++)
    {
        // Factorial calculation
        factx = 1;
        if (i > 0) {
            for (int j = 1; j < 2 * i + 1; j++)
            {
                factx = factx * j;
            }
        }
        facty = factx * (2 * i + 1);

        euler_table(i, 0) = pow(-1, i) * 2 / (factx * ((4 * i) + 1));
        euler_table(i, 1) = pow(-1, i) * 2 / (facty * ((4 * i) + 3));
    }
    //std::cout << "The euler table is :\n" << euler_table << std::endl;
}

Eigen::RowVector2d Global_traj_class::unit_euler_spiral_coordinate_calculator(double &angle, Eigen::MatrixXd &table) {

    // This function computes the coordinates of a unit Euler spiral given the current orientation angle (0 < angle)
    Eigen::RowVector2d unit_euler_coordinate; // Coordinate of a point along the unit Euler spiral
    unit_euler_coordinate << 0, 0; // Initialization

    for (unsigned int i = 0; i < N_EULER; i++)
    {
        double exp_1 = 0.5 * (4 * i + 1);
        double exp_2 = 0.5 * (4 * i + 3);
        unit_euler_coordinate(0) = unit_euler_coordinate(0) + table(i, 0) * pow(angle, exp_1);
        unit_euler_coordinate(1) = unit_euler_coordinate(1) + table(i, 1) * pow(angle, exp_2);
    }
    unit_euler_coordinate = unit_euler_coordinate / sqrt(2 * PI);
    return unit_euler_coordinate;
}

double Global_traj_class::calculate_angle_from_vector(Eigen::RowVector2d &vector) {

    // This function computes the angle[rad]: -pi < angle < pi of a vector in any of the four quadrants
    double angle;
    double vector_norm = vector.norm();// hypot(vector(1), vector(2));
    Eigen::RowVector2d unit_vector = vector / vector_norm;
    double unit_vector_comp_1    = std::abs(unit_vector(0));

    if (unit_vector(1) >= 0) {
        if (isSame(unit_vector(0), 0)) {
            angle = 0.5 * PI;
        }
        else {
            angle = acos(unit_vector(0));
        }
    }
    else {
        if (isSame(unit_vector(0), 0)) {
            angle = -0.5 * PI;
        }
        else if (unit_vector(0) < 0) {
            angle = -PI + acos(unit_vector_comp_1);
        }
        else {
            angle = -acos(unit_vector_comp_1);
        }
    }
    //std::cout << "Angle:\t" << angle << std::endl;
    return angle;
}

void Global_traj_class::get_robot_current_state(Eigen::RowVector2d& robot_current_position, double& robot_current_path_velocity) {

    pos_current      = robot_current_position;      // [m]   Robot's current [X, Y] coordinates in the global map
    path_vel_current = robot_current_path_velocity; // [m/s] Robot's current path velocity.
    // NB: It is assumed that the robot's heading is toward the next way point
}
void Global_traj_class::way_point_generator() {

    // way_points.resize(3, 2);
    // way_points << 0  , 0,
    //               6.0 , 0.0,
    //               6.0 , 2.5;
    // This function generates the waypoints and map bounds for the interpolation of the global trajectory
        /*
    way_points.resize(4, 2);
    way_points << 0  , 0,
            2.5, 0,
            2.5, -6.75,
            7  , -6.75;

    map_bounds.resize(9, 2);
    map_bounds << -1*1, 1.5*1,
            5*1, 1.5*1,
            5*1,-5.5*1,
            9*1,-5.5*1,
            9*1,-8.5*1,
            1*1,-8.5*1,
            1*1,-1.5*1,
            -1*1,-1.5*1,
            -1*1, 1.5*1;
        */

       /* way_points.resize(4, 2);
        way_points << 0, 0,
                                  5, 0,
                                  5, 3,
                                  10, 3;
        map_bounds.resize(9, 2);
    map_bounds << -1, -1,
                                  6, -1,
                                  6, 2,
                                  11, 2,
                                  11, 4,
                                  4, 4,
                                  4, 1,
                                  -1, 1,
                                  -1, -1; */

    
    // way_points.resize(astar_path_.size(), 2);


    // for (int i = 0; i < astar_path_.size(); ++i)
    // {
    //     way_points.row(i) = astar_path_[i].head<2>();
    // }
    n_point = way_points.rows(); // Number of points
    n_line = n_point - 1;        // Number of linear segments
    n_corner = n_point - 2;      // Number of corner segments

    std::cout << "The way points are :\n" << way_points << std::endl;
}

void Global_traj_class::way_point_generator_01() {
    // This function generates the waypoints and map bounds for the interpolation of the global trajectory
    Eigen::VectorXd arc_length;
    Eigen::VectorXd distance_calc;
    Eigen::RowVector2d temp;
    double arc_length_pos = 0;
    double distance_calc_min = 0;
    unsigned int min_index = 0;
    unsigned int n_point_original = 0;

    map_bounds.resize(9, 2);
    map_bounds << -1, 1.5,
            5, 1.5,
            5, -5.5,
            9, -5.5,
            9, -8.5,
            1, -8.5,
            1, -1.5,
            -1, -1.5,
            -1, 1.5;

    way_points_original.resize(4, 2);
    way_points_original << 0, 0,
            2.5, 0,
            2.5, -6.75,
            7, -6.75;

    if (pos_current(0) == way_points_original(0, 0) && pos_current(1) == way_points_original(0, 1))
    {
        way_points.resize(way_points_original.rows(), way_points_original.cols());
        way_points = way_points_original;
    }
    else
    {
        n_point_original = way_points_original.rows(); // Number of way points
        //arc_length.resize(n_point_original);
        arc_length = Eigen::VectorXd::Zero(n_point_original);
        //std::cout << "Arc length vector is: \n" << arc_length;
        //distance_calc.resize(n_point_original);
        distance_calc = Eigen::VectorXd::Zero(n_point_original);
        temp = way_points_original.row(0) - pos_current;
        distance_calc(0) = temp.norm();

        for (int i = 1; i < n_point_original; i++)
        {
            temp = way_points_original.row(i) - way_points_original.row(i - 1);
            arc_length(i) = arc_length(i - 1) + temp.norm();

            temp = way_points_original.row(i) - pos_current;
            distance_calc(i) = temp.norm();
        }
        //std::cout << "Arc length vector is: \n" << arc_length;
        distance_calc_min = distance_calc.minCoeff();
        while (min_index < n_point_original - 1)
        {
            if (distance_calc_min == distance_calc(min_index))
            {
                break;
            }
            else
            {
                min_index = min_index + 1;
            }
        }

        // Generation of new way points
        if (min_index == 0)
        {
            temp = way_points_original.row(min_index) - pos_current;
            arc_length_pos = temp.norm();
        }
        else
        {
            temp = way_points_original.row(min_index - 1) - pos_current;
            arc_length_pos = arc_length(min_index - 1) + temp.norm();
        }

        if (arc_length(min_index) > arc_length_pos)
        {
            // the next way point after pos is at min_index
            n_point = n_point_original - min_index + 1;
            way_points.resize(n_point, 2);
            //Block of size (p,q), starting at (i,j): syntax matrix.block(i,j,p,q)
            way_points << pos_current, way_points_original.block(min_index, 0, n_point - 1, 2);
        }
        else
        {
            // the next way point after pos is at min_index + 1
            n_point = n_point_original - min_index;
            way_points.resize(n_point, 2);
            //Block of size (p,q), starting at (i,j): syntax matrix.block(i,j,p,q)
            //std::cout << "The original way points are :\n" << way_points_original << std::endl;
            //std::cout << "The block is :\n" << way_points_original.block(min_index + 1, 0, n_point - 1, 2) << std::endl;
            way_points << pos_current, way_points_original.block(min_index + 1, 0, n_point - 1, 2);
        }

    }
    n_point = way_points.rows(); // Number of points
    n_line = n_point - 1;        // Number of linear segments
    n_corner = n_point - 2;      // Number of corner segments

    //std::cout << "The original way points are :\n" << way_points_original << std::endl;
    //std::cout << "The  modified way points are :\n" << way_points << std::endl;
}

void Global_traj_class::euler_spiral_corner_smoothing_solver() {

    // This function computes the path velocities and euler coefficients required to generate smooth corner trajectories
    // through all way points given a cornering toleranceand robot parameters(i.e., wheel radius, distance from the
    // left / right wheel to the center of the wheel drive axis, angular jerk, acceleration and velocity limits)

    epsilon.resize(n_corner);             // [m]   Cornering tolerance at each corner
    pos_start_corner.resize(n_corner, 2); // [m]   Start point of each corner
    pos_mid_corner.resize(n_corner, 2);   // [m]   Mid point of each corner
    pos_end_corner.resize(n_corner, 2);   // [m]   End point of each corner
    path_vel_corner.resize(n_corner);     // [m/s] Path velocity of each corner
    T_corner.resize(n_corner);            // [s]   Time required to traverse the each corner
    a_euler.resize(n_corner);             // [m]   Euler scaling factor
    Lc.resize(n_corner);                   // [m]   Euler scaling factor
    turn_direction.resize(n_corner);      // [ ]   1: Counter clockwise and -1 : clockwise

    unsigned short int n_bounds = map_bounds.rows();                             // [ ] Number of boarder vertices
    Eigen::VectorXd dist_cost = Eigen::VectorXd::Zero(n_bounds);  // [ ] Evaluation of discost function

    for (unsigned short int i = 0; i < n_corner; i++)
    {
        Eigen::RowVector2d pos_start  = way_points.row(i);   // [m] Way point before the corner
        Eigen::RowVector2d pos_corner = way_points.row(i+1); // [m] Way point at the corner
        Eigen::RowVector2d pos_end    = way_points.row(i+2); // [m] Way point after the corner

        // Calculation of unit vector while approaching the corner
        Eigen::RowVector2d delta_Pcs       = pos_corner - pos_start;
        Eigen::RowVector2d unit_vect_start = delta_Pcs / delta_Pcs.norm();
        double theta_start                  = calculate_angle_from_vector(unit_vect_start);

        // Calculation of unit vector while leaving the corner
        Eigen::RowVector2d delta_Pec     = pos_end - pos_corner;
        Eigen::RowVector2d unit_vect_end = delta_Pec / delta_Pec.norm();
        double theta_end                  = calculate_angle_from_vector(unit_vect_end);

        // Change of coordinate frame from world to local (i.e. cornering start point) frame
        Eigen::Matrix2d R_localw;
        R_localw << cos(theta_start), -sin(theta_start),
                    sin(theta_start), cos(theta_start);

        Eigen::Vector2d temp                = R_localw.transpose() * delta_Pcs.transpose();
        Eigen::RowVector2d pos_corner_local = temp.transpose();
        double theta_end_local               = theta_end - theta_start;


        ////DEBUGGED added part//// -N
        if(theta_end_local > MathConstants::PI)
        {
            theta_end_local = GeneralFunctions::wrapToPi(theta_end_local);
        }
        if(theta_end_local < -MathConstants::PI)
        {
            theta_end_local = GeneralFunctions::wrapToPi(theta_end_local);
        }
        ////////////////////////////////

        double delta_phi;
         if (theta_end_local > 0)
         {
             turn_direction(i) = 1; // Counter clockwise turning
             //delta_phi = theta_end_local;
         }
         else
         {
             turn_direction(i) = -1; // Clockwise turning
             //delta_phi = -1 * theta_end_local;
         }



         delta_phi = std::abs(theta_end_local);            // Absolute of the change in orientation at the corner
         double beta                = PI - delta_phi;         // Angle between the unit vector approaching the corner and the one leaving it
         double theta_epsilon_abs   = delta_phi + 0.5 * beta; // Absolute angle between the cornering bisector and the unit vector approaching the corner
         double theta_epsilon_local = theta_epsilon_abs * turn_direction(i);

         // Calculating the Euler spiral coefficients at the corner mid-point
         double phi_mid = 0.5 * delta_phi;
         Eigen::RowVector2d unit_euler_coordinate_mid = unit_euler_spiral_coordinate_calculator(phi_mid, euler_table);

         // Cornering tolerance calculation
         double a_euler_min = WHEEL_AXIS_DIST * sqrt(2 * PI * phi_mid);                   // Minimum allowable Euler scaling factor
         double epsilon_min = a_euler_min * unit_euler_coordinate_mid(1) / sin(beta / 2); // Minimum possible cornering tolerance
        //  epsilon(i) = (std::min(epsilon_max, epsilon_min) + epsilon_max) / 2;  ////////////////////////ALTERED///////////////////
        epsilon(i) = epsilon_min + 0.4;
         Lc(i)      = epsilon(i) * ((sin(beta / 2) / unit_euler_coordinate_mid(1)) * unit_euler_coordinate_mid(0) + cos(beta / 2));   // [m] Euclidian distance
         a_euler(i) = (epsilon(i) * sin(beta / 2)) / unit_euler_coordinate_mid(1);
         Eigen::RowVector2d Lc_check;
         double Lc_scale = 0.4; // the Lc scale factor (0 < factor < 0.5) is a rule of thumb selection
         Lc_check << delta_Pec.norm(), delta_Pcs.norm();
         if (Lc(i) > Lc_scale * Lc_check.minCoeff())
         {
             // Corner is too big
             Lc(i)      = Lc_scale * Lc_check.minCoeff();
             epsilon(i) = Lc(i) / ((sin(beta / 2) / unit_euler_coordinate_mid(1)) * unit_euler_coordinate_mid(0) + cos(beta / 2));
             a_euler(i) = (epsilon(i) * sin(beta / 2)) / unit_euler_coordinate_mid(1); // [m] Scaling factor used in the Euler spiral definition
         }

         if (a_euler_min > a_euler(i))
         {
             a_euler(i) = a_euler_min;
             epsilon(i) = a_euler_min * unit_euler_coordinate_mid(1) / (sin(beta / 2));
             Lc(i) = epsilon(i) * ((sin(beta / 2) / unit_euler_coordinate_mid(1)) * unit_euler_coordinate_mid(0) + cos(beta / 2));   // [m] Euclidian distance
         }

         // Corner points in local coordinate frame
         Eigen::RowVector2d unit_vect_temp; //
         unit_vect_temp << -1, 0;
         Eigen::RowVector2d pos_start_corner_local = pos_corner_local + Lc(i) * unit_vect_temp;         // [m] Corner starting point

         unit_vect_temp << cos(theta_epsilon_local), sin(theta_epsilon_local);
         Eigen::RowVector2d pos_epsilon_local      = pos_corner_local + epsilon(i) * unit_vect_temp; // [m] Corner mid point

         unit_vect_temp << cos(theta_end_local), sin(theta_end_local);
         Eigen::RowVector2d pos_end_corner_local = pos_corner_local + Lc(i) * unit_vect_temp;           // [m] Corner ending point

         // Corner points in world coordinate frame
         temp = R_localw * pos_start_corner_local.transpose();
         pos_start_corner.row(i) =  temp.transpose() + pos_start; // [m] Corner starting point

         temp = R_localw * pos_epsilon_local.transpose();
         pos_mid_corner.row(i) = temp.transpose() + pos_start;    // [m] Corner mid point

         temp = R_localw * pos_end_corner_local.transpose();
         pos_end_corner.row(i) = temp.transpose() + pos_start;    // [m] Corner ending point

         // Path velocity calculation

         // By saturating angular velocity limits
         double v_vel = ANG_VEL_LIM * WHEEL_RADIUS / (1 + (WHEEL_AXIS_DIST / a_euler(i)) * sqrt(2 * PI * phi_mid)); // [m/s]
         // By saturating angular acceleration limits
         double v_acc = sqrt(ANG_ACC_LIM * WHEEL_RADIUS * a_euler(i) * a_euler(i) / (PI * WHEEL_AXIS_DIST)); // [m/s]

         Eigen::RowVector3d path_vel_comp; // Path velocity comparison
         path_vel_comp << v_vel, v_acc, PATH_VEL_LIM;

         path_vel_corner(i) = path_vel_comp.minCoeff();

         // Trajectory interpolation
         double s_mid = a_euler(i) * sqrt(2 * phi_mid / PI); // [m] arc length from start point to mid point of the corner
         T_corner(i) = 2 * s_mid / path_vel_corner(i);       // [s] Time required to traverse the each corner

        //    std::cout << "pos_start_corner:\t" << pos_start_corner.row(i) << std::endl;
        //    std::cout << "pos_end_corner:\t" << pos_end_corner.row(i) << std::endl;
        //    std::cout << "Lc:\t" << Lc(i) << std::endl;
    }

}

Eigen::RowVector4d Global_traj_class::jlap_acceleration_phase_displacement_calculator(double &path_vel_start, double &path_acc_start, double &path_vel_end, double &path_acc_end) {

    // This function computes the displacement and the time intervals of a JLAP trajectory during an acceleration phase

    double delta_vel = path_vel_end - path_vel_start; // [m/s] Change in velocity
    double path_acc_max, path_jerk;
    double sign_delta_vel;

    if (delta_vel > 0)
    {
        sign_delta_vel = 1;
    }
    else if (delta_vel < 0)
    {
        sign_delta_vel = -1;
    }
    else
    {
        sign_delta_vel = 0;
    }
    path_acc_max = PATH_ACC_LIM * sign_delta_vel;
    path_jerk    = PATH_JERK_LIM * sign_delta_vel;

    double T1_a = 0;	// T1_a, T2_a and T3_a are time intervals during the positive, zero and negative jerk phases, respectively
    double T2_a = 0;
    double T3_a = 0;

    if (std::abs(delta_vel) > 0)
    {
        T1_a = (path_acc_max - path_acc_start) / path_jerk;
        T3_a = (path_acc_end - path_acc_max) / (-path_jerk);
        T2_a = (1 / path_acc_max) * (delta_vel - (1.0 / 2) * T1_a * (path_acc_max + path_acc_start) - (1.0 / 2) * T3_a * (path_acc_max + path_acc_end));

        if (T2_a < 0)
        {
            path_acc_max = sqrt((1.0 / 2) * (pow(path_acc_start, 2) + pow(path_acc_end, 2) + 2 * path_jerk * delta_vel));
            path_acc_max = path_acc_max * sign_delta_vel;
            T1_a = (path_acc_max - path_acc_start) / path_jerk;
            T3_a = (path_acc_end - path_acc_max) / (-path_jerk);
            T2_a = 0;
        }
    }

    double acc_1a = path_jerk * T1_a;                                           // [m/s^2] Acceleration at the end of time interval T1_a
    double vel_1a = path_vel_start + (1.0 / 2) * path_jerk * pow(T1_a, 2);        // [m/s] Velocity at the end of time interval T1_a
    double S_1a   = path_vel_start * T1_a + (1.0 / 6) * path_jerk * pow(T1_a, 3); // [m] Displacement at the end of time interval T1_a

    double vel_2a = vel_1a + acc_1a * T2_a;
    double S_2a   = S_1a + vel_1a * T2_a + (1.0 / 2) * acc_1a * pow(T2_a, 2);

    double vel_3a = vel_2a + acc_1a * T3_a - (1.0 / 2) * path_jerk * pow(T3_a, 2);
    double   S_3a = S_2a + vel_2a * T1_a + (1.0 / 2) * acc_1a * pow(T1_a, 2) - (1.0 / 6) * path_jerk * pow(T1_a, 3);


    Eigen::RowVector4d jlap_result;
    jlap_result << T1_a, T2_a, T3_a, std::abs(S_3a);
    return jlap_result;
}

Eigen::VectorXd Global_traj_class::point_to_point_motion_single_line_jlap_solver(Eigen::RowVector2d& pos_start, double& path_vel_start, double& path_acc_start, Eigen::RowVector2d& pos_end, double& path_vel_end, double& path_acc_end) {

    // This function computes time intervals for jerk limited(JLAP) trajectories for point to point motion
    // in a linear path.
    Eigen::RowVector2d disp_vector = pos_start - pos_end;
    double line_length = disp_vector.norm(); // [m] Line length

    // Initialization of maximum path velocity
    Eigen::RowVector2d temp;
    temp << path_vel_start, path_vel_end;

    double path_vel_max = 1.1 * temp.maxCoeff();
    //std::cout << "path vel = \n" << path_vel_max << std::endl;

    if (path_vel_max > PATH_VEL_LIM) {
        path_vel_max = PATH_VEL_LIM;
    }

    double path_acc_s = 0; // [m/s^2] path acceleration at the start of the deceleration phase
    double path_acc_e = 0;   // [m/s^2] path acceleration at the end of the acceleration phase

    // Acceleration phase(a)
    Eigen::RowVector4d data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start, path_acc_start, path_vel_max, path_acc_e);

    // Deceleration phase(d)
    Eigen::RowVector4d data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_s, path_vel_end, path_acc_end);

    double displacement = data_a(3) + data_d(3); // [m] Total distance covered during the velocity transition

    // Rercusive determination of the maximum path velocity along the line
    if (displacement > line_length)
    {
        while (displacement > line_length&& path_vel_max > path_vel_end)
        {
            temp << path_vel_max - PATH_VEL_STEP, path_vel_end;
            path_vel_max = temp.maxCoeff();
            data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start, path_acc_start, path_vel_max, path_acc_e);
            data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_s, path_vel_end, path_acc_end);
            displacement = data_a(3) + data_d(3);
        }
    }
    else {
        while (displacement < line_length && path_vel_max < PATH_VEL_LIM)
        {
            temp << path_vel_max + PATH_VEL_STEP, PATH_VEL_LIM;
            path_vel_max = temp.minCoeff();
            data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start, path_acc_start, path_vel_max, path_acc_e);
            data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_s, path_vel_end, path_acc_end);
            displacement = data_a(3) + data_d(3);
        }
    }
    // Determination of JLAP time intervals
    data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start, path_acc_start, path_vel_max, path_acc_e);
    data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_s, path_vel_end, path_acc_end);
    displacement = data_a(3) + data_d(3);

    double cruise_dist = line_length - displacement;
    temp << cruise_dist, 0;
    temp = temp / path_vel_max;
    double Tc = temp.maxCoeff(); // [m/s] Cruise velocity time interval

    Eigen::VectorXd jlap_data; // [T1a, T2a, T3a, Tc, T1d, T2d, T3d]
    jlap_data.resize(7);
    jlap_data << data_a(0), data_a(1), data_a(2), Tc, data_d(0), data_d(1), data_d(2);

    return jlap_data;

}

void Global_traj_class::point_to_point_motion_jlap_solver() {

    // This function computes time intervals for jerk limited(JLAP) trajectories for point to point motions
    // in a piece linear path.The inputs are series of start& end pointsand the cornering
    // path velocities in the interconnecting corners.

    T_jlap.resize(n_line, 7);

    pos_start.resize(n_corner + 1, 2);
    pos_end.resize(n_corner + 1, 2);

    pos_start << way_points.row(0), pos_end_corner;
    pos_end << pos_start_corner, way_points.row(n_point - 1);

    //std::cout << "Way point:\n"<<way_points << std::endl;
    //std::cout << "Corner start point:\n" << pos_start_corner << std::endl;
    //std::cout << "Corner end point:\n" << pos_end_corner << std::endl;
    //std::cout << "Line start point:\n" << pos_start << std::endl;
    //std::cout << "Line end point:\n" << pos_end << std::endl;

    path_vel_start.resize(n_corner + 1);
    path_vel_end.resize(n_corner + 1);

    path_vel_start << path_vel_current, path_vel_corner;
    path_vel_end   << path_vel_corner, 0;

    for (unsigned short int i = 0; i < n_line; i++)
    {
        Eigen::RowVector2d disp_vector = pos_end.row(i) - pos_start.row(i);
        double line_length = disp_vector.norm(); // [m] Line length

        // Initialization of maximum path velocity
        Eigen::RowVector2d temp;
        temp << path_vel_start(i), path_vel_end(i);

        double path_vel_max = 1.1 * temp.maxCoeff();
        //std::cout << "Path Vel max:\t" << path_vel_max << std::endl;
        if (path_vel_max > PATH_VEL_LIM) {
            path_vel_max = PATH_VEL_LIM;
        }

        double path_acc_start = 0; // [m/s^2] Acceleration at the initial acceleration phase
        double path_acc_end   = 0; // [m/s^2] Acceleration at the end of the acceleration phase

        // Acceleration phase(a)
        Eigen::RowVector4d data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start(i), path_acc_start, path_vel_max, path_acc_end);

        // Deceleration phase(d)
        Eigen::RowVector4d data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_start, path_vel_end(i), path_acc_end);

        double displacement = data_a(3) + data_d(3); // [m] Total distance covered during the velocity transition
        //std::cout << "Displacement:\t" << displacement << std::endl;
        // Rercusive determination of the maximum path velocity along the line
        if (displacement > line_length)
        {
            while (displacement > line_length && path_vel_max > path_vel_end(i))
            {
                temp << path_vel_max - PATH_VEL_STEP, path_vel_end(i);
                path_vel_max = temp.maxCoeff();
                data_a       = jlap_acceleration_phase_displacement_calculator(path_vel_start(i), path_acc_start, path_vel_max, path_acc_end);
                data_d       = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_start, path_vel_end(i), path_acc_end);
                displacement = data_a(3) + data_d(3);
            }
        }
        else {
            while (displacement < line_length && path_vel_max < PATH_VEL_LIM)
            {
                temp << path_vel_max + PATH_VEL_STEP, PATH_VEL_LIM;
                path_vel_max = temp.minCoeff();
                data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start(i), path_acc_start, path_vel_max, path_acc_end);
                data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_start, path_vel_end(i), path_acc_end);
                displacement = data_a(3) + data_d(3);
            }
        }
        // Determination of JLAP time intervals
        data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start(i), path_acc_start, path_vel_max, path_acc_end);
        data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_start, path_vel_end(i), path_acc_end);
        displacement = data_a(3) + data_d(3);

        double cruise_dist = line_length - displacement;
        temp << cruise_dist, 0;
        temp = temp / path_vel_max;
        double Tc = temp.maxCoeff(); // [m/s] Cruise velocity time interval

        T_jlap.row(i) << data_a(0), data_a(1), data_a(2), Tc, data_d(0), data_d(1), data_d(2);
    }
    //std::cout << "JLAP time intervals:\n" <<T_jlap<< std::endl;

}

void Global_traj_class::point_to_point_single_line_trajectory_interpolator() {

    Eigen::RowVector2d temp;
    Eigen::Vector2d temp_vertical;

    n_line = 1;

    Eigen::RowVector2d pos_start = way_points.row(0);
    Eigen::RowVector2d pos_end = way_points.row(1);

    Eigen::RowVector2d disp_vector = pos_end.row(0) - pos_start.row(0);
    unit_vector.resize(n_line, 2);
    unit_vector.row(0) = disp_vector / disp_vector.norm();

    double path_acc_start   = 0;
    double path_vel_start   = path_vel_current;
    double path_acc_end     = 0;
    double path_vel_end     = 0;

    Eigen::VectorXd T_jlap;
    T_jlap.resize(7); // [T1a, T2a, T3a, Tc, T1d, T2d, T3d]
    T_jlap = point_to_point_motion_single_line_jlap_solver(pos_start, path_vel_start, path_acc_start, pos_end, path_vel_end, path_acc_end);

    double T_total = T_jlap.sum();                       // [s] Total time
    Ns = rint(T_total / SAMPLING_TIME); // Number of data samples

    // Trajectory data initialization
    t.resize(Ns);          // [s] time instant
    ang_acc.resize(Ns, 2); // [rad/s^2] Angular acceleration
    ang_vel.resize(Ns, 2); // [rad/s] Angular velocity

    acc.resize(Ns, 2);     // [m/s] Axial acceleration
    vel.resize(Ns, 2);     // [m/s] Axial velocities
    pos.resize(Ns, 2);     // [m] robot position

    path_acc.resize(Ns);    // [m/s] path acceleration
    path_vel.resize(Ns);    // [m/s] path velocity
    path_length.resize(Ns); // [m]
    phi.resize(Ns);         // [rad] Robot orientation
    dphi.resize(Ns);        // [rad/s] First derivative of robot orientation
    ddphi.resize(Ns);       // [rad/s^2] Second derivative of robot orientation

    max_ang_acc.resize(n_corner); // [rad/s^2] Maximum angular acceleration
    max_ang_vel.resize(n_corner); // [rad/s] Maximum angular velocity

    // Data initialization
    t.setZero();          // [s] time instant
    ang_acc.setZero(); // [rad/s^2] Angular acceleration
    ang_vel.setZero(); // [rad/s] Angular velocity

    acc.setZero();     // [m/s] Axial acceleration
    vel.setZero();     // [m/s] Axial velocities
    pos.setZero();     // [m] robot position
    pos.row(0) << pos_start;

    path_acc.setZero();    // [m/s] path acceleration
    path_vel.setZero();    // [m/s] path velocity
    path_vel.row(0) << path_vel_start;

    temp << 1, 1;
    ang_acc.row(0) = path_acc(0) / WHEEL_RADIUS * temp;
    ang_vel.row(0) = path_vel(0) / WHEEL_RADIUS * temp;

    path_length.setZero(); // [m]
    phi.setZero();         // [rad] Robot orientation
    dphi.setZero();        // [rad/s] First derivative of robot orientation
    ddphi.setZero();       // [rad/s^2] Second derivative of robot orientation

    // Interpolation
    double T_clock = 0;
    unsigned int N_count = 0;
    unsigned int j_c = 0;
    unsigned int j_t = 0;
    double z = 0;

    double T1_a, T2_a, T3_a, Tc, T1_d, T2_d, T3_d;
    double T_a, T_d;

    Eigen::RowVector2d vel_not, pos_not, acc_1a, vel_1a, pos_1a, vel_2a, pos_2a, vel_3a, pos_3a, pos_c, acc_1d, vel_1d, pos_1d, vel_2d, pos_2d;

    // Line segment interpolation

    // Read Jerk limited acceleration profile data
    T1_a = T_jlap(0);
    T2_a = T_jlap(1);
    T3_a = T_jlap(2);
    Tc = T_jlap(3);
    T1_d = T_jlap(4);
    T2_d = T_jlap(5);
    T3_d = T_jlap(6);

    T_a = T1_a + T2_a + T3_a;
    T_d = T1_d + T2_d + T3_d;

    vel_not = path_vel_current * unit_vector.row(0);
    pos_not = pos.row(0);

    acc_1a = PATH_JERK_LIM * (T1_a)*unit_vector.row(0);
    vel_1a = vel_not + (1.0 / 2) * PATH_JERK_LIM * pow(T1_a, 2) * unit_vector.row(0);
    pos_1a = pos_not + vel_not * T1_a + (1.0 / 6) * PATH_JERK_LIM * pow(T1_a, 3) * unit_vector.row(0);

    vel_2a = vel_1a + acc_1a * T2_a;
    pos_2a = pos_1a + vel_1a * T2_a + (1.0 / 2) * acc_1a * pow(T2_a, 2);

    vel_3a = vel_2a + acc_1a * T3_a - (1.0 / 2) * PATH_JERK_LIM * pow(T3_a, 2) * unit_vector.row(0);
    pos_3a = pos_2a + vel_2a * T3_a + (1.0 / 2) * acc_1a * pow(T3_a, 2) - (1.0 / 6) * PATH_JERK_LIM * pow(T3_a, 3) * unit_vector.row(0);

    pos_c = pos_3a + vel_3a * Tc;

    acc_1d = -PATH_JERK_LIM * T1_d * unit_vector.row(0);
    vel_1d = vel_3a - (1.0 / 2) * PATH_JERK_LIM * pow(T1_d, 2) * unit_vector.row(0);
    pos_1d = pos_c + vel_3a * T1_d - (1.0 / 6) * PATH_JERK_LIM * pow(T1_d, 3) * unit_vector.row(0);

    vel_2d = vel_1d + acc_1d * T2_d;
    pos_2d = pos_1d + vel_1d * T2_d + (1.0 / 2) * acc_1d * pow(T2_d, 2);

    for (unsigned int j = 1; j < Ns; j++) {

        t(j_c + j) = t(j_c) + j * SAMPLING_TIME;

        if (t(j_c + j) <= T1_a)
        {
            z = t(j_c + j) - T_clock;

            acc.row(j_c + j) = z * PATH_JERK_LIM * unit_vector.row(0);
            vel.row(j_c + j) = vel_not + (1.0 / 2) * pow(z, 2) * PATH_JERK_LIM * unit_vector.row(0);
            pos.row(j_c + j) = pos_not + z * vel_not + (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM * unit_vector.row(0);

            temp = acc.row(j_c + j);
            path_acc(j_c + j) = temp.norm();
            temp = vel.row(j_c + j);
            path_vel(j_c + j) = temp.norm();
            path_length(j_c + j) = path_length(j_c) + z * path_vel(j_c) + (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM;

            //std::cout << "path vel= \t" << path_vel(j_c + j) << std::endl;
            temp << 1, 1;
            ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
            ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
            ddphi(j_c + j) = 0;
            dphi(j_c + j) = 0;

            temp = unit_vector.row(0);
            phi(j_c + j) = calculate_angle_from_vector(temp);

        }
        else if (t(j_c + j) > T1_a&& t(j_c + j) <= T1_a + T2_a) {

            z = t(j_c + j) - (T1_a);
            j_t = int((T1_a) / SAMPLING_TIME);

            acc.row(j_c + j) = acc_1a;
            vel.row(j_c + j) = vel_1a + acc_1a * z;
            pos.row(j_c + j) = pos_1a + z * vel_1a + (1.0 / 2) * pow(z, 2) * acc_1a;

            temp = acc.row(j_c + j);
            path_acc(j_c + j) = temp.norm();
            temp = vel.row(j_c + j);
            path_vel(j_c + j) = temp.norm();
            path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t) + (1.0 / 2) * pow(z, 2) * path_acc(j_t);

            temp << 1, 1;
            ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
            ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
            ddphi(j_c + j) = 0;
            dphi(j_c + j) = 0;

            temp = unit_vector.row(0);
            phi(j_c + j) = calculate_angle_from_vector(temp);
        }
        else if (t(j_c + j) > T1_a + T2_a && t(j_c + j) <= T_a) {

            z = t(j_c + j) - (T1_a + T2_a);
            j_t = int((T1_a + T2_a) / SAMPLING_TIME);

            acc.row(j_c + j) = acc_1a - z * PATH_JERK_LIM * unit_vector.row(0);
            vel.row(j_c + j) = vel_2a + z * acc_1a - (1.0 / 2) * pow(z, 2) * PATH_JERK_LIM * unit_vector.row(0);
            pos.row(j_c + j) = pos_2a + z * vel_2a + (1.0 / 2) * pow(z, 2) * acc_1a - (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM * unit_vector.row(0);

            temp = acc.row(j_c + j);
            path_acc(j_c + j) = temp.norm();
            temp = vel.row(j_c + j);
            path_vel(j_c + j) = temp.norm();
            path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t) + (1.0 / 2) * pow(z, 2) * path_acc(j_t) - (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM;

            temp << 1, 1;
            ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
            ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
            ddphi(j_c + j) = 0;
            dphi(j_c + j) = 0;

            temp = unit_vector.row(0);
            phi(j_c + j) = calculate_angle_from_vector(temp);
        }
        else if (t(j_c + j) > T_a&& t(j_c + j) <= T_a + Tc) {

            z = t(j_c + j) - (T_a);
            j_t = int((T_a) / SAMPLING_TIME);

            acc.row(j_c + j) << 0, 0;
            vel.row(j_c + j) = vel_3a;
            pos.row(j_c + j) = pos_3a + z * vel_3a;

            temp = acc.row(j_c + j);
            path_acc(j_c + j) = temp.norm();
            temp = vel.row(j_c + j);
            path_vel(j_c + j) = temp.norm();
            path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t);

            temp << 1, 1;
            ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
            ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
            ddphi(j_c + j) = 0;
            dphi(j_c + j) = 0;

            temp = unit_vector.row(0);
            phi(j_c + j) = calculate_angle_from_vector(temp);
        }
        else if (t(j_c + j) > T_a + Tc && t(j_c + j) <= T_a + Tc + T1_d) {

            z = t(j_c + j) - (T_a + Tc);
            j_t = int((T_a + Tc) / SAMPLING_TIME);

            acc.row(j_c + j) = -z * PATH_JERK_LIM * unit_vector.row(0);
            vel.row(j_c + j) = vel_3a - (1.0 / 2) * pow(z, 2) * PATH_JERK_LIM * unit_vector.row(0);
            pos.row(j_c + j) = pos_c + z * vel_3a - (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM * unit_vector.row(0);

            temp = -acc.row(j_c + j);
            path_acc(j_c + j) = temp.norm();
            temp = vel.row(j_c + j);
            path_vel(j_c + j) = temp.norm();
            path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t) - (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM;

            temp << 1, 1;
            ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
            ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
            ddphi(j_c + j) = 0;
            dphi(j_c + j) = 0;

            temp = unit_vector.row(0);
            phi(j_c + j) = calculate_angle_from_vector(temp);
        }
        else if (t(j_c + j) > T_a + Tc + T1_d && t(j_c + j) <= T_a + Tc + T1_d + T2_d) {

            z = t(j_c + j) - (T_a + Tc + T1_d);
            j_t = int((T_a + Tc + T1_d) / SAMPLING_TIME);

            acc.row(j_c + j) = acc_1d;
            vel.row(j_c + j) = vel_1d + acc_1d * z;
            pos.row(j_c + j) = pos_1d + z * vel_1d + (1.0 / 2) * pow(z, 2) * acc_1d;

            temp = -acc.row(j_c + j);
            path_acc(j_c + j) = temp.norm();
            temp = vel.row(j_c + j);
            path_vel(j_c + j) = temp.norm();
            path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t) + (1.0 / 2) * pow(z, 2) * path_acc(j_t);

            temp << 1, 1;
            ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
            ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
            ddphi(j_c + j) = 0;
            dphi(j_c + j) = 0;

            temp = unit_vector.row(0);
            phi(j_c + j) = calculate_angle_from_vector(temp);
        }
        else {
            if (t(j_c + j) < T_total) {

                z = t(j_c + j) - (T_a + Tc + T1_d + T2_d);
                j_t = int((T_a + Tc + T1_d + T2_d) / SAMPLING_TIME);

                acc.row(j_c + j) = acc_1d + z * PATH_JERK_LIM * unit_vector.row(0);
                vel.row(j_c + j) = vel_2d + z * acc_1d + (1.0 / 2) * pow(z, 2) * PATH_JERK_LIM * unit_vector.row(0);
                pos.row(j_c + j) = pos_2d + z * vel_2d + (1.0 / 2) * pow(z, 2) * acc_1d + (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM * unit_vector.row(0);

                temp = -acc.row(j_c + j);
                path_acc(j_c + j) = temp.norm();
                temp = vel.row(j_c + j);
                path_vel(j_c + j) = temp.norm();
                path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t) + (1.0 / 2) * pow(z, 2) * path_acc(j_t) + (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM;

                temp << 1, 1;
                ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
                ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
                ddphi(j_c + j) = 0;
                dphi(j_c + j) = 0;

                temp = unit_vector.row(0);
                phi(j_c + j) = calculate_angle_from_vector(temp);
                //std::cout << "end line path vel = " << path_vel.row(j_c + j) << std::endl;
            }
            else {
                acc.row(j_c + j) << 0, 0;
                vel.row(j_c + j) << 0, 0;
                pos.row(j_c + j) = pos_end.row(n_line - 1);

                path_acc(j_c + j) = 0;
                path_vel(j_c + j) = 0;

                path_length(j_c + j) = path_length(j_t);

                temp << 1, 1;
                ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
                ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
                ddphi(j_c + j) = 0;
                dphi(j_c + j) = 0;

                temp = unit_vector.row(0);
                phi(j_c + j) = calculate_angle_from_vector(temp);
            }

        }
        //std::cout << "variable z = /t" << z<< std::endl;
        //std::cout << pos.row(j_c + j) << std::endl;
    }
}

void Global_traj_class::trajectory_interpolator() {

    Eigen::RowVector2d temp;
    Eigen::Vector2d temp_vertical;

    unit_vector.resize(n_line, 2);
    Eigen::RowVector2d disp_vector;
    double line_length;

    for (int i = 0; i < n_line; i++)
    {
        disp_vector = pos_end.row(i) - pos_start.row(i);
        line_length = disp_vector.norm(); // [m] Line length
        unit_vector.row(i) = (pos_end.row(i) - pos_start.row(i)) / line_length;
    }
    // std::cout << "Unit vector:\n" << unit_vector << std::endl;
    // Euler corner smoothing paramters
    double theta_start;
    Eigen::Matrix2d R_matrix; // Rotation matrix from local to world (or map) frame
    double arc_length;
    double arc_length_mid; // Arc length from the start to the mid point of the arc
    double phi_mid_local; // Orientation angle at the mid point of the arc with respect to the local coordinate frame

    Eigen::RowVector2d acc_local;             // [m/s^2] Axial accelerations
    Eigen::RowVector2d vel_local;             // [m/s] Axial velocities
    Eigen::RowVector2d pos_local;             // [m] robot position
    Eigen::RowVector2d unit_euler_coordinate; // Coordinates of a point on a unit Euler spiral curve

    double phi_local, phi_abs; // [rad] Robot orientation
    double dphi_local;         // [rad/s] First derivative of robot orientation
    double ddphi_local;        // [rad/s^2] Second derivative of robot orientation

    // Trajectory duration calculation
    double T_total         = 0; // [s] Total time
    unsigned int Ns_line   = 0; // Number of samples for linear segments
    unsigned int Ns_corner = 0; // Number of samples for corner segments

    // Corner segments
    for (unsigned int i = 0; i < n_corner; i++) {
        T_total   = T_total + T_corner(i);
        Ns_corner = Ns_corner + rint(T_corner(i) / SAMPLING_TIME);
    }
    // std::cout << "Total time:\t" << T_total << std::endl;
    //std::cout << "T_corner = " << T_corner << std::endl;
    //std::cout << "T_jlap = " << T_jlap << std::endl;
    // Line segments
    for (unsigned int i = 0; i < n_line; i++) {
        double T_line = 0;
        for (unsigned short int j = 0; j < T_jlap.cols(); j++) {
            T_total = T_total + T_jlap(i, j);
            T_line  = T_line + T_jlap(i, j);
        }
        Ns_line = Ns_line + rint(T_line / SAMPLING_TIME);
    }
    // Trajectory data initialization
    Ns = Ns_line + Ns_corner + 1; // Number of data samples

    t.resize(Ns);          // [s] time instant
    ang_acc.resize(Ns, 2); // [rad/s^2] Angular acceleration
    ang_vel.resize(Ns, 2); // [rad/s] Angular velocity

    acc.resize(Ns, 2);     // [m/s] Axial acceleration
    vel.resize(Ns, 2);     // [m/s] Axial velocities
    pos.resize(Ns, 2);     // [m] robot position

    path_acc.resize(Ns);    // [m/s] path acceleration
    path_vel.resize(Ns);    // [m/s] path velocity
    path_length.resize(Ns); // [m]
    phi.resize(Ns);         // [rad] Robot orientation
    dphi.resize(Ns);        // [rad/s] First derivative of robot orientation
    ddphi.resize(Ns);       // [rad/s^2] Second derivative of robot orientation

    max_ang_acc.resize(n_corner); // [rad/s^2] Maximum angular acceleration
    max_ang_vel.resize(n_corner); // [rad/s] Maximum angular velocity

    // Data initialization
    t.setZero();          // [s] time instant
    ang_acc.setZero(); // [rad/s^2] Angular acceleration
    ang_vel.setZero(); // [rad/s] Angular velocity

    acc.setZero();     // [m/s] Axial acceleration
    vel.setZero();     // [m/s] Axial velocities
    pos.setZero();     // [m] robot position
    pos.row(0) << pos_current;

    path_acc.setZero();    // [m/s] path acceleration
    path_vel.setZero();    // [m/s] path velocity
    path_vel.row(0) << path_vel_current;

    temp << 1, 1;
    ang_acc.row(0) = path_acc(0) / WHEEL_RADIUS * temp;
    ang_vel.row(0) = path_vel(0) / WHEEL_RADIUS * temp;

    path_length.setZero(); // [m]
    phi.setZero();         // [rad] Robot orientation
    dphi.setZero();        // [rad/s] First derivative of robot orientation
    ddphi.setZero();       // [rad/s^2] Second derivative of robot orientation

    // Interpolation
    double T_clock       = 0;
    unsigned int N_count = 0;
    unsigned int j_c     = 0;
    unsigned int j_t     = 0;
    double z             = 0;

    double T1_a, T2_a, T3_a, Tc, T1_d, T2_d, T3_d;
    double T_a, T_d, T_line;

    Eigen::RowVector2d vel_not;
    vel_not = path_vel_current * unit_vector.row(0);

    Eigen::RowVector2d pos_not, acc_1a, vel_1a, pos_1a, vel_2a, pos_2a, vel_3a, pos_3a, pos_c, acc_1d, vel_1d, pos_1d, vel_2d, pos_2d;



    for (unsigned int i = 0; i < n_line; i++){
        // Line segment interpolation
        // Read Jerk limited acceleration profile data

        T1_a = T_jlap(i, 0);
        T2_a = T_jlap(i, 1);
        T3_a = T_jlap(i, 2);
        Tc   = T_jlap(i, 3);
        T1_d = T_jlap(i, 4);
        T2_d = T_jlap(i, 5);
        T3_d = T_jlap(i, 6);

        //std::cout << "T JLAP:\t" << T_jlap << std::endl;

        T_a = T1_a + T2_a + T3_a;
        T_d = T1_d + T2_d + T3_d;

        T_line = T_a + Tc + T_d;
        Ns_line = rint(T_line / SAMPLING_TIME);

        if (i > 0) {
            vel_not = path_vel_corner(i - 1) * unit_vector.row(i);
        }
        pos_not = pos_start.row(i);

        acc_1a = PATH_JERK_LIM * (T1_a) * unit_vector.row(i);
        vel_1a = vel_not + (1.0 / 2) * PATH_JERK_LIM * pow(T1_a, 2) * unit_vector.row(i);
        pos_1a = pos_not + vel_not * T1_a + (1.0 / 6) * PATH_JERK_LIM * pow(T1_a, 3) * unit_vector.row(i);

        vel_2a = vel_1a + acc_1a * T2_a;
        pos_2a = pos_1a + vel_1a * T2_a + (1.0 / 2) * acc_1a * pow(T2_a, 2);

        vel_3a = vel_2a + acc_1a * T3_a - (1.0 / 2) * PATH_JERK_LIM * pow(T3_a, 2) * unit_vector.row(i);
        pos_3a = pos_2a + vel_2a * T3_a + (1.0 / 2) * acc_1a * pow(T3_a, 2) - (1.0 / 6) * PATH_JERK_LIM * pow(T3_a, 3) * unit_vector.row(i);

        pos_c = pos_3a + vel_3a * Tc;

        acc_1d = -PATH_JERK_LIM * T1_d * unit_vector.row(i);
        vel_1d = vel_3a - (1.0 / 2) * PATH_JERK_LIM * pow(T1_d, 2) * unit_vector.row(i);
        pos_1d = pos_c + vel_3a * T1_d - (1.0 / 6) * PATH_JERK_LIM * pow(T1_d, 3) * unit_vector.row(i);

        vel_2d = vel_1d + acc_1d * T2_d;
        pos_2d = pos_1d + vel_1d * T2_d + (1.0 / 2) * acc_1d * pow(T2_d, 2);

        for (unsigned int j = 1; j < Ns_line + 1; j++){

            t(j_c + j) = t(j_c) + j * SAMPLING_TIME;

            if (t(j_c + j) <= T_clock + T1_a)
            {
                z = t(j_c + j) - T_clock;

                acc.row(j_c + j) = z * PATH_JERK_LIM * unit_vector.row(i);
                vel.row(j_c + j) = vel_not + (1.0 / 2) * pow(z, 2) * PATH_JERK_LIM * unit_vector.row(i);
                pos.row(j_c + j) = pos_not + z * vel_not + (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM * unit_vector.row(i);

                temp                 = acc.row(j_c + j);
                path_acc(j_c + j)    = temp.norm();
                temp                 = vel.row(j_c + j);
                path_vel(j_c + j)    = temp.norm();
                path_length(j_c + j) = path_length(j_c) + z * path_vel(j_c) + (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM;
                //std::cout << "Path Vel:\t" << path_vel(j_c + j) << std::endl;
                temp << 1, 1;
                ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
                ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
                ddphi(j_c + j)       = 0;
                dphi(j_c + j)        = 0;

                temp                 = unit_vector.row(i);
                phi(j_c + j)         = calculate_angle_from_vector(temp);

            }
            else if (t(j_c + j) > T_clock + T1_a && t(j_c + j) <= T_clock + T1_a + T2_a){

                z   = t(j_c + j) - (T_clock + T1_a);

                j_t = int((T_clock + T1_a) / SAMPLING_TIME);

                acc.row(j_c + j) = acc_1a;
                vel.row(j_c + j) = vel_1a + acc_1a * z;
                pos.row(j_c + j) = pos_1a + z * vel_1a + (1.0 / 2) * pow(z, 2) * acc_1a;

                temp                 = acc.row(j_c + j);
                path_acc(j_c + j)    = temp.norm();
                temp                 = vel.row(j_c + j);
                path_vel(j_c + j)    = temp.norm();
                path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t) + (1.0 / 2) * pow(z, 2) * path_acc(j_t);

                temp << 1, 1;
                ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
                ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
                ddphi(j_c + j)       = 0;
                dphi(j_c + j)        = 0;

                temp                 = unit_vector.row(i);
                phi(j_c + j)         = calculate_angle_from_vector(temp);
            }
            else if (t(j_c + j) > T_clock + T1_a + T2_a && t(j_c + j) <= T_clock + T_a) {

                z   = t(j_c + j) - (T_clock + T1_a + T2_a);
                j_t = int((T_clock + T1_a + T2_a) / SAMPLING_TIME);

                acc.row(j_c + j) = acc_1a - z * PATH_JERK_LIM * unit_vector.row(i);
                vel.row(j_c + j) = vel_2a + z * acc_1a - (1.0 / 2) * pow(z, 2) * PATH_JERK_LIM * unit_vector.row(i);
                pos.row(j_c + j) = pos_2a + z * vel_2a + (1.0 / 2) * pow(z, 2) * acc_1a - (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM * unit_vector.row(i);

                temp                 = acc.row(j_c + j);
                path_acc(j_c + j)    = temp.norm();
                temp                 = vel.row(j_c + j);
                path_vel(j_c + j)    = temp.norm();
                path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t) + (1.0 / 2) * pow(z, 2) * path_acc(j_t) - (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM;

                temp << 1, 1;
                ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
                ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
                ddphi(j_c + j)       = 0;
                dphi(j_c + j)        = 0;

                temp                 = unit_vector.row(i);
                phi(j_c + j)         = calculate_angle_from_vector(temp);
            }
            else if (t(j_c + j) > T_clock + T_a && t(j_c + j) <= T_clock + T_a + Tc){

                z   = t(j_c + j) - (T_clock + T_a);
                j_t = int((T_clock + T_a) / SAMPLING_TIME);

                acc.row(j_c + j) << 0, 0;
                vel.row(j_c + j) = vel_3a;
                pos.row(j_c + j) = pos_3a + z * vel_3a;

                temp                 = acc.row(j_c + j);
                path_acc(j_c + j)    = temp.norm();
                temp                 = vel.row(j_c + j);
                path_vel(j_c + j)    = temp.norm();
                path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t);

                temp << 1, 1;
                ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
                ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
                ddphi(j_c + j)       = 0;
                dphi(j_c + j)        = 0;

                temp                 = unit_vector.row(i);
                phi(j_c + j)         = calculate_angle_from_vector(temp);
            }
            else if (t(j_c + j) > T_clock + T_a + Tc && t(j_c + j) <= T_clock + T_a + Tc + T1_d){

                z   = t(j_c + j) - (T_clock + T_a + Tc);
                j_t = int((T_clock + T_a + Tc) / SAMPLING_TIME);

                acc.row(j_c + j) = -z * PATH_JERK_LIM * unit_vector.row(i);
                vel.row(j_c + j) = vel_3a - (1.0 / 2) * pow(z, 2) * PATH_JERK_LIM * unit_vector.row(i);
                pos.row(j_c + j) = pos_c + z * vel_3a - (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM * unit_vector.row(i);

                temp                 = -acc.row(j_c + j);
                path_acc(j_c + j)    = temp.norm();
                temp                 = vel.row(j_c + j);
                path_vel(j_c + j)    = temp.norm();
                path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t) - (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM;

                temp << 1, 1;
                ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
                ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
                ddphi(j_c + j)       = 0;
                dphi(j_c + j)        = 0;

                temp                 = unit_vector.row(i);
                phi(j_c + j)         = calculate_angle_from_vector(temp);
            }
            else if (t(j_c + j) > T_clock + T_a + Tc + T1_d && t(j_c + j) <= T_clock + T_a + Tc + T1_d + T2_d){

                z   = t(j_c + j) - (T_clock + T_a + Tc + T1_d);
                j_t = int((T_clock + T_a + Tc + T1_d) / SAMPLING_TIME);

                acc.row(j_c + j) = acc_1d;
                vel.row(j_c + j) = vel_1d + acc_1d * z;
                pos.row(j_c + j) = pos_1d + z * vel_1d + (1.0 / 2) * pow(z, 2) * acc_1d;

                temp                 = -acc.row(j_c + j);
                path_acc(j_c + j)    = temp.norm();
                temp                 = vel.row(j_c + j);
                path_vel(j_c + j)    = temp.norm();
                path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t) + (1.0 / 2) * pow(z, 2) * path_acc(j_t);

                temp << 1, 1;
                ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
                ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
                ddphi(j_c + j)       = 0;
                dphi(j_c + j)        = 0;

                temp                 = unit_vector.row(i);
                phi(j_c + j)         = calculate_angle_from_vector(temp);
            }
            else {
                if (t(j_c + j) < T_total) {

                    z   = t(j_c + j) - (T_clock + T_a + Tc + T1_d + T2_d);
                    j_t = int((T_clock + T_a + Tc + T1_d + T2_d) / SAMPLING_TIME);

                    acc.row(j_c + j) = acc_1d + z * PATH_JERK_LIM * unit_vector.row(i);
                    vel.row(j_c + j) = vel_2d + z * acc_1d + (1.0 / 2) * pow(z, 2) * PATH_JERK_LIM * unit_vector.row(i);
                    pos.row(j_c + j) = pos_2d + z * vel_2d + (1.0 / 2) * pow(z, 2) * acc_1d + (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM * unit_vector.row(i);

                    temp                 = -acc.row(j_c + j);
                    path_acc(j_c + j)    = temp.norm();
                    temp                 = vel.row(j_c + j);
                    path_vel(j_c + j)    = temp.norm();
                    path_length(j_c + j) = path_length(j_t) + z * path_vel(j_t) + (1.0 / 2) * pow(z, 2) * path_acc(j_t) + (1.0 / 6) * pow(z, 3) * PATH_JERK_LIM;

                    temp << 1, 1;
                    ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
                    ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
                    ddphi(j_c + j)       = 0;
                    dphi(j_c + j)        = 0;

                    temp                 = unit_vector.row(i);
                    phi(j_c + j)         = calculate_angle_from_vector(temp);
                }
                else {
                    acc.row(j_c + j) << 0, 0;
                    vel.row(j_c + j) << 0, 0;
                    pos.row(j_c + j) = pos_end.row(n_line - 1);

                    path_acc(j_c + j)    = 0;
                    path_vel(j_c + j)    = 0;
                    path_length(j_c + j) = path_length(j_t);

                    temp << 1, 1;
                    ang_acc.row(j_c + j) = path_acc(j_c + j) / WHEEL_RADIUS * temp;
                    ang_vel.row(j_c + j) = path_vel(j_c + j) / WHEEL_RADIUS * temp;
                    ddphi(j_c + j)       = 0;
                    dphi(j_c + j)        = 0;

                    temp                 = unit_vector.row(i);
                    phi(j_c + j)         = calculate_angle_from_vector(temp);
                }
            }
            //std::cout << "z Variable:\t" << z << std::endl;
        }


        // Corner segment interpolation
        j_c = j_c + Ns_line;
        T_clock = T_clock + Ns_line * SAMPLING_TIME;

        //std::cout << "end line pos = " << pos.row(j_c) << std::endl;


        if (i < n_line - 1){

            // Euler corner smoothing
            temp        << unit_vector(i,0), unit_vector(i, 1);
            theta_start = calculate_angle_from_vector(temp);

            R_matrix << cos(theta_start), -sin(theta_start),
                    sin(theta_start), cos(theta_start); // Rotation matrix from local to world(or map) frame

            arc_length_mid = path_vel_corner(i) * (1.0 / 2) * T_corner(i); // Arc length from the start to the mid point of the arc
            phi_mid_local  = (PI / 2) * pow((arc_length_mid / a_euler(i)), 2); // Orientation at the arc mid point with reference to the start point orientation

            max_ang_vel(i) = (path_vel_corner(i) / WHEEL_RADIUS) * (1.0 + (WHEEL_AXIS_DIST / a_euler(i)) * sqrt(2 * PI * phi_mid_local)); // [rad / s]
            max_ang_acc(i) = (PI * WHEEL_AXIS_DIST / WHEEL_RADIUS) * pow((path_vel_corner(i) / a_euler(i)), 2);

            Ns_corner = rint(T_corner(i) / SAMPLING_TIME);
            ///////////////////////////////////////////////////////////////////////////////////////
            for (unsigned int j = 1; j < Ns_corner + 1; j++)
            {
                t(j_c + j)           = t(j_c) + j * SAMPLING_TIME;
                arc_length           = path_vel_corner(i) * (t(j_c + j) - T_clock);
                path_length(j_c + j) = path_length(j_c) + arc_length;
                path_vel(j_c + j)    = path_vel_corner(i);
                path_acc(j_c + j)    = 0;

                if (t(j_c + j) <= T_clock + (1.0 / 2) * T_corner(i)) // Path approaching mid-point
                {
                    // Local coordinate frame
                    phi_abs     = (PI / 2) * pow((arc_length / a_euler(i)), 2);
                    phi_local   = turn_direction(i) * phi_abs;
                    dphi_local  = turn_direction(i) * PI * path_vel_corner(i) * arc_length / pow(a_euler(i), 2);
                    ddphi_local = turn_direction(i) * PI * pow((path_vel_corner(i) / a_euler(i)), 2);

                    temp                  << cos(phi_local), sin(phi_local);
                    vel_local             = path_vel_corner(i) * temp;
                    temp                  << -sin(phi_local), cos(phi_local);
                    acc_local             = path_vel_corner(i) * dphi_local * temp;
                    unit_euler_coordinate = unit_euler_spiral_coordinate_calculator(phi_abs, euler_table);
                    temp                  << unit_euler_coordinate(0), (unit_euler_coordinate(1) * turn_direction(i));
                    pos_local             = a_euler(i) * temp;

                    // World coordinate frame
                    temp << 1, - 1;
                    ang_acc.row(j_c + j) = (PI * WHEEL_AXIS_DIST / (WHEEL_RADIUS * pow(a_euler(i), 2))) * pow(path_vel_corner(i), 2) * turn_direction(i) * temp;
                    ang_vel(j_c + j, 0)  = (path_vel_corner(i) + WHEEL_AXIS_DIST * dphi_local) / WHEEL_RADIUS;
                    ang_vel(j_c + j, 1)  = (path_vel_corner(i) - WHEEL_AXIS_DIST * dphi_local) / WHEEL_RADIUS;

                    ddphi(j_c + j) = ddphi_local;
                    dphi(j_c + j)  = dphi_local;
                    phi(j_c + j)   = phi_local + theta_start;

                    temp_vertical    = R_matrix * acc_local.transpose();
                    acc.row(j_c + j) = temp_vertical.transpose();
                    temp_vertical    = R_matrix * vel_local.transpose();
                    vel.row(j_c + j) = temp_vertical.transpose();
                    temp_vertical    = R_matrix * pos_local.transpose();
                    pos.row(j_c + j) = temp_vertical.transpose() + pos_start_corner.row(i);

                    //std::cout << "approaching mid-point: corner pos = " << pos.row(j_c+j) << std::endl;
                }
                else // Path leaving the mid-point
                {
                    // Local coordinate frame
                    phi_abs     = phi_mid_local + (PI / pow(a_euler(i), 2)) * (-(1.0 / 2) * pow(arc_length, 2) + 2 * arc_length_mid * arc_length - (3.0 / 2) * pow(arc_length_mid, 2));
                    phi_local   = turn_direction(i) * phi_abs;
                    dphi_local  = turn_direction(i) * PI * path_vel_corner(i) * (2 * arc_length_mid - arc_length) / pow(a_euler(i), 2);
                    ddphi_local = -turn_direction(i) * PI * pow(path_vel_corner(i) / a_euler(i), 2);

                    pos_local   = pos_local + vel_local * SAMPLING_TIME + (1.0 / 2) * acc_local * pow(SAMPLING_TIME, 2);
                    temp        << cos(phi_local), sin(phi_local);
                    vel_local   = path_vel_corner(i) * temp;
                    temp        << -sin(phi_local), cos(phi_local);
                    acc_local   = path_vel_corner(i) * dphi_local * temp;

                    // World coordinate frame
                    temp << 1, - 1;
                    ang_acc.row(j_c + j) = -(PI * WHEEL_AXIS_DIST / (WHEEL_AXIS_DIST * pow(a_euler(i), 2))) * pow(path_vel_corner(i), 2) * turn_direction(i) * temp;
                    ang_vel(j_c + j, 0)  = (path_vel_corner(i) + WHEEL_AXIS_DIST * dphi_local) / WHEEL_RADIUS;
                    ang_vel(j_c + j, 1)  = (path_vel_corner(i) - WHEEL_AXIS_DIST * dphi_local) / WHEEL_RADIUS;

                    ddphi(j_c + j) = ddphi_local;
                    dphi(j_c + j)  = dphi_local;
                    phi(j_c + j)   = phi_local + theta_start;

                    temp_vertical    = R_matrix * acc_local.transpose();
                    acc.row(j_c + j) = temp_vertical.transpose();
                    temp_vertical    = R_matrix * vel_local.transpose();
                    vel.row(j_c + j) = temp_vertical.transpose();
                    temp_vertical    = R_matrix * pos_local.transpose();
                    pos.row(j_c + j) = temp_vertical.transpose() + pos_start_corner.row(i);

                    // std::cout << "leaving the mid-point: corner pos = " << pos.row(j_c + j) << std::endl;

                }
            }

            j_c     = j_c + Ns_corner;
            T_clock = T_clock + Ns_corner * SAMPLING_TIME;

        }
        //std::cout << "end corner pos = " << pos.row(j_c) << std::endl;
        //std::cout << "T_clock = " << T_clock << std::endl;

    }
    //std::cout << "T_clock = " << T_clock << std::endl;
    //std::cout << "final time stamp = " << t(j_c) << std::endl;
    //std::cout << "Acc: " << acc.row(j_c-1) << std::endl;
    //std::cout << "Vel: " << vel.row(j_c-1) << std::endl;
    //std::cout << "Pos: " << pos.row(j_c-1) << std::endl;
    //std::cout << "phi: " << phi(j_c-1) << std::endl;
}

void Global_traj_class::export_global_trajectory() {
    // This function combines all the trajectory information (i.e. time instant [s], X-axis position [m], Y-axis position [m],
    // path velocity [m/s], path acceleration [m/s^2], orientation angle phi [rad], dphi [rad/s], ddphi [rad/s^2]) into one matrix
    // and exports it as a csv file

    // Global trajectory data compilation
    ref_traj.resize(Ns,8);
    ref_traj << t, pos, path_vel, path_acc, phi, dphi, ddphi;

    // Creating/ opening a csv file
    std::ofstream export_file1;
    export_file1.open("global_trajectory.csv",std::ios::out|std::ios::trunc);

    // Write data into the file
    export_file1<< "Time [s], X-Pos [m], Y-Pos [m], phi [rad], dX-Pos [m/s], dY-Pos [m/s], dphi [rad/s], Path Vel [m/s]" << std::endl;
    //	export_file << ref_traj << std::endl;

    for (unsigned int i = 0; i < Ns; i++)
    {
        export_file1 << t(i) << "," << pos(i, 0) << "," << pos(i, 1) << "," << phi(i) << "," << vel(i, 0) << "," << vel(i, 1) << "," << dphi(i) << "," << path_vel(i) << std::endl;
    }
    // Close the file
    export_file1.close();
}

// wheel velocities are in rad/s
void Global_traj_class::rpm_convert()
{
  static double wheel_velocities [2];

    v_rpm.resize(Ns,2);

    
    for(unsigned int i = 0; i < Ns; i++)
    {
        double linear_robot_vel = path_vel(i);
        double angular_robot_vel = dphi(i);
        
        double left_wheel_vel = (linear_robot_vel / RobotConstants::WHEEL_RADIUS) - (RobotConstants::AXLE_LENGTH / 2 * angular_robot_vel / RobotConstants::WHEEL_RADIUS);
        wheel_velocities[0] = left_wheel_vel;

        double right_wheel_vel = (linear_robot_vel / RobotConstants::WHEEL_RADIUS) + (RobotConstants::AXLE_LENGTH / 2 * angular_robot_vel / RobotConstants::WHEEL_RADIUS);
        wheel_velocities[1] = right_wheel_vel;
        // left
        // v_rpm[0] = round((wheel_velocities[0] * 60/MathConstants::TWOPI) * 10) / 10;
        // value in *10
        v_rpm(i, 0) = round((wheel_velocities[0] * 60/MathConstants::TWOPI) * 10);
        // right
        v_rpm(i, 1) = round((wheel_velocities[1] * 60/MathConstants::TWOPI) * 10);
    }

    // Creating/ opening a csv file
    std::ofstream export_file2;
    export_file2.open("trajectory_with_rpm.csv",std::ios::out|std::ios::trunc);

    // Write data into the file
    export_file2 << "Time [s], X-Pos [m], Y-Pos [m], phi [rad], dphi [rad/s], Path Vel [m/s], v_l [rpm], v_r [rpm]" << std::endl;
    //	export_file << ref_traj << std::endl;

    for (unsigned int i = 0; i < Ns; i++)
    {
        export_file2 << t(i) << "," << pos(i, 0) << "," << pos(i, 1) << "," << phi(i) << "," << dphi(i) << "," << path_vel(i) << "," << v_rpm(i, 0) << "," << v_rpm(i, 1) <<std::endl;
    }
    // Close the file
    export_file2.close();
}

void Global_traj_class::generateTrajectory()
{
    // generate trajectory
    way_point_generator();
    if (way_points.rows() > 2)
    {
        //Multi-line case
        euler_spiral_coefficient_generator();
        euler_spiral_corner_smoothing_solver();
        point_to_point_motion_jlap_solver();
        trajectory_interpolator();
    }
    else
    {
        // Single line case
        point_to_point_single_line_trajectory_interpolator();
    }

    // export_global_trajectory();

    // TEMP calculate v_l v_r in rpm
    // rpm_convert();
}

// wheel velocities are in rad/s
void Global_traj_class::convertToFrontWheel(double x_dot, double y_dot, double phi_dot, double &v_front, double &delta)
{
    double v = sqrt(pow(x_dot, 2) + pow(y_dot, 2));

    double w = phi_dot;

    if (v != 0)
    {
        delta = atan2(w * RobotConstants::AXLE_LENGTH, v);
    }
    else
    {
        delta = 0;
    }

    v_front = v / cos(delta);
}

void Global_traj_class::publishPathAndTrajectory()
{
    // generate trajectory
    // generateTrajectory();

    trajectory_msg_.points.resize(Ns);
    adapted_path_msg_.poses.resize(Ns);
    adapted_path_msg_.header.frame_id = "/map";

    for (unsigned int i = 0; i < Ns; i++)
    {
        double v_front, delta;
        convertToFrontWheel(vel(i, 0), vel(i, 1), dphi(i), v_front, delta);

        // trajectory msg
        trajectory_msg_.points[i].x = pos(i, 0);
        trajectory_msg_.points[i].y = pos(i, 1);
        trajectory_msg_.points[i].heading = phi(i);
        trajectory_msg_.points[i].x_dot = v_front;
        trajectory_msg_.points[i].y_dot = 0;
        trajectory_msg_.points[i].heading_rate_radps = delta;
        trajectory_msg_.points[i].velocity_mps = path_vel(i);
        trajectory_msg_.points[i].acceleration_mps2 = path_acc(i);
        trajectory_msg_.points[i].heading_acc_radps2 = ddphi(i);

        // path msg
        adapted_path_msg_.poses[i].pose.position.x = pos(i,0);
        adapted_path_msg_.poses[i].pose.position.y = pos(i,1);
        tf::Quaternion q;
        q.setRPY(0, 0, phi(i));
        q.normalize();
        geometry_msgs::Quaternion quat;
        quat.w = q.w();
        quat.x = q.x();
        quat.y = q.y();
        quat.z = q.z();
        adapted_path_msg_.poses[i].pose.orientation = quat;
    }
    traj_pub_.publish(trajectory_msg_);
    path_pub_.publish(adapted_path_msg_);
}
