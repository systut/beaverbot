#ifndef UTILITIES_H
#define UTILITIES_H

#include <boost/math/constants/constants.hpp>

#include <eigen3/Eigen/Core>

#include <cmath>
#include <iostream>

namespace MathConstants
{
    constexpr double PI = boost::math::constants::pi<double>();
    constexpr double TWOPI = 2 * boost::math::constants::pi<double>();
    constexpr float PIf = boost::math::constants::pi<float>();
    constexpr float TWOPIf = 2 * boost::math::constants::pi<float>();
    constexpr double G = 9.81;  // [m/s^2]
    constexpr double EPS = 1e-10;
    constexpr float EPSf = 1e-10f;
}

namespace RobotConstants
{
    // miniAGV
    // constexpr double AXLE_LENGTH                 = 0.33;             // [m]
    // constexpr double ROBOT_WIDTH                 = 0.38;             // [m] 
    // constexpr double ROBOT_LENGTH                = 0.63;             // [m]
    // constexpr double WHEEL_RADIUS                = 0.055;           // [m]


    // NewMiniAGV
    constexpr double AXLE_LENGTH                 = 0.3211;             // [m]
    constexpr double ROBOT_WIDTH                 = 0.510;             // [m] 
    constexpr double ROBOT_LENGTH                = 0.7468;             // [m]
    constexpr double WHEEL_RADIUS                = 0.075;           // [m]


    constexpr double MAX_WHEEL_ACC               = 0.08;          // [m/s^2]
    constexpr double MAX_WHEEL_VEL               = 0.2;       // [m/s]
    // constexpr double ROBOT_MASS                  = 313.4;              // [kg]
    // constexpr double FRONT_TO_REAR_WHEEL         = 0.8;              // [m]
    // constexpr double WHEEL_CONTACT_LENGTH        = 0.05;             // [m] approx contact length of one wheel
    // constexpr double FRICTION_COEFFICIENT        = 0.7;              // [] approx friction coeff of rubber and dry concrete
    // constexpr double DRIVING_GEAR_RATIO          = 40.0;             // [ ] Driving motor to wheel gear ratio
    // constexpr double DRIVING_RATED_MOTOR_TORQUE  = 1.3;              // [Nm]
    // constexpr double DRIVING_RATED_MOTOR_SPEED   = 3000;             // [RPM]
    // constexpr double DRIVING_MOTOR_INERTIA       = 0.66e-4;          // [kgm^2]
    // constexpr double DRIVING_RATED_TORQUE        = DRIVING_GEAR_RATIO * DRIVING_RATED_MOTOR_TORQUE;                  // [Nm]    Torque reflected to the robot's wheels
    // constexpr double DRIVING_INERTIA             = DRIVING_GEAR_RATIO * DRIVING_GEAR_RATIO * DRIVING_MOTOR_INERTIA;  // [kgm^2] INERTIA reflected to the robot's wheels
    // constexpr double STEERING_RATED_MOTOR_SPEED  = 3000;             // [RPM]
    // constexpr double STEERING_GEAR_RATIO         = 96.428;           // []
    // constexpr double STEERING_RATED_MOTOR_TORQUE = 0.637;            // [Nm]
    // constexpr double STEERING_MOTOR_INERTIA      = 0.219;            // [kgm^2]
    // //constexpr double MAX_WHEEL_ACC               = (DRIVING_RATED_TORQUE / DRIVING_INERTIA) * WHEEL_RADIUS;          // [m/s^2]

    // constexpr double MAX_STEERING_VEL            = 0.9; //[rad/s]//(MathConstants::TWOPI / 60) * STEERING_RATED_MOTOR_SPEED / STEERING_GEAR_RATIO;    // [rad/s]
    // constexpr double MAX_STEERING_ACC            = (12 * STEERING_GEAR_RATIO * STEERING_RATED_MOTOR_TORQUE - FRICTION_COEFFICIENT * ROBOT_MASS * MathConstants::G * WHEEL_CONTACT_LENGTH) / (12 * STEERING_GEAR_RATIO * STEERING_GEAR_RATIO * STEERING_MOTOR_INERTIA);  // [rad/s^2]
    // constexpr double MAX_DRIVING_RATED_MOTOR_SPEED_PER_V = 400;      // [RPM / V]
    // constexpr double MAX_STEERING_VOLTAGE        = 5;              // [V]
    // constexpr double MAX_DRIVING_VOLTAGE         = 10.0;             // [V]
    // constexpr double DRIVING_GAIN                = (MathConstants::PI / 3) * WHEEL_RADIUS;      // [m/s/V]
    // constexpr double STEERING_GAIN               = MathConstants::PI / 10; // [rad/s/V] //10 * MathConstants::PI / STEERING_GEAR_RATIO;
    // constexpr double COULOMB_FRICTION            = 0.0;   // [m/s]
    // constexpr double wheel_radius = 0.328 / 2;
    // constexpr double wheel_to_center = 0.53 / 2;

    // in m/s^2 for the wheels not the robot
    const double max_translational_acc = 10.5;
    // for the wheels
    const double max_defined_lin_acc = 4.0;	// m/s^2
    // const double max_defined_ang_acc = max_defined_lin_acc / wheel_radius;	//rad/s^2
}

// namespace EncoderConstants
// {
//     constexpr unsigned int DRIVING_ENCODER_SAMPLE = 8000;
//     constexpr unsigned int STEERING_ENCODER_SAMPLE = 65535;
//     constexpr double STEERING_MAXIMUM_PULSE = STEERING_ENCODER_SAMPLE * RobotConstants::STEERING_GEAR_RATIO;
// }

// namespace Testing
// {
//     // temporary booleans for testing controllers or behaviours
//     constexpr bool USE_MPC = false;
//     constexpr bool USE_OPEN_LOOP = false;
//     constexpr bool USE_VFH_PP = true;

//     // turns of emergency stop and only runs avoidance algo (VFH/PP)
//     constexpr bool TEST_AVOIDANCE = true;
//     // ignores lidar readings
//     constexpr bool TRACKING_ONLY = true;
// }

namespace ControlConstants
{
    constexpr unsigned int HORIZON = 40;    // horizon for MPC
    constexpr double PURE_PURSUIT_LOOKAHEAD = 0.4;
}

namespace TrajectoryParameters
{
    constexpr double PATH_VEL_LIM  = 0.3;
    constexpr double PATH_VEL_MIN = 0.1;
}

namespace GeneralFunctions
{
    inline bool isEqual(const double lhs, const double rhs)
    {
        return std::abs(lhs - rhs) < MathConstants::EPS;
    }

    inline bool isEqualf(const float lhs, const float rhs)
    {
        return std::abs(lhs - rhs) < MathConstants::EPSf;
    }

    inline double wrapTo2Pi(const double rad)
    {
        double value = fmod(rad, MathConstants::TWOPI);
        if (value < 0)
        {
            value += MathConstants::TWOPI;
        }
        return value;
    }

    inline float wrapTo2Pif(const float rad)
    {
        float value = fmodf(rad, MathConstants::TWOPIf);
        if (value < 0)
        {
            value += MathConstants::TWOPIf;
        }
        return value;
    }

    inline double wrapToPi(double rad)
    {
        double value = rad - MathConstants::TWOPI * floor((rad + MathConstants::PI) / MathConstants::TWOPI);
        return value;
    }

    inline float wrapToPif(float rad)
    {
        float value = rad - MathConstants::TWOPIf * floorf((rad + MathConstants::PIf) / MathConstants::TWOPIf);
        return value;
    }

    inline double absoluteDifferenceBetweenAngles(const double rad1, const double rad2)
    {
        double value = MathConstants::PI - fabs(fmod(fabs(rad1 - rad2), MathConstants::TWOPI) - MathConstants::PI);
        return value;
    }

    inline double angularVelocity(const double rad1, const double rad2, const double sampling_time)
    {
        double signed_diff;
        double mod_diff = fmod(fabs(rad1 - rad2), MathConstants::TWOPI);
        if (mod_diff > MathConstants::PI)
        {
            signed_diff = MathConstants::TWOPI - mod_diff;
            if (rad2 > rad1)
            {
                signed_diff *= -1;
            }
        }
        else
        {
            signed_diff = mod_diff;
            if (rad1 > rad2)
            {
                signed_diff *= -1;
            }
        }
        return signed_diff / sampling_time;
    }

    inline double relativeAngleRobotPoint(const Eigen::Vector3d &robot_pose, const Eigen::Vector2d &pos)
    {
        Eigen::Vector2d vec1(std::cos(robot_pose.z()), std::sin(robot_pose.z()));
        Eigen::Vector2d vec2 = pos - robot_pose.head<2>();

        // "https://www.euclideanspace.com/maths/algebra/vectors/angleBetween/issues/index.htm"
        double angle = std::atan2(vec2(1), vec2(0)) - std::atan2(vec1(1), vec1(0));
        if (angle > M_PI){angle -= 2 * M_PI;}
        else if (angle < -M_PI) {angle += 2 * M_PI;}
        return angle;
    }

    template<typename T> static int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

}

// namespace RobotLimits
// {
//     /// check if the control commands hit the input constraints
//     /// returns angle value between -Pi and Pi
//     inline double steeringLimit(const double prev_angle, const double angle_cmd, const double sampling_time)
//     {
//         double ret = angle_cmd;;
//         // steering velocity limit
//         double angle_diff = GeneralFunctions::absoluteDifferenceBetweenAngles(prev_angle, angle_cmd);
//         double vel = (angle_diff) / sampling_time;
//         if (std::abs(vel) > RobotConstants::MAX_STEERING_VEL)
//         {
//             if (GeneralFunctions::isEqual(GeneralFunctions::wrapToPi(prev_angle - angle_diff), angle_cmd)) 
//             {ret = prev_angle - RobotConstants::MAX_STEERING_VEL * sampling_time;}
//             else 
//             {ret = prev_angle + RobotConstants::MAX_STEERING_VEL * sampling_time;}
//         }
//         return GeneralFunctions::wrapToPi(ret);
//     }

//     /// checks if the steering velocity is satisfied
//     /// velocities are in [rad/s]
//     inline double steeringVelLimit(const double steering_vel)
//     {
//         double ret = steering_vel;
//         if (std::abs(steering_vel) > RobotConstants::MAX_STEERING_VEL)
//         {
//             if (steering_vel < 0)
//             {ret = -RobotConstants::MAX_STEERING_VEL;}
//             else
//             {ret = RobotConstants::MAX_STEERING_VEL;}
//         }
//         return ret;
//     }

//     /// check if the control commands hit the input constraints
//     /// velocities are in [m/s]
//     inline double drivingLimit(const double prev_vel, const double vel_cmd, const double sampling_time)
//     {
//         double ret = vel_cmd;
//         // wheel acceleration limit
//         double acc = (vel_cmd - prev_vel) / sampling_time;
//         if (std::abs(acc) > RobotConstants::MAX_WHEEL_ACC)
//         {
//             if (acc < 0) 
//             {ret = prev_vel - RobotConstants::MAX_WHEEL_ACC * sampling_time;}
//             else 
//             {ret = prev_vel + RobotConstants::MAX_WHEEL_ACC * sampling_time;}
//         }
//         // wheel velocity limit
//         if (std::abs(ret) > RobotConstants::MAX_WHEEL_VEL)
//         {
//             if (ret < 0) 
//             {ret = -RobotConstants::MAX_WHEEL_VEL;}
//             else 
//             {ret = RobotConstants::MAX_WHEEL_VEL;}
//         }
//         return ret;
//     }

// }

#endif