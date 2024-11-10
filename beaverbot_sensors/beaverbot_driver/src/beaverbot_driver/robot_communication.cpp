/*
 * ===============================================================================
 * robot_communication.cpp
 * Author: Schaefle Tobias
 * -------------------------------------------------------------------------------
 * Description:
 * this node starts a commuication with a serial interface with the robot
 * the robot sends informations to ROS (battery, encoder, etc.), this program allows
 * the user to send velocities to the left and right wheel of the robot
 *
 * Subscribes to:
 * - /industrial_robot/wheel_velocities
 * - or to /cmd_vel
 *
 * Publishes:
 * - Data type: sensor_msgs/BatteryState
 *		beaverbot_driver/EncoderWheelVel
 * - Address: encoder
 *	      battery
 * ===============================================================================
 */

#include <ros/ros.h>
#include <SerialStream.h>
#include <beaverbot_driver/RobotWheelVel.h>
#include <beaverbot_driver/EncoderWheelVel.h>
#include <sensor_msgs/BatteryState.h>
#include "beaverbot_driver/robot_data.h"
#include "geometry_msgs/Twist.h"

using namespace LibSerial;
using namespace industrial_robot;

class RobotCom
{
private:
    int send_left_wheel_, send_right_wheel_;
    int enc_left_wheel_, enc_right_wheel_, voltage_, current_, battery_capacity_;

    char send_left_0_;
    char send_left_1_;
    char send_right_0_;
    char send_right_1_;
    unsigned char stop_condition_;
    unsigned char operation_selection_;
    unsigned char operating_device_setting_;
    unsigned char checksum_;

    SerialStream robot_port_;
    std::string port_;

    ros::NodeHandle nh_;

    ros::Publisher encoder_;
    ros::Publisher battery_;

    // transform bytes to integers and save them
    void convertData(char readData[])
    {
        char left_wheel_0 = readData[7];
        char left_wheel_1 = readData[6];
        char right_wheel_0 = readData[5];
        char right_wheel_1 = readData[4];

        char voltage_0 = readData[13];
        char voltage_1 = readData[12];
        char current_0 = readData[15];
        char current_1 = readData[14];

	enc_left_wheel_ = byteToInteger(left_wheel_0, left_wheel_1);
	enc_right_wheel_ = byteToInteger(right_wheel_0, right_wheel_1);

	voltage_ = byteToInteger(voltage_0, voltage_1);
	current_ = byteToInteger(current_0, current_1);

	battery_capacity_ = static_cast<unsigned char>(readData[10]);
    }

    int byteToInteger(char byte_0, char byte_1)
    {
        int first, second;

        first = static_cast<unsigned char>(byte_0);
        second = static_cast<unsigned char>(byte_1)<<8;   //shift to the left by one byte

        int total = first + second;
        if (total > 32767)  // max pos number (0111 1111 1111 1111)
        {
            total = total - 65536; // equals to 1 0000 0000 0000 0000
        }
        return total;
    }

    // this function calculates the checksum of the incoming data and returns a boolean
    bool getChecksum(char readData[])
    {
        char checksum;
        bool check = true;

	checksum = readData[0] + readData[1] + readData[2] + readData[3] + readData[4] + readData[5] + readData[6] +
                readData[7] + readData[8] + readData[9] + readData[10] + readData[11] + readData[12] + readData[13] +
	        readData[14] + readData[15] + readData[16] + readData[17];

	//ROS_INFO("Checksum is: %d", checksum);

        if (checksum != 0) {check = false;}

        return check;
    }

    int intToByteWheelValue(int wheel_vel)
    {
        int int_to_byte_value;

        if (wheel_vel < 0)
        {
	    if (wheel_vel < -RobotParams::max_wheel_vel_robot)  //max vel
            {
		wheel_vel = -RobotParams::max_wheel_vel_robot;
            }
	    int_to_byte_value = 65536 + wheel_vel;  // that way its a signed byte
        }
        else
        {
	    if (wheel_vel > RobotParams::max_wheel_vel_robot)   //max vel
            {
		wheel_vel = RobotParams::max_wheel_vel_robot;
            }
            int_to_byte_value = wheel_vel;
        }

        return int_to_byte_value;
    }

    char * getWheelVelBytes(int wheel_vel_left, int wheel_vel_right)
    {
        int int_to_byte_value_left;
        int int_to_byte_value_right;
        char left[2];
        char right[2];
        static char bytes[4];

        int_to_byte_value_left = intToByteWheelValue(wheel_vel_left);
        int_to_byte_value_right = intToByteWheelValue(wheel_vel_right);

	// this changes the integer to bytes[2] (in C++ char[2])
        std::copy(static_cast<const char*>(static_cast<const void*>(&int_to_byte_value_left)),
                  static_cast<const char*>(static_cast<const void*>(&int_to_byte_value_left)) + 2,   // the 2 is the number of bytes
                  left);

        std::copy(static_cast<const char*>(static_cast<const void*>(&int_to_byte_value_right)),
                  static_cast<const char*>(static_cast<const void*>(&int_to_byte_value_right)) + 2,   // the 2 is the number of bytes
                  right);

        bytes[0] = left[0];
        bytes[1] = left[1];
        bytes[2] = right[0];
        bytes[3] = right[1];

        return bytes;
    }

public:
    RobotCom(ros::NodeHandle nh, std::string port="")
    {
	// set the port address
	this->port_ = port;

	// fixed commands to send to the robot
	this->operation_selection_ = 160;
	this->stop_condition_ = 0;
	this->operating_device_setting_ = 128;

        // at setup set speed to zero to prevent an error if there will be no input
	this->send_left_wheel_ = 0;
	this->send_right_wheel_ = 0;

	this->nh_ = nh;
	this->encoder_ = nh.advertise<beaverbot_driver::EncoderWheelVel>("encoder", 1000);
	this->battery_ = nh.advertise<sensor_msgs::BatteryState>("battery", 1000);
    }

    bool openRobot()
    {
        try
        {
	    // robot port settings
	    robot_port_.Open(port_);
	    robot_port_.SetBaudRate( SerialStreamBuf::BAUD_38400 );
	    robot_port_.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
	    robot_port_.SetNumOfStopBits(1);
	    robot_port_.SetParity(SerialStreamBuf::PARITY_EVEN);
	    robot_port_.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_DEFAULT);

        }
        catch (int e)
        {
            ROS_INFO("An exception occurred. Exception Nr. %d", e);
        }
	if (robot_port_.IsOpen())
        {
            ROS_INFO_STREAM("Serial Port Initialized");
        }
        else
        {
            ROS_ERROR_STREAM("Unable to open port");
            return false;
        }
        return true;
    }

    void readRobot()
    {
	// when data is available
	if(robot_port_.rdbuf()->in_avail() > 0)
        {
	    //ROS_INFO_STREAM("Reading from serial port");

            char readData[18];
	    robot_port_.read(readData, 18);

	    // check if the checksum is correct, if not then do not use data
            if (getChecksum(readData)) {convertData(readData);}
	    // if checksum wrong throw one byte away
	    else {char throw_byte; robot_port_.get(throw_byte);}

	    beaverbot_driver::EncoderWheelVel encoder_msg;
	    encoder_msg.enc_left_wheel = enc_left_wheel_;
	    encoder_msg.enc_right_wheel = enc_right_wheel_;

	    //ROS_INFO("Encoder %d %d", enc_left_wheel, enc_right_wheel);

	    // voltage and current have to be divided by a certain value
	    sensor_msgs::BatteryState battery_msg;
	    battery_msg.voltage = voltage_ / 100;
	    battery_msg.current = current_ / 100;
	    battery_msg.capacity = battery_capacity_;

	    encoder_.publish(encoder_msg);
	    battery_.publish(battery_msg);
        }
    }

    void writeRobot()
    {
	char * velocities = getWheelVelBytes(send_left_wheel_, send_right_wheel_);

	send_left_0_ = velocities[0];
	send_left_1_ = velocities[1];
	send_right_0_ = velocities[2];
	send_right_1_ = velocities[3];

	// calculation of the checksum
	unsigned char sum = stop_condition_ + operation_selection_ +
	        static_cast<unsigned char>(send_right_1_) + static_cast<unsigned char>(send_right_0_) +
	        static_cast<unsigned char>(send_left_1_) + static_cast<unsigned char>(send_left_0_) +
	        operating_device_setting_;

	checksum_ = static_cast<unsigned char>(256) - sum;

	char sendSpeed[] = {static_cast<char>(stop_condition_), static_cast<char>(operation_selection_),
	                   static_cast<char>(send_right_1_), static_cast<char>(send_right_0_),
	                   static_cast<char>(send_left_1_), static_cast<char>(send_left_0_),
	                   static_cast<char>(operating_device_setting_), static_cast<char>(checksum_)};

	//ROS_INFO("test: %d %d %d %d %d %d %d %d", sendSpeed[0], sendSpeed[1], sendSpeed[2], sendSpeed[3], sendSpeed[4], sendSpeed[5], sendSpeed[6], sendSpeed[7]);

	robot_port_.write(sendSpeed, 8);

    }

    void closeRobot()
    {
	// message to stop the robot before closing the port
	char send_bytes_zero[] = { 0x00, static_cast<char>(0xA0), 0x00, 0x00, 0x00, 0x00, static_cast<char>(0x80), static_cast<char>(0xE0)};

	robot_port_.write(send_bytes_zero, 8);
	robot_port_.Close();
    }

    void getWheelVel(const beaverbot_driver::RobotWheelVel::ConstPtr &vel)
    {
	send_left_wheel_ = vel->left_wheel;
	send_right_wheel_ = vel->right_wheel;
    }

    void getRobotVel(const geometry_msgs::Twist::ConstPtr &robot_vel_msg)
    {
	double * wheel_vel;

	wheel_vel = RobotModel::robot_velocites_to_wheel_velocities(robot_vel_msg->linear.x, robot_vel_msg->angular.z);
	send_left_wheel_ = RobotSupportFunctions::rad_to_robot_value(wheel_vel[0]);
	send_right_wheel_ = RobotSupportFunctions::rad_to_robot_value(wheel_vel[1]);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_communication");

    std::string port = std::string("/dev/ttyUSB0");
    ros::param::get("~port", port);

    ros::NodeHandle nh;
    RobotCom robot(nh, port);
    //ros::Subscriber subWheelSpeed = nh.subscribe("/industrial_robot/wheel_velocities", 1000, &RobotCom::getWheelVel, &robot);
    ros::Subscriber subRobotSpeed = nh.subscribe("/cmd_vel", 1000, &RobotCom::getRobotVel, &robot);

    // try to open port
    if (robot.openRobot() == false) {return -1;}

    // loop rate of 20Hz
    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        ros::spinOnce();

        robot.writeRobot();
	robot.readRobot();

        loop_rate.sleep();
    }
    robot.closeRobot();

    return 0;
}
