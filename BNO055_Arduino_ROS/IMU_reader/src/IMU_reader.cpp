
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include "SerialPort.h"
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include "tf/tf.h"

const double PI = 3.14159;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "IMU_reader");
	ros::NodeHandle n;
	ros::Rate loop_rate(80);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	uint32_t shape = visualization_msgs::Marker::CUBE;

	try
	{
		//Open serial port
		ROS_INFO("Opening serial connection.");
		SerialPort serial_port ("/dev/ttyACM0");
		serial_port.Open(SerialPort::BAUD_9600,
						 SerialPort::CHAR_SIZE_8, 
						 SerialPort::PARITY_NONE, 
						 SerialPort::STOP_BITS_1, 
						 SerialPort::FLOW_CONTROL_NONE);
		
		// Enter the main loop
		while (ros::ok())
		{
			// Wait until the data is available
			if(serial_port.IsDataAvailable())
		    {
				// Vector of strings
				std::vector<std::string> strs;

				// Read the serial port and pass the result to the splitting function
				boost::split(strs, serial_port.ReadLine(), boost::is_any_of(","));
				
				// 3 strings are expected, any other case is an error
				if(strs.size() == 3)
				{
					// Print the result to the console
					ROS_INFO("%s %s %s", strs[0].c_str(), strs[1].c_str(), strs[2].c_str());
				
					// Create a marker to visualize the IMU data
					visualization_msgs::Marker marker;
					marker.header.frame_id = "/my_frame";
					marker.header.stamp = ros::Time::now();
					marker.ns = "basic_shapes";
					marker.id = 0;
					marker.type = shape;
					marker.action = visualization_msgs::Marker::ADD;
				
					// Convert the Roll Pitch Yaw to quaternion. BNO055 can actually output quaternions, need to look into that.
					float roll = PI*atof(strs[1].c_str())/180.0;
					float pitch = -PI*atof(strs[2].c_str())/180.0;
					float yaw = -PI*atof(strs[0].c_str())/180.0;
					//ROS_INFO("%f %f %f", roll, pitch, yaw);

					tf::Quaternion quat(roll, pitch, yaw);
					quat.setRPY(roll, pitch, yaw);
				
					// Position of the marker
					marker.pose.position.x = 0;
					marker.pose.position.y = 0;
					marker.pose.position.z = 0;
		
					// Pose of the marker
					marker.pose.orientation.x = quat.x();
					marker.pose.orientation.y = quat.y();
					marker.pose.orientation.z = quat.z();
					marker.pose.orientation.w = quat.w();
				
					// Scale of the marker
					marker.scale.x = 3.0;
					marker.scale.y = 3.0;
					marker.scale.z = 0.5;

					// Set the color -- be sure to set alpha to something non-zero!
					marker.color.r = 0.0f;
					marker.color.g = 1.0f;
					marker.color.b = 0.0f;
					marker.color.a = 1.0;

					marker.lifetime = ros::Duration();

					while (marker_pub.getNumSubscribers() < 1)
					{
						if (!ros::ok())
						{
							return 0;
						}
						ROS_WARN_ONCE("Please create a subscriber to the marker");
						sleep(1);
					}
				
					marker_pub.publish(marker);
				}
			}

			ros::spinOnce();

			loop_rate.sleep();
		}
		
		// Close connection
		if(serial_port.IsOpen())
		{
			ROS_INFO("Closing serial connection.");
			serial_port.Close();
		}

		ROS_INFO("Serial node closed.");
		return 0;
	}
	catch (SerialPort::OpenFailed exception)
	{
		ROS_ERROR("Failed to open the port");
	}

	catch (SerialPort::AlreadyOpen exception)
	{
		ROS_ERROR("Port is already open");
	}
}

