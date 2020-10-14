#include "ros/ros.h"
#include <ros/time.h>
#include <sensor_msgs/Imu.h>// imu msg
#include <iostream>

using namespace std;

class LowpassFilter
{
	public:
		LowpassFilter();
		void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
		void run();

	private:
		ros::NodeHandle node_;
		ros::Subscriber imu_subscriber;
		ros::Publisher imu_publisher;

		string input_topic = "input";
		string output_topic = "output";

		double cutoff = 1;

		ros::Time time_sampling = ros::Time::now();
		bool initial_value = true;

		double T_ = ros::Time::now().toSec();
		double imu_temp[3] = {0, 0, 0};
		double X[3][2] = {{0,0}};
		double Y[3][2] = {{0}};
};

LowpassFilter::LowpassFilter()
{
	ros::NodeHandle nh("~");

	//Setting class parameters
	if(!nh.getParam("input_topic", input_topic))
		input_topic = "input";
	if(!nh.getParam("output_topic", output_topic))
		output_topic = "output";
	if(!nh.getParam("cutoff", cutoff))
		cutoff = 1;

	imu_subscriber = node_.subscribe<sensor_msgs::Imu> (input_topic.c_str(), 1, boost::bind(&LowpassFilter::imuCallback,this, _1));
	imu_publisher = node_.advertise<sensor_msgs::Imu> (output_topic.c_str(), 1, false);
}

void LowpassFilter::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
	sensor_msgs::ImuPtr output(new sensor_msgs::Imu());
	*output = *imu;

	imu_temp[0] = imu->angular_velocity.x;
	imu_temp[1] = imu->angular_velocity.y;
	imu_temp[2] = imu->angular_velocity.z;

	if(initial_value)
	{
		initial_value = false;
		for(int i=0; i<3; i++)
		{
			X[i][0] = imu_temp[i];
			X[i][1] = imu_temp[i];
			Y[i][0] = X[i][0];
			Y[i][1] = X[i][0];
		}
	}
	else
	{
		T_ = (ros::Time::now() - time_sampling).toSec();
		for(int i=0; i<3; i++)
		{
			X[i][1] = X[i][0];
			X[i][0] = imu_temp[i];
			Y[i][1] = Y[i][0];
			Y[i][0] = 2*M_PI*cutoff*T_*X[i][1] + (1 - 2*M_PI*cutoff*T_)*Y[i][1];
		}
	}
	time_sampling = ros::Time::now();
	output->angular_velocity.x = Y[0][0];
	output->angular_velocity.y = Y[1][0];
	output->angular_velocity.z = Y[2][0];
	imu_publisher.publish(output);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lowpass_filter");

	LowpassFilter _Lowpass_Filter;

	ros::spin();

	return 0;
}