#include "tf01/tf01.h"
#include "lidar_driver/Lidar.h"
#include <sensor_msgs/Range.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tf01_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");
	ros::Publisher lidar_pub = n.advertise<sensor_msgs::Range>("/mavros/lidar/range", 10);
	TF01 tf01(n_private);
	ros::Rate loop_rate(500);

	bool first_measure = false;
	//lidar_driver::Lidar msg;
	sensor_msgs::Range msg;	
	int cnt = 0;
	while(ros::ok())
	{
		cnt++;
		if(cnt>10)
		{
			cnt = 0;
		}

		tf01.read_data();

		if(tf01.data_valid)
		{
			if(!first_measure)
			{
				ROS_INFO("TF01 Lidar started!");
				first_measure = true;
			}

			//msg.header.stamp = ros::Time::now();
			//msg.distance.data = tf01.distance / 100.0;
			//msg.amplitude.data = tf01.amplitude;
			msg.header.stamp = ros::Time::now();
			msg.radiation_type = sensor_msgs::Range::INFRARED;
			msg.min_range = 0.1;
			msg.max_range = 10.0;
			msg.range = tf01.distance / 100.0;
			if(cnt == 0)
			{
				lidar_pub.publish(msg);
			}
			tf01.data_valid = false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

