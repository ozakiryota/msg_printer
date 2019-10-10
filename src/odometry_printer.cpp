#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class OdometryPrinter{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscriber*/
		ros::Subscriber sub_odom;
	public:
		OdometryPrinter();
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
};

OdometryPrinter::OdometryPrinter()
{
	sub_odom = nh.subscribe("/odom", 1, &OdometryPrinter::CallbackOdom, this);
}

void OdometryPrinter::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	/*compute RPY*/
	tf::Quaternion q_orientation;
	quaternionMsgToTF(msg->pose.pose.orientation, q_orientation);
	std::vector<double> rpy(3);
	tf::Matrix3x3(q_orientation).getRPY(rpy[0], rpy[1], rpy[2]);
	for(int i=0;i<3;i++)	rpy[i] = rpy[i]/M_PI*180.0;
	/*print*/
	std::cout << "----- " << msg->child_frame_id << " -----" << std::endl;
	std::cout 
		<< "(x, y, z) = "
		<< msg->pose.pose.position.x << ", " 
		<< msg->pose.pose.position.y << ", " 
		<< msg->pose.pose.position.z << std::endl;
	double d = sqrt(msg->pose.pose.position.x*msg->pose.pose.position.x + msg->pose.pose.position.y*msg->pose.pose.position.y + msg->pose.pose.position.z*msg->pose.pose.position.z);
	std::cout << "Euclidian distance = " << d << std::endl;
	std::cout 
		<< "(r, p, y) = "
		<< rpy[0] << ", " 
		<< rpy[1] << ", " 
		<< rpy[2] << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_printer");

	OdometryPrinter odometry_printer;

	ros::spin();
}
