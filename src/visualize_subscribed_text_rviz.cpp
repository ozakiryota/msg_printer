#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

class VisualizeSubscribedTextRviz{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_string;
		/*publisher*/
		ros::Publisher _pub_vismarker;
		/*visualization marker*/
		visualization_msgs::Marker _text;
		/*parameter*/
		std::string _frame_id;
		double _position_x, _position_y, _position_z;
		double _scale;
		double _color_r, _color_b, _color_g, _color_a;
		double _lifetime_sec;

	public:
		VisualizeSubscribedTextRviz();
		void initializeVisMarker(void);
		void callbackString(const std_msgs::StringConstPtr& msg);
		void inputVisMarker(std_msgs::String string_msg);
		void publication(void);
};

VisualizeSubscribedTextRviz::VisualizeSubscribedTextRviz()
	: _nhPrivate("~")
{
	std::cout << "--- visualize_subscribed_text_rviz ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("frame_id", _frame_id, std::string("/frame"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	_nhPrivate.param("position_x", _position_x, 0.0);
	std::cout << "_position_x = " << _position_x << std::endl;
	_nhPrivate.param("position_y", _position_y, 0.0);
	std::cout << "_position_y = " << _position_y << std::endl;
	_nhPrivate.param("position_z", _position_z, 0.0);
	std::cout << "_position_z = " << _position_z << std::endl;
	_nhPrivate.param("scale", _scale, 1.0);
	std::cout << "_scale = " << _scale << std::endl;
	_nhPrivate.param("color_r", _color_r, 0.0);
	std::cout << "_color_r = " << _color_r << std::endl;
	_nhPrivate.param("color_g", _color_g, 0.0);
	std::cout << "_color_g = " << _color_g << std::endl;
	_nhPrivate.param("color_b", _color_b, 0.0);
	std::cout << "_color_b = " << _color_b << std::endl;
	_nhPrivate.param("color_a", _color_a, 1.0);
	std::cout << "_color_a = " << _color_a << std::endl;
	_nhPrivate.param("lifetime_sec", _lifetime_sec, 0.1);
	std::cout << "_lifetime_sec = " << _lifetime_sec << std::endl;
	/*subscriber*/
	_sub_string = _nh.subscribe("/vis_text_rviz", 1, &VisualizeSubscribedTextRviz::callbackString, this);
	/*subscriber*/
	_pub_vismarker = _nh.advertise<visualization_msgs::Marker>("/vis/text", 1);
	/*initialize*/
	initializeVisMarker();
}

void VisualizeSubscribedTextRviz::initializeVisMarker(void)
{
	_text.header.frame_id = _frame_id;
	_text.ns = "text";
	_text.id = 0;
	_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	_text.action = visualization_msgs::Marker::ADD;
	_text.pose.position.x = _position_x;
	_text.pose.position.y = _position_y;
	_text.pose.position.z = _position_z;
	_text.scale.z = _scale;
	_text.color.r = _color_r;
	_text.color.g = _color_g;
	_text.color.b = _color_b;
	_text.color.a = _color_a;
	_text.lifetime = ros::Duration(_lifetime_sec);
}

void VisualizeSubscribedTextRviz::callbackString(const std_msgs::StringConstPtr& msg)
{
	/*input*/
	inputVisMarker(*msg);
	/*publication*/
	publication();
}

void VisualizeSubscribedTextRviz::inputVisMarker(std_msgs::String string_msg)
{
	/*header*/
	_text.header.stamp = ros::Time::now();
	/*text*/
	_text.text = string_msg.data;
}

void VisualizeSubscribedTextRviz::publication(void)
{
	/*reset*/
	visualization_msgs::Marker delete_markers;
	delete_markers.action = visualization_msgs::Marker::DELETEALL;
	_pub_vismarker.publish(delete_markers);
	/*_text*/
	_pub_vismarker.publish(_text);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualize_subscribed_text_rviz");
	
	VisualizeSubscribedTextRviz visualize_subscribed_text_rviz;

	ros::spin();
}
