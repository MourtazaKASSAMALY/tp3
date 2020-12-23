#include "ros/ros.h"
#include "tf/tf.h"
#include <geometry_msgs/PoseStamped.h>
#include "visualization_msgs/Marker.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "targetpublisher");  // node
	ros::NodeHandle n;  // node handler

	ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("targetpos", 0); // publie sur le topic targetpos (local au namespace)
	ros::Publisher vispub = n.advertise<visualization_msgs::Marker>("/targets", 0); // publie sur le topic /targets (global)
	ros::Rate loop_rate(25);

	geometry_msgs::PoseStamped msg;

	float targetx = 30;
	float targety = 30;

	while (ros::ok())
	{
		msg.header.stamp = ros::Time::now();
		msg.pose.position.x = targetx;
		msg.pose.position.y = targety;
		
		visualization_msgs::Marker marker; // message de type visualization_msgs/Marker
		marker.header.frame_id = "world"; // cible affichée dans l'absolu
		marker.header.stamp = ros::Time::now();
		marker.ns = ros::this_node::getNamespace();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 1;
		marker.scale.y = 1;
		marker.scale.z = 1;
		marker.pose.position.x = targetx;
		marker.pose.position.y = targety;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		marker.color.a = 1.0; // alpha = transparence
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.mesh_resource = "package://tp3/meshs/iphone.dae"; // la cible est un iPhone

		vispub.publish(marker); // publie le message marker sur le topic /targets
		pub.publish(msg); // publie le message msg sur le topic targetpos
		loop_rate.sleep(); // pause
	}

	return 0;
}