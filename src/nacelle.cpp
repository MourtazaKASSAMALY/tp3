#include <math.h>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

float u2 = 0; // commande

float f2(float X) // fonction d'évolution
{
	float Xdot = u2;
	return Xdot;
}

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg) // réception sur un topic: listener
{
	u2 = msg->angular.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nacellesimulator");  // node
	ros::NodeHandle n;  // node handler

	float initialx, initialy;
	n.param<float>("initialx", initialx, 0.); // pour un float: permet de recevoir des paramètres en entrée du programme, cf le .launch
	n.param<float>("initialy", initialy, 0.); // pour un float

	std::string boatframe, nacelleframe;
	n.param<std::string>("boatframe", boatframe, ros::this_node::getNamespace() + "boat"); // nom des repères
	n.param<std::string>("nacelleframe", nacelleframe, ros::this_node::getNamespace() + "nacelle");

	ros::Subscriber sub = n.subscribe("commandnacelle", 1000, chatterCallback); // s'abonne au topic commandnacelle (local au namespace)
	ros::Publisher vispub = n.advertise<visualization_msgs::Marker>("/nacelles", 0); // publie sur le topic /nacelles (global)
	ros::Rate loop_rate(25);

	tf2_ros::TransformBroadcaster br; // création d'un object permettant de publier des tf
	geometry_msgs::TransformStamped transformStamped; // message pour la transmission sur le canal
	transformStamped.header.frame_id = boatframe; // définition de la relation entre les repères
	transformStamped.child_frame_id = nacelleframe; // par exemple

	float X = 0; // vecteur d'état
	float dt = 0.1;

	while (ros::ok())
	{
		X = X + f2(X) * dt;

		tf::Quaternion q;
		q.setRPY(0, 0, X); // roll, pitch, yaw

		ros::Time stamp = ros::Time::now();
		transformStamped.header.stamp = stamp; // date de la transformée
		transformStamped.transform.translation.x = 4.5; // définition de la translation
		transformStamped.transform.translation.y = 0;
		tf::quaternionTFToMsg(q, transformStamped.transform.rotation); // définition de la rotation

		visualization_msgs::Marker marker; // message de type visualization_msgs/Marker
		marker.header.frame_id = nacelleframe; // nacelle affichée dans son propre repère
		marker.header.stamp = stamp;
		marker.ns = ros::this_node::getNamespace();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0; // pas de mouvements relatifs de la nacelle par rapport à elle même
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		marker.scale.x = 1;
		marker.scale.y = 1;
		marker.scale.z = 1;
		marker.color.a = 1.0; // alpha = transparence
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.mesh_resource = "package://tp3/meshs/turret.dae";
		
		vispub.publish(marker); // publie le message marker sur le topic /nacelles
		ros::spinOnce(); // listener sur le topic commandnacelle
		br.sendTransform(transformStamped); // envoi du tf
		loop_rate.sleep(); // pause
	}

	return 0;
}