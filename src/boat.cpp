#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

float u1 = 0; // commande

Eigen::Vector3f f1(Eigen::Vector3f X) // fonction d'évolution
{
	Eigen::Vector3f Xdot(1*cos(X(2)), 1*sin(X(2)), u1);
	return Xdot;
}

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg) // réception sur un topic: listener
{
	u1 = msg->angular.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "boatsimulator");  // node
	ros::NodeHandle n;  // node handler

	float initialx, initialy;
	n.param<float>("initialx", initialx, 0.); // pour un float: permet de recevoir des paramètres en entrée du programme, cf le .launch
	n.param<float>("initialy", initialy, 0.);

	std::string boatframe;
	n.param<std::string>("boatframe", boatframe, ros::this_node::getNamespace() + "boat"); // nom des repères

	ros::Subscriber sub = n.subscribe("commandboat", 1000, chatterCallback); // s'abonne au topic commandboat (local au namespace)
	ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("boatstate", 0); // publie sur le topic boatstate (local au namespace)
	ros::Publisher vispub = n.advertise<visualization_msgs::Marker>("/boats", 0); // publie sur le topic /boats (global)
	ros::Rate loop_rate(25);

	tf2_ros::TransformBroadcaster br; // création d'un object permettant de publier des tf
	geometry_msgs::TransformStamped transformStamped; // message pour la transmission sur le canal
	transformStamped.header.frame_id = "world"; // définition de la relation entre les repères
	transformStamped.child_frame_id = boatframe; // par exemple

	Eigen::Vector3f X = Eigen::Vector3f(initialx, initialy, 0); // vecteur d'état
	float dt = 0.1;

	while (ros::ok())
	{
		X = X + f1(X) * dt;

		tf::Quaternion q;
		q.setRPY(0, 0, X(2));  // roll, pitch, yaw

		ros::Time stamp = ros::Time::now();
		transformStamped.header.stamp = stamp; // date de la transformée
		transformStamped.transform.translation.x = X(0); // définition de la translation
		transformStamped.transform.translation.y = X(1);
		tf::quaternionTFToMsg(q, transformStamped.transform.rotation); // définition de la rotation
		
		visualization_msgs::Marker marker; // message de type visualization_msgs/Marker
		marker.header.frame_id = boatframe; // bateau affiché par rapport à son propre repère
		marker.header.stamp = stamp;
		marker.ns = ros::this_node::getNamespace();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 1;
		marker.scale.y = 1;
		marker.scale.z = 1;
		marker.pose.position.x = 0; // pas de mouvements relatifs du bateau par rapport à lui-même
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		marker.color.a = 1.0; // alpha = transparence
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.mesh_resource = "package://tp3/meshs/boat.dae";
		
		geometry_msgs::PoseStamped state; // état du bateau pour le controlleur
		state.pose.position.x = X(0);
		state.pose.position.y = X(1);
		tf::quaternionTFToMsg(q, state.pose.orientation);

		pub.publish(state); // publie le message marker sur le topic boatstate
		vispub.publish(marker); // publie le message marker sur le topic /boats
		ros::spinOnce(); // listener sur le topic commandboat
		br.sendTransform(transformStamped); // envoi du tf
		loop_rate.sleep(); // pause
	}

	return 0;
}