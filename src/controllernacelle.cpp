#include <math.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include "tf/tf.h"

float targetx = 0; // position absolue de la cible 
float targety = 0;
float u2 = 0; // commande en vitesse angulaire de la tourelle/nacelle

void chatterCallbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg) // listener sur le topic targetpos qui permet de choisir une position cible à atteindre (publication en ligne de commande par exemple)
{
	targetx = msg->pose.position.x;
	targety = msg->pose.position.y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controllernacelle"); // node
	ros::NodeHandle n; // node handler

	std::string nacelleframe;
	n.param<std::string>("nacelleframe", nacelleframe, ros::this_node::getNamespace() + "nacelle");

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("commandnacelle", 1000); // publie sur le topic commandnacelle (local au namespace)
	ros::Subscriber targetsub = n.subscribe("targetpos", 1000, chatterCallbackTarget); // s'abonne au topic targetpos (local au namespace)

	// Objets permettant de recevoir toutes les tf2.
	tf2_ros::Buffer tfBuffer; // 10s de buffer des tf2
	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Rate rate(10.0);

	while (ros::ok())
	{
		geometry_msgs::TransformStamped transformStamped;
		geometry_msgs::Twist twistmsg;

		try
		{
			// target frame, from frame, at time (ros::Time(0) = latest available)
			transformStamped = tfBuffer.lookupTransform("world", nacelleframe, ros::Time(0));
			// Ajout optionel possible d'un timeout comme 4ième argument ex: "ros::Duration(0.0)"
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		// Les données sont accessibles dans la variable transformStamped
		// transformStamped.transform.translation.x par exemple
		float posnacellex = transformStamped.transform.translation.x;
		float posnacelley = transformStamped.transform.translation.y;
		float cap = tf::getYaw(transformStamped.transform.rotation);

		float ang = atan2(posnacelley-targety, posnacellex-targetx); // calcul du gisement (angle relatif) de la cible
		float gisement = M_PI - cap + ang;
		if (gisement > M_PI) { gisement = gisement - 2*M_PI; }
		if (gisement < -M_PI) { gisement = 2*M_PI + gisement; }

		u2 = gisement; // tend à faire converger le gisement vers 0, c'est à dire que la tourelle pointe vers la cible
		twistmsg.angular.z = u2; // mise à jour du message avec la loi de commande u2

		pub.publish(twistmsg); // publie la commande sur le topic commandnacelle
		ros::spinOnce(); // listener sur le topic targetpos
		rate.sleep();
	}

	return 0;
}