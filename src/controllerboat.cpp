#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"

float u1 = 0; // commande
float targetx = 0; // abscisse de la cible
float targety = 0;  // ordonnée de la cible

float u1max = M_PI/10.; // saturation de la commande
float kp = 0.1; // coefficient de proportionnalité
float e = 0; // erreur à mettre à jour

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) // listener sur le topic boatstate
{
	float posx = msg->pose.position.x;
	float posy = msg->pose.position.y;

	double roll, pitch, yaw;
	double quatx= msg->pose.orientation.x;
    double quaty= msg->pose.orientation.y;
    double quatz= msg->pose.orientation.z;
    double quatw= msg->pose.orientation.w;
	tf::Quaternion quat(quatx, quaty, quatz, quatw);
	tf::Matrix3x3 mquat(quat);
    mquat.getRPY(roll, pitch, yaw);

	float ang = atan2(targety-posy, targetx-posx); // gisement de la cible
	e = 2*atan(tan((yaw-ang)/2)); // erreur à compenser
}

void chatterCallbackTarget(const geometry_msgs::PoseStamped::ConstPtr& msg) // listener sur le topic targetpos qui permet de choisir une position cible à atteindre (publication en ligne de commande par exemple)
{
	targetx = msg->pose.position.x;
	targety = msg->pose.position.y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller"); // node
	ros::NodeHandle n; // node handler
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("commandboat", 1000); // publie sur le topic command (local au namespace)
	ros::Subscriber sub = n.subscribe("boatstate", 1000, chatterCallback); // s'abonne au topic boatstate (local au namespace)
	ros::Subscriber targetsub = n.subscribe("targetpos", 1000, chatterCallbackTarget); // s'abonne au topic target (local au namespace)
	ros::Rate loop_rate(25);
	
	while (ros::ok())
	{
		geometry_msgs::Twist twistmsg; // message de type geometry_msgs/Twist pour la commande
		twistmsg.linear.x = 0;
		twistmsg.linear.y = 0;
		twistmsg.linear.z = 0;
		twistmsg.angular.x = 0;
		twistmsg.angular.y = 0;

		if (e > M_PI/4) { u1 = -u1max; } // calcul de la commande
		else
		{
			u1 = e*kp;
			if (u1 > u1max) { u1 = u1max; }
			else if (u1 < -u1max) { u1 = -u1max; }
		}

		twistmsg.angular.z = u1; // mise à jour du message avec la loi de commande
		
		pub.publish(twistmsg); // publie sur le topic commandboat
		ros::spinOnce(); // listener sur le topic boatstate et targetpos
		loop_rate.sleep(); // pause
	}

	return 0;
}