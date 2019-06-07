// This program generate the proper trajectory of the robot

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "air_hockey/hockey_pose.h"
#include "air_hockey/todoOK.h"
#include <math.h>

#define frecuency 30.00
#define NUM_MOTORES 2.00

int state;
float x_dest;
float y_dest;

double t_now;
double t_ant;
air_hockey::hockey_pose send_pos;

// Callback Functions for system state (ConstPtr make a message copy)
void Recibe_Estado(const std_msgs::Int8::ConstPtr& msg)
{
    state = msg->data;

    t_ant = (float) ros::Time::now().sec + ((float) ros::Time::now().nsec/(float) 1E9);

    if (state==1) // Error state
    {
        ROS_WARN("Estado Defensivo");
    }
    else if (state==2) // Error state
    {
        ROS_WARN("Estado Ofensivo");
    }
    else
    {
        ROS_ERROR("Something is wrong");
    }
}

// Callback Function for save the goal position
void Recibe_Destino(const air_hockey::hockey_pose::ConstPtr& msg)
{

    // Getting the desired position of the robot
    x_dest = msg->x;
    y_dest = msg->y;

}

//Callback Function for the state of the planificador
bool Devuelve_Estado(air_hockey::todoOK::Response &req, air_hockey::todoOK::Response &res)
{
    res.error = false;

    return true;
}


// Main Function
int main(int argc, char **argv)
{

	// Create the node
    ros::init(argc, argv, "gestor_es");

    // Fully initialize the node
    ros::NodeHandle n;

    // Define the subscribers
	ros::Subscriber sub_pos_dest = n.subscribe("pos_dest", 1, Recibe_Destino);
	ros::Subscriber sub_state = n.subscribe("estado_global", 1, Recibe_Estado);

    // Define the publisher
    ros::Publisher pub_pos_objetivo = n.advertise<air_hockey::hockey_pose>("control", 1);

    // Define the service
    ros::ServiceServer service = n.advertiseService("ask_gestor_es", Devuelve_Estado);

    // Frecuency of publication
    ros::Rate loop_rate(frecuency);

    // Periodic publication
    while (ros::ok())
    {

        if (state==0) // Error state
        {
            ROS_INFO("Error State!");
        }
        else // Normal state
        {
            t_now = (float) ros::Time::now().sec + ((float) ros::Time::now().nsec/(float) 1E9);
            ROS_INFO("%f",t_now);
            if (t_now > (t_ant+0.1))
            {
                state = 0;
            }
            else
            {
                send_pos.x = x_dest;
                send_pos.y = y_dest;

                // Public the position for information
                ROS_INFO("[%f, %f]", x_dest, y_dest);

                // Publish the message in the proper topic
                pub_pos_objetivo.publish(send_pos);
            }
            
        }

        // Necesary to receive througth the subscribers
        ros::spinOnce();

        // Wait for the next publication
        loop_rate.sleep();
    }


    return 0;
}
