// This program manage the state of the system

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "air_hockey/hockey_pose.h"
#include "air_hockey/todoOK.h"
#include "air_hockey/moveHomeMotors.h"
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

using namespace std;

#define frecuency 30.00
#define porcion_vision 1.0

float x_now;
float y_now;
float x_now1;
float y_now1;
float x_now2;
float y_now2;

air_hockey::hockey_pose send_pos;

// Callback Functions for robot position
void Recibe_Pos_Punch1(const air_hockey::hockey_pose::ConstPtr& msg)
{
    // Getting the time and position of contact
    x_now1 = msg->x;
    y_now1 = msg->y;
}

// Callback Functions for robot position
void Recibe_Pos_Punch2(const air_hockey::hockey_pose::ConstPtr& msg)
{
    // Getting the time and position of contact
    x_now2 = msg->x;
    y_now2 = msg->y;
}

// Main Function
int main(int argc, char **argv)
{

	// Create the node
    ros::init(argc, argv, "gestor");

    // Fully initialize the node
    ros::NodeHandle n;

    // Define the clients services
    ros::ServiceClient client_ask_vision = n.serviceClient<air_hockey::todoOK>("ask_vision");
    ros::ServiceClient client_ask_predict = n.serviceClient<air_hockey::todoOK>("ask_predict");
    ros::ServiceClient client_ask_planif = n.serviceClient<air_hockey::todoOK>("ask_planif");
    ros::ServiceClient client_ask_gestor_es = n.serviceClient<air_hockey::todoOK>("ask_gestor_es");
    ros::ServiceClient client_ask_arduino = n.serviceClient<air_hockey::todoOK>("ask_arduino");

	// Define the search of syncronism
	ros::ServiceClient client_sinc = n.serviceClient<air_hockey::moveHomeMotors>("moveHome");

    // Define the publisher
    ros::Publisher pub_state = n.advertise<std_msgs::Int8>("estado_global", 1);
    ros::Publisher pub_posicion_robot = n.advertise<std_msgs::Int8>("pos_robot", 1);

    // Define the subscriber
    ros::Subscriber sub_pos_punch1 = n.subscribe("pos_punch", 1, Recibe_Pos_Punch1);
    ros::Subscriber sub_pos_punch2 = n.subscribe("pos_actual", 1, Recibe_Pos_Punch2);

    // Frecuency of publication
    ros::Rate loop_rate(frecuency);

    int turno = 1;
    int fallos = 0;
    int state;

	// Search the syncronism
	for (int i=1; i<=2; i++)
	{
		air_hockey::moveHomeMotors sinc;
		sinc.request.id = i;

		if(client_sinc.call(sinc))
		{
           	ROS_WARN("Finding out the syncronism of Motor %d", i);
            if (sinc.response.goal)
			{
                ROS_INFO("Syncronism of Motor %d was successful", i);
			}
			else
			{
				ROS_ERROR("Syncronism of Motor %d was unsuccessful", i);
				return 0;
			}
            }
            else
            {
           	    ROS_ERROR("Failed to call Syncronism System");
                return 0;
            }
	}

	ROS_WARN("Syncronism of Motors was well");

    ROS_INFO("Modo de juego:");
    ROS_INFO("1- Defensivo.");
    ROS_INFO("2- Ofensivo.");
    cin >> state;

    while(state!=1 && state!=2)
    {
        ROS_INFO("Introduce un numero valido: ");
        cin >> state;
    }

    // Periodic publication
    while (ros::ok())
    {

        air_hockey::todoOK srv;
        srv.request.aux =  true;

        switch (turno)
        {
            case 1:
                if(client_ask_vision.call(srv))
                {
                    ROS_INFO("Correct: Vision System");
                    if (srv.response.error)
                        fallos++;
                }
                else
                {
                    ROS_ERROR("Failed to call Vision system");
                    fallos++;
                }
                break;
            case 2:
                if(client_ask_predict.call(srv))
                {
                    ROS_INFO("Correct: Prediction System");
                    if (srv.response.error)
                        fallos++;
                }
                else
                {
                    ROS_ERROR("Failed to call Prediction system");
                    fallos++;
                }
                break;
            case 3:
                if(client_ask_planif.call(srv))
                {
                    ROS_INFO("Correct: Planification System");
                    if (srv.response.error)
                        fallos++;
                }
                else
                {
                    ROS_ERROR("Failed to call Planification system");
                    fallos++;
                }
                break;
            case 4:
                if(client_ask_gestor_es.call(srv))
                {
                    ROS_INFO("Correct: Gestor I/O System");
                    if (srv.response.error)
                        fallos++;
                }
                else
                {
                    ROS_ERROR("Failed to call Gestor I/O system");
                    fallos++;
                }
                break;
            case 5:
                if(client_ask_arduino.call(srv))
                {
                    ROS_INFO("Correct: Arduino System");
                    if (srv.response.error)
                        fallos++;
                }
                else
                {
                    ROS_ERROR("Failed to call Arduino system");
                    fallos++;
                }
                break;
        }

        turno++;

        // Publish the global state
        if (turno>5)
        {
            std_msgs::Int8 msg;
            if (fallos!=0)
            {
                msg.data = 0;
            }
            else
            {
                msg.data = state;
            }

            pub_state.publish(msg);

            turno = 1;
            fallos = 0;
        }

        send_pos.x = porcion_vision*x_now1 + (1-porcion_vision)*x_now2;
        send_pos.y = porcion_vision*y_now1 + (1-porcion_vision)*y_now2;

        // Publish the message in the proper topic
        pub_posicion_robot.publish(send_pos);

        // Necesary to receive througth the subscribers
        ros::spinOnce();

        // Wait for the next publication
        loop_rate.sleep();

    }

    std_msgs::Int8 msg;
    msg.data = 0;
    pub_state.publish(msg);

    return 0;
}
