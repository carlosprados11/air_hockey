// This program generate the proper trajectory of the robot

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "air_hockey/hockey_pose.h"
#include "air_hockey/todoOK.h"
#include <math.h>
#include <iostream>
#include <fstream>

using namespace std;

#define frecuency 30.00
#define T_ATAQUE 0.20
#define x_home 35.00
#define y_home 20.00
#define y_mitad 65.00

int state;

float t_now;
float x_now;
float y_now;

float t_final;
float x_final;
float y_final;

float x_disco;
float y_disco;

air_hockey::hockey_pose send_pos;

// Output file
ofstream fs("datos.txt");

// Callback Functions for system state (ConstPtr make a message copy)
void Recibe_Estado(const std_msgs::Int8::ConstPtr& msg)
{
    state = msg->data;

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

// Callback Functions for robot position
void Recibe_Posicion(const air_hockey::hockey_pose::ConstPtr& msg)
{
    // Getting the time and position of contact
    x_now = msg->x;
    y_now = msg->y;

    fs << 0 << " " << x_now << " " << y_now << endl; 
}

// Callback Function for trajectory prediction
void Recibe_Objetivo(const air_hockey::hockey_pose::ConstPtr& msg)
{

    // Getting the time and position of contact
    t_final = msg->tiempo;
    x_final = msg->x;
    y_final = msg->y;

}

// Callback Function for trajectory prediction
void Recibe_Pos_Disco(const air_hockey::hockey_pose::ConstPtr& msg)
{

    // Getting the time and position of the disk
    x_disco = msg->x;
    y_disco = msg->y;

    fs << 1 << " " << x_disco << " " << y_disco << endl;

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
    ros::init(argc, argv, "planificador");

    // Fully initialize the node
    ros::NodeHandle n;

    // Define the subscribers
	ros::Subscriber sub_pose_obj = n.subscribe("pos_objetivo", 1, Recibe_Objetivo);
	ros::Subscriber sub_pose_now = n.subscribe("pos_robot", 1, Recibe_Posicion);
	ros::Subscriber sub_state = n.subscribe("estado_global", 1, Recibe_Estado);
	ros::Subscriber sub_pos_disco = n.subscribe("pos_disco", 1, Recibe_Pos_Disco);

    // Define the publisher
    ros::Publisher pub_pos = n.advertise<air_hockey::hockey_pose>("pos_dest", 1);

    // Define the service
    ros::ServiceServer service = n.advertiseService("ask_planif", Devuelve_Estado);

    // Frecuency of publication
    ros::Rate loop_rate(frecuency);

    //t_now=0;
    float periodo = 1/frecuency;

    // Periodic publication
    while (ros::ok())
    {

        //t_now = t_now + periodo;

        t_now = (float) ros::Time::now().sec + ((float) ros::Time::now().nsec/(float) 1E9);
        if (state==1) // Defensive state
        {
            // Possible issue
            if ((x_final==0)||(y_final==0))
            {
                send_pos.x = x_now;
                send_pos.y = y_now;
            }
            // Waitting a little
            else if (y_disco>y_mitad)
            {
                send_pos.x = x_home;
                send_pos.y = y_home;
            }
            // Correct the position
            else
            {
                send_pos.x = x_final;
                send_pos.y = y_final;
            }

        }
        else if (state==2) // Ofensive state
        {
            float inc_t;
            float inc_x;
            float inc_y;

            inc_t = t_final-t_now;
            inc_x = x_final-x_now;
            inc_y = y_final-y_now;

            //ROS_INFO("%f", inc_t);

            // Keep the position in the home
            //if ((x_final==0)||(y_final==0)||(inc_x==0)||(inc_y==0)||(inc_t>T_ATAQUE)||(inc_t<0)||(y_disco>y_mitad))
            if ((x_final==0)||(y_final==0)||(inc_t>T_ATAQUE)||(inc_t<0)||(y_disco>y_mitad))
            {
                send_pos.x = x_home;
                send_pos.y = y_home;
            }
            // Atack the disk
            else
            {
//                float veces = inc_t*frecuency;
                float veces = inc_t/periodo;
                if (abs(veces)<0.01)
                {
                    veces = 100000;
                }

                send_pos.x = x_now + inc_x/veces;
                send_pos.y = y_now + inc_y/veces;
            }

        }

        // Send the time
        send_pos.tiempo = t_now;

        // Public the position
        ROS_INFO("Objetivo: [%f %f, %f]", t_now, x_now, y_now);

        // Publish the message in the proper topic
        pub_pos.publish(send_pos);

        // Necesary to receive througth the subscribers
        ros::spinOnce();

        // Wait for the next publication
        loop_rate.sleep();
    }

    fs.close();

    return 0;
}
