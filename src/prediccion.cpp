// This program generate the proper position of the impact

//state=1 modo defensivo y_choque=20
//state=2 modo ofensivo y_choque=40

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "air_hockey/hockey_pose.h"
#include "air_hockey/todoOK.h"

#define frecuency 30.00
#define choque_defensivo 30.00
#define choque_ofensivo 50.00
#define X_min 0.00
#define X_max 54.00
#define x_home 35.00
#define y_home 30.00

using namespace std;

int state;

float x_disco = 0;
float y_disco = 0;
float t_disco = 0;
float x_disco_ant = 0;
float y_disco_ant = 0;
float t_disco_ant = 0;

air_hockey::hockey_pose send_pos;

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

// Callback Function for trajectory prediction
void Recibe_Pos_Disco(const air_hockey::hockey_pose::ConstPtr& msg)
{

    // Setting the previous position of the disk
    x_disco_ant = x_disco;
    y_disco_ant = y_disco;
    t_disco_ant = t_disco;

    // Getting the position of the disk
    x_disco = msg->x;
    y_disco = msg->y;
    t_disco = (float) ros::Time::now().sec + ((float) ros::Time::now().nsec/(float) 1E9);

}


//Callback Function for the state of the planificador
bool Devuelve_Estado(air_hockey::todoOK::Response &req, air_hockey::todoOK::Response &res)
{
    res.error = false;

    return true;
}


// Function to calculate the time
float calt(float P1[2], float P2[2], float vel)
{		//Supone que no hay roz

	float inc_t, rec, dx, dy;

	dx = abs(P1[0] - P2[0]);
	dy = P1[1] - P2[1];
	rec = sqrt(dx*dx + dy*dy);

	inc_t = rec / vel;

	return inc_t;
}


int main(int argc, char **argv)
{

    // Create the node
    ros::init(argc, argv, "prediccion");

    // Fully initialize the node
    ros::NodeHandle n;

    // Define the subscribers
	ros::Subscriber sub_state = n.subscribe("estado_global", 1, Recibe_Estado);
	ros::Subscriber sub_pos_disco = n.subscribe("pos_disco", 1, Recibe_Pos_Disco);

    // Define the publisher
    ros::Publisher pub_pos = n.advertise<air_hockey::hockey_pose>("pos_objetivo", 1);

    // Define the service
    ros::ServiceServer service = n.advertiseService("ask_predict", Devuelve_Estado);

    // Frecuency of publication
    ros::Rate loop_rate(frecuency);

    // Periodic publication
    while (ros::ok())
    {
        float x_col, y_col, choque[3];
        float dx, dy, e, v, t_choque, col1[2], col2[2];
        int bien = 0;
        float A[2], B[2], aux1[2], aux2[2], com[2], tA, tB, m;

        A[0] = x_disco_ant;
        A[1] = y_disco_ant;
        A[2] = t_disco_ant;
        B[0] = x_disco;
        B[1] = y_disco;
        B[2] = t_disco;

        aux1[0] = A[0];
        aux1[1] = A[1];
        tA = A[2];
        aux2[0] = B[0];
        aux2[1] = B[1];
        tB = B[2];

        t_choque = tA;
        col1[0] = aux1[0];
        col1[1] = aux1[1];
        dx = abs(A[0] - B[0]);
        dy = A[1] - B[1];
        e = sqrt(dx*dx + dy*dy);
        v = e / (tB - tA);

        // Avanza en perpendicular la porteria

        if (aux1[0]!=aux2[0])
        {
            m = (B[1] - A[1]) / (B[0] - A[0]);
        }
        else
        {
            choque[0] = x_disco;
            if (state==1)
                choque[1] = choque_defensivo;
            else if (state==2)
                choque[1] = choque_ofensivo;
            choque[2] = t_disco;
        }

        if (aux1[1]<=aux2[1])
        {	//El disco va hacia la porteria contraria
            bien = 1;
            choque[0] = x_home;
            choque[1] = y_home;
            choque[2] = t_disco;
        }


        while ((bien == 0)&&(t_disco_ant!=0)&&(aux1[1]>aux2[1]))
        {
            if(m<0)
                x_col = X_max;   //Chocara con x=54

            else if(m>0)
                x_col = 0;	 //Chocara con x=0

            else // no hay choque
            {
                if (aux1[0] == aux2[0]) {//m=infinito
                    x_col = aux1[0];
                    t_choque += calt(aux1, aux2, v);
                }
                else
                { //m=0
                    x_col = x_home;
                    t_choque = t_disco;
                }
                bien = 1; //No va a avanzar
                choque[0] = x_col;
                if (state == 1)
                    choque[1] = choque_defensivo;
                else
                    choque[1] = choque_ofensivo;
                choque[2] = t_choque;

            }

            if (bien == 0)
            {
                //Calculo y_col
                y_col = aux1[1] + (x_col - aux1[0]) / (aux2[0] - aux1[0])*(aux2[1] - aux1[1]);

                //Comprobacion y_col<y_choque
                if (state == 1 && y_col <= choque_defensivo) { //Modo defensivo
                                               //Se calcula t del trozo recorrido de trayectoria
                    col2[0] = aux1[0] + ((choque_defensivo - aux1[1]) / (aux2[1] - aux1[1]))*(aux2[0] - aux1[0]);
                    col2[1] = choque_defensivo;
                    t_choque += calt(col1, col2, v);

                    choque[0] = col2[0];
                    choque[1] = choque_defensivo;
                    choque[2] = t_choque;
                    bien = 1;
                }
                else if (state == 2 && y_col <= choque_ofensivo) {//Modo ofensivo
                                                   //Se calcula t del trozo recorrido de trayectoria
                    col2[0] = aux1[0] + ((choque_ofensivo - aux1[1]) / (aux2[1] - aux1[1]))*(aux2[0] - aux1[0]);
                    col2[1] = choque_ofensivo;
                    t_choque += calt(col1, col2, v);

                    choque[0] = col2[0];
                    choque[1] = choque_ofensivo;
                    choque[2] = t_choque;
                    bien = 1;
                }
                else { //y_col>y_choque

                       //Se calcula t de una trayectoria completa
                    col2[0] = x_col;
                    col2[1] = y_col;
                    t_choque += calt(col1, col2, v);
                    col1[0] = col2[0];
                    col1[1] = col2[1];

                    com[0] = aux1[0];
                    com[1] = aux1[1];
                    aux1[0] = aux2[0];
                    aux1[1] = 2 * y_col - aux2[1];
                    aux2[0] = com[0];
                    aux2[1] = 2 * y_col - com[1];
                    m = -m;
                }
            }
        }

        // Limites
        if (choque[0]<X_min)
            choque[0]=X_min;
        else if (choque[0]>X_max)
            choque[0]=X_max;

        send_pos.x = choque[0];
        send_pos.y = choque[1];
        send_pos.tiempo = choque[2];


        // Public the position
        ROS_INFO("[%f, %f]", send_pos.x, send_pos.y);

        // Publish the message in the proper topic
        if (t_disco_ant!=0)
            pub_pos.publish(send_pos);

        // Necesary to receive througth the subscribers
        ros::spinOnce();

        // Wait for the next publication
        loop_rate.sleep();

    }

}

