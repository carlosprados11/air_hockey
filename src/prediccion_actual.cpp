// This program generate the proper position of the impact

//state=1 modo defensivo y_choque=20
//state=2 modo ofensivo y_choque=40

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "air_hockey/hockey_pose.h"
#include "air_hockey/todoOK.h"

#define frecuency 30.00
#define choque_defensivo 20.00
#define choque_ofensivo 40.00
#define X_min 0.00
#define X_max 54.00
#define x_home 35.00
#define y_home 20.00
#define registro 4

using namespace std;

int state;

float x_disco[3];
float y_disco[3];
float t_disco[3];

float Punto_Ant[2];
float Punto_Choque[3];

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
    for (int i=3; i>0; i--)
    {
        x_disco[i] = x_disco[i-1];
        y_disco[i] = y_disco[i-1];
        t_disco[i] = t_disco[i-1];
    }

    // Getting the position of the disk
    x_disco[0] = msg->x;
    y_disco[0] = msg->y;
    t_disco[0] = (float) ros::Time::now().sec + ((float) ros::Time::now().nsec/(float) 1E9);
    //ROS_WARN("%f %f %f %f",x_disco[0], x_disco[1],x_disco[2], x_disco[3]);
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


void interpola(float P1[3], float P2[3], float P3[3], float P4[3], float* ang, float* v) {
	float A[2];
	float a[3];
	float vel[3], punto[12];
	 *v = 0;
	int i;

    if (P1[0] == P2[0] && P1[1] == P2[1] || P2[0] == P3[0] && P2[1] == P3[1] || P3[0] == P4[0] && P3[1] == P4[1]) {
		*ang = 0;
		A[0] = P1[0];
		A[1] = P1[1];
	}
    else if ((abs(P1[0]-P4[0])<0.5)&&(abs(P1[1]-P4[1])<0.5))
    {
        
    }
	else {
        //Calculo del angulo interpolado
        a[0] = atan((P2[0] - P1[0]) / (P2[1] - P1[1]));
        a[1] = atan((P3[0] - P1[0]) / (P3[1] - P1[1]));
        a[2] = atan((P4[0] - P1[0]) / (P4[1] - P1[1]));
        *ang = (a[0] + a[1] + a[2]) / 3;

        //Calculo del  punto de la recta
        A[0] = P1[0];
        A[1] = P1[1];

        //Calculo velocidad
        for (i = 0; i < 12; i++) {
            if (i < 3)
                punto[i] = P1[i];
            else if (i < 6)
                punto[i] = P2[i - 3];
            else if (i < 9)
                punto[i] = P3[i - 6];
            else if (i < 12)
                punto[i] = P4[i - 9];
        }

        for (i = 0; i < 3; i++)
            vel[i] = sqrt(pow(punto[0] - punto[3 * (i + 1)], 2) + pow(punto[1] - punto[3 * (i + 1) + 1], 2)) / (punto[2] - punto[3 * (i + 1) + 2]);

        for (i = 0; i < 3; i++)
            *v = *v+ vel[i];

        *v=*v / 3;
    }
	
	Punto_Ant[0] = A[0];
    Punto_Ant[1] = A[1];
}

//Funcion trayectoria
void TrayD(float A[2], float* a, float v,int ope) {
	
    float inc_y1=0, inc_y,inc_x1, inc_t;
	float yb, xc, yc,xd, yd, rec;
	int col = 0;
	float choque[3];

	if (*a == 0) {
		choque[0] = x_home;
		choque[1] = y_home;
		choque[2] = t_disco[0];
	}
	else {
		if (ope == 1) yc = 20;
		else if (ope == 2) yc = 40;

		if (*a == 3.141516 / 2) {
			inc_t = (A[1] - yc)*v;
			choque[0] = A[0];
			choque[1] = yc;
			choque[2] = inc_t;
		}
		else {
            
			if (*a > 0) inc_x1 = A[0];
			else if (*a < 0) inc_x1 = 74 - A[0];

			inc_y1 = abs(tan(*a)*inc_x1);
			yb = A[1] - inc_y1;
			inc_y = abs(tan(*a) * 74);

			col = static_cast<int>(round((yb - yc) / inc_y));

			yd = yb - col*inc_y;
			if (*a < 0) {
				if (col % 2 == 0) {
					xd = 74;
					xc = 74 - (yd - yc) / abs(tan(*a));
				}
				else {
					xd = 0;
					xc = (yd - yc) / abs(tan(*a));
				}
			}
			else if (*a > 0) {
				if (col % 2 == 0) {
					xd = 0;
					xc = (yd - yc) / abs(tan(*a));
				}
				else {
					xd = 74;
					xc = 74 - (yd - yc) / abs(tan(*a));
				}
			}

			rec = sqrt(pow(inc_y, 2) + pow(74, 2))*col + sqrt(pow(inc_y1, 2) + pow(inc_x1, 2)) + sqrt(pow((yd - yc), 2) + pow((xd - xc), 2));
			inc_t = rec / abs(v);

			choque[0] = xc;
			choque[1] = yc;
			choque[2] = inc_t;
		}
	}

    Punto_Choque[0] = choque[0];
    Punto_Choque[1] = choque[1];
    Punto_Choque[2] = choque[2];

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

    float alpha, vel,T[2], P1[3], P2[3], P3[3], P4[3];

    // Periodic publication
    while (ros::ok())
    {
        P1[0] = x_disco[0];
        P1[1] = y_disco[0];
        P1[2] = t_disco[0];

        P2[0] = x_disco[1];
        P2[1] = y_disco[1];
        P2[2] = t_disco[1];

        P3[0] = x_disco[2];
        P3[1] = y_disco[2];
        P3[2] = t_disco[2];

        P4[0] = x_disco[3];
        P4[1] = y_disco[3];
        P4[2] = t_disco[3];

        //ROS_WARN("%f %f %f %f",P1[0], P2[0],P3[0], P4[0]);

        if (P4[2]!=0 && state!=0)
        {
            interpola(P4,P3,P2,P1,&alpha,&vel);

            T[0] = Punto_Ant[0];
            T[1] = Punto_Ant[1];

            TrayD(T, &alpha, vel, state);

            send_pos.x = Punto_Choque[0];
            send_pos.y = Punto_Choque[1];
            send_pos.tiempo = Punto_Choque[2] + t_disco[3];


            // Public the position
            ROS_INFO("[%f, %f]", send_pos.x, send_pos.y);

            // Publish the message in the proper topic
            if (t_disco[3]!=0)
                pub_pos.publish(send_pos);
        }
	    

        // Necesary to receive througth the subscribers
        ros::spinOnce();

        // Wait for the next publication
        loop_rate.sleep();

    }

}

