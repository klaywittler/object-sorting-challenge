// Ros packages and header files
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

// Standard packages and header files
//#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <sstream>
//using namespace std;
using std::cout;
using std::endl;

// Custom header file for GPIO
#include "BBB_GPIO.h"

// Define servo PWM bounds
#define PERIOD 20000000
#define LB 1350000
#define UB 1650000


// Define some pin numbers
int suctionCupPin = 65;

// Define controller gains
int kp_x = 120;
int kp_y = 600;
int ki_x = 70;
int ki_y = 100;
int kd_x = 80;

// Define computer vision errors
int ex = 0;
int ey = 0;
int ix = 0;
int iy = 0;
int ex_1 = 0;
int box = 0;

// Initialize pwm and suction cup values
int hold_pos = 1500000; //zero speed variable
int speed_mult = 10000; //multiplier to make pwm input easiear
int rotate = 1500000;
int extend = 1500000;
int drop = 1500000;
int suction= 0;

// Other variables.
int mode = 2;	// Set intial mode.
double begin = 0; //start timer
double current = 0; //track time
double  end = 0; //end timer
double rotate_time = 0; //time to bin

double suction_up = 3.0;
double suction_down = 1.5;


///////////////////////////// Helping functions ////////////////////////////////////////


// Function that limits the pulse width value for servo control
int limitPulse(int pulseWidth){
        if(pulseWidth>UB){
                pulseWidth = UB;
        }
        else if(pulseWidth<LB){
                pulseWidth = LB;
        }

        return pulseWidth;
}


// Function to be run when subscribed data is received
void callback(const std_msgs::Int32MultiArray::ConstPtr& msg){
	int i = 0;
	for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
		if(i==0){
			ex = *it;
		}
		else if(i==1){
			ey = *it;
		}
		else if(i==2){
			box = *it;
		}
		i++;
	}
	cout << "xe: " << ex << " ye: " << ey << " box: " << box << endl;
}


double get_binTime(int bin=0) {
	if (bin == 2) {
		rotate_time = 4;
	}
	else if (bin == 3) {
		rotate_time = 6;
	}
	else {
		rotate_time = 0;
	}

	return rotate_time;
}


////////////////////////////////// Main function //////////////////////////////////////


// Main program
int main(int argc, char **argv){
	cout << "Start Project 2 -- Control of Sorting Robot" << endl;	
	// Initialize ROS
	ros::init(argc, argv, "project2_controller_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	//timing variables
	begin = ros::Time::now().toSec(); //initialize begin time
	ros::Duration duration(5); // pause time at bottom for suction cup
	// Publisher
	ros::Publisher pub = nh.advertise<std_msgs::String>("controller_topic", 10, true);
	std_msgs::String msg;
	std::stringstream ss;
	ss << "track";
	msg.data = ss.str();
	//msg.data = "track";
	ROS_INFO("%s", msg.data.c_str());
	pub.publish(msg);
	//ros::spinOnce();

	//Subscriber
	ros::Subscriber sub = nh.subscribe("computerVision_topic", 10, callback);

	// Enable three PWM pins for servo control
	enablePWM(0, PERIOD);	// Rotate
	enablePWM(1, PERIOD);	// Extend
	enablePWM(2, PERIOD);	// Drop
	// Enable digital pins for suction cup control
	digitalEnable(suctionCupPin);
	digitalEnable(46);
	digitalWrite(46,0);

	// Main ROS loop, runs continously
	while(ros::ok()){


		// Run different modes of movement
		switch(mode){
		// Go to box with items
		case 1:
			//cout << "mode 1" << endl;
			rotate = 1535000;
			extend = hold_pos;
			drop = hold_pos;
			suction = 0;

                        end = 4;//get_binTime(box);
                        current = ros::Time::now().toSec();
                        if (current-begin > end) {
                                duration.sleep();
                                mode = 2;
                                cout << "time: " << begin << " moving to mode " << mode << endl;
				std_msgs::String msg;
       				std::stringstream ss;
        			ss << "track";
				msg.data = ss.str();
       				//msg.data = "track";
				ROS_INFO("%s", msg.data.c_str());
				pub.publish(msg);
				ros::spinOnce();
                        }

			break;

		// Find and move to item
		case 2:
			current = ros::Time::now().toSec();
			//cout << "mode 2" << endl;
			rotate = (UB/2+LB/2) + kp_x*ex + ki_x*ix + kd_x*(ex-ex_1)*10;
			extend = (UB/2+LB/2) + kp_y*ey + ki_y*iy;
			drop = hold_pos;
			suction = 0;

	                cout << "rotate: " << rotate << endl;
        	        cout << "extend: " << extend << endl;
               		//cout << "drop: " << drop << endl;
			//cout << "proportion_x: " << kp_x*ex << " proportional_y: " << kp_y*ey << endl;
			//cout << "derivative_x: " << -kd_x*(ex-ex_1)*10 << endl;
			//cout << "integral_x: " << ki_x*ix << " integral_y: " << ki_y*iy << endl;
			cout << "box: " << box <<endl;
			//book keeping
			ex_1 = ex;
			ix = ix + ex;
			iy = iy + ey;



			if (ix > 310) {
				ix = 310;
			}
			else if (ix < -310) {
				ix = -310;
			}

			if (iy > 310) {
				iy = 310;
			}
			else if (iy < -310) {
				iy = -310;
			}
			// if moved to item successfully, move to next mode
			if(box!=0){
				if (box == -9999) {
                                	rotate = hold_pos;
                                	extend = hold_pos;
					cout << "No target" << endl;
					ix = 0;
					iy = 0;
				}
				else if (box > 0){
					mode = 3;
					ix = 0;
					iy = 0;
					begin = ros::Time::now().toSec();
					cout << "time: " << begin << " moving to mode " << mode << endl;
				}
			}

			begin = ros::Time::now().toSec();

			break;

		// Drop suction cup
		case 3:
			cout << "mode 3" << endl;
			rotate = hold_pos;
			extend = hold_pos;
			drop = UB;
			suction = 1;

			end = suction_down;
			current = ros::Time::now().toSec();
			if (current-begin > end) {
	                        writePWM(0,limitPulse(rotate));
       		                writePWM(1,limitPulse(extend));
                	        writePWM(2,limitPulse(drop));
               			digitalWrite(suctionCupPin, suction);
                        	duration.sleep();


				duration.sleep();
				mode = 4;
				begin = ros::Time::now().toSec();
				cout << "time: " << begin << " moving to mode " << mode << endl;
			}

			break;

		// Lift suction cup
		case 4:
			cout << "mode 4" << endl;
			rotate = hold_pos;
                        extend = hold_pos;
                        drop = LB;
                        suction = 1;

                        end = suction_up;
			current = ros::Time::now().toSec();
			if (current-begin > end) {
				//mode = 6;
				mode = 5;
				begin = ros::Time::now().toSec();
				cout << "time: " << begin << " moving to mode " << mode << endl;
			}

                        break;

		// Go to colored box (destination box)
		case 5:
			cout << "mode 5" << endl;
			rotate = 1435000;
			extend = hold_pos;
			drop = hold_pos;
			suction = 1;

			end = 4;//get_binTime(box);
			current = ros::Time::now().toSec();
			if (current-begin > end) {
	                        writePWM(0,limitPulse(rotate));
        	                writePWM(1,limitPulse(extend));
                	        writePWM(2,limitPulse(drop));
                      		digitalWrite(suctionCupPin, suction);
                     	        duration.sleep();

				mode = 6;
				cout << "time: " << begin << " moving to mode " << mode << endl;
			}

			break;

		// Drop item
		case 6:
			cout << "mode 6" << endl;
			rotate = hold_pos;
			extend = hold_pos;
			drop = hold_pos;
			suction = 0;

			//mode = 2;
			mode = 1;
			begin = ros::Time::now().toSec();
	                writePWM(0,limitPulse(rotate));
        	        writePWM(1,limitPulse(extend));
        	        writePWM(2,limitPulse(drop));
        	        digitalWrite(suctionCupPin, suction);
			duration.sleep();

			break;
		}

		// Write PWM values and suction cup command
		/*cout << "sending rotate: " << limitPulse(rotate) << endl;
		cout << "sending extend: " << limitPulse(extend) << endl;
		cout << "sending drop: " << limitPulse(drop) << endl;
		cout << "The mode is currently: " << mode << endl; */
		writePWM(0,limitPulse(rotate));
		writePWM(1,limitPulse(extend));
		writePWM(2,limitPulse(drop));
		digitalWrite(suctionCupPin, suction);

	ros::spinOnce();
	loop_rate.sleep();
	}

	// Stop motors and suction cup
        writePWM(0,hold_pos);
        writePWM(1,hold_pos);
        writePWM(2,hold_pos);
       	digitalWrite(suctionCupPin, 0);

	cout << "End Project 2" << endl;
	return 0;
}
