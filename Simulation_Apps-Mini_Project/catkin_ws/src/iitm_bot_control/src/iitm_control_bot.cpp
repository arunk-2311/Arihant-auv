#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <sstream>
#include <cmath>

// Pre-defined constants
#define PI 3.14157
#define INVALID -1
// Limit the maximum integrated error to control the role that I component plays in PID
#define MAX_ERROR 10.0
// THreshold distance for front sensor to immediately stop the bot
#define WALL_COLLISION_THRESH 0.1
// Optimum distance to inintiate rotation towards left/right direction
#define WALL_TURN_THRESH 0.30
// Optimum distance to maintain from left/right walls in case of errorneous reading from sensor
#define WALL_OPTIMUM_DIST 0.24
// Sensor threshold to assume left/right sensor is looking into free space
#define FREE_SPACE_READING_LR_THRESH 0.5

using namespace std;

// ENUM for tracking the Bot control states
enum robot_states
{
	ROBOT_START = 0,
	ROBOT_FIND_DIRECTION,
	ROBOT_MOVE_FORWARD,
	ROBOT_TURN_LEFT,
	ROBOT_TURN_RIGHT,
	ROBOT_REVERSE,
	ROBOT_STOP,
	ROBOT_REACHED_GOAL,
	ROBOT_INVALID_STATE,
};

// ENUM for tracking the rotation state
enum robot_rotate
{
	ROBOT_ROT_INITIATE = 0,
	ROBOT_ROT_GOTO_TARGET_ORIENTATION,
	ROBOT_ROT_TARGET_ORIENTATION_REACHED,
	ROBOT_ROT_ALIGN,
	ROBOT_ROT_ALIGN_REACHED,
	ROBOT_ROT_COMPLETED,
	ROBOT_ROT_INVALID,
};

// ENUM to control type of messages to print
enum print_msg_class
{
	PRINT_SENSOR_DATA = 0,
	PRINT_ROTATION_STATE,
	PRINT_POSE,
	PRINT_PID_STATE,
	PRINT_INVALID,
};

class IITM_Bot
{
private:
	// Sensors reading
	float left_sensor_dist_last = 0;
	float left_sensor_dist = 0;
	float left_sensor_range_min = 0;
	float left_sensor_range_max = 0;

	float right_sensor_dist_last = 0;
	float right_sensor_dist = 0;
	float right_sensor_range_min = 0;
	float right_sensor_range_max = 0;

	float front_sensor_dist_last = 0;
	float front_sensor_dist = 0;
	float front_sensor_range_min = 0;
	float front_sensor_range_max = 0;

	// The amount of error to expect in sensor readings (std dev of noise)
	float sensor_error_thresh = 0.05;
	// The amount of error to expect in odometry measurements
	float pose_error_thresh = 0.01;

	// Current pose of the bot
	float pose_x = 0;
	float pose_y = 0;
	float pose_z = 0;
	float orientation_x = 0;
	float orientation_y = 0;
	float orientation_z = 0;

	// PID state variables
	float error = 0;
	float error_sum = 0;
	float last_error = 0;
	
	float pid_coefficient = 0;

	// P, I and D constants
	float k_p = 75;
	float k_d = 3.0;
	float k_i = 0.1;
	
	// Misc. state variables
	// To track time between each odometry message to accurately track the pose
	float last_odom_msg_time_stamp_msecs = 0;
	// State variable to indicate robot rotation state
	int rotation_state = ROBOT_ROT_INVALID;
	// Variable to store the target orinetation theta to be achieved
	float target_orientation = INVALID;

public:
	// Public variable to inidcate BOT control status
	int robot_state = INVALID;

	// Constuctor to initialize robot state
	IITM_Bot(int robot_state)
	{
		this->robot_state = robot_state;
	}

	// Helper fn. to get theta value in 0 to 2*PI range
	float get_valid_theta(float in_theta)
	{
		float out_theta = in_theta;

		if (in_theta >= 2*PI)
		{
			out_theta = in_theta - 2*PI;
		}
		else if (in_theta < 0)
		{
			out_theta = in_theta + 2*PI;
		}

		return out_theta;
	}

	// Helper fn. to get theta in radians from degreee
	float deg_to_rad(float deg)
	{
		return (deg * PI) / 180;
	}

	// Callback fn to read left sensor value
	void left_sensor_read_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		// Update Left sensor min range
		this->left_sensor_range_min = msg->range_min;
		// Update Left sensor max range
		this->left_sensor_range_max = msg->range_max;

		// Update left sensor reading in weighter avg. manner.
		this->left_sensor_dist_last = this->left_sensor_dist;

		if (std::isfinite(msg->ranges[0]))
		{
			this->left_sensor_dist = 0.3*this->left_sensor_dist_last + 0.7*msg->ranges[0];
		}
		else
		{
			this->left_sensor_dist = 0.3*this->left_sensor_dist_last + 0.7*this->left_sensor_range_max;
		}
	}

	// Callback fn to read right sensor value
	void right_sensor_read_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		// Update Right sensor min range
		this->right_sensor_range_min = msg->range_min;
		// Update Right sensor max range
		this->right_sensor_range_max = msg->range_max;

		// Update right sensor reading in weighter avg. manner.
		this->right_sensor_dist_last = this->right_sensor_dist;

		if (std::isfinite(msg->ranges[0]))
		{
			this->right_sensor_dist = 0.3*this->right_sensor_dist_last + 0.7*msg->ranges[0];
		}
		else
		{
			this->right_sensor_dist = 0.3*this->right_sensor_dist_last + 0.7*this->right_sensor_range_max;
		}
	}

	// Callback fn to read front sensor value
	void front_sensor_read_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		// Update front sensor min range
		this->front_sensor_range_min = msg->range_min;
		// Update front sensor max range
		this->front_sensor_range_max = msg->range_max;

		// Update front sensor reading in weighter avg. manner.
		this->front_sensor_dist_last = this->front_sensor_dist;

		if (std::isfinite(msg->ranges[0]))
		{
			this->front_sensor_dist = 0.3*this->front_sensor_dist_last + 0.7*msg->ranges[0];
		}
		else
		{
			this->front_sensor_dist = 0.3*this->front_sensor_dist_last + 0.7*this->front_sensor_range_max;
		}
			
	}

	// Callback fn to read odometry msgs
	void odom_read_cb(const nav_msgs::Odometry::ConstPtr& msg)
	{
		float time_period, curr_time;

		// Update Pose parameters
		this->pose_x = msg->pose.pose.position.x;
		this->pose_y = msg->pose.pose.position.y;
		this->pose_z = msg->pose.pose.position.z;

		// Find the time interval w.r.t last odom message
		curr_time = (msg->header.stamp.nsec/1000000 + msg->header.stamp.sec * 1000);
		time_period = curr_time - this->last_odom_msg_time_stamp_msecs;
		this->last_odom_msg_time_stamp_msecs = curr_time;

		// Track orientation in Z direction
		this->orientation_z += msg->twist.twist.angular.z * (time_period/1000);

		// Normalize orientation in 0 to 2*Pi range
		this->orientation_z = get_valid_theta(this->orientation_z);
	}

	// Helper fn. to print state variables for debugging
	void print_variables(int var_class)
	{
		ROS_INFO("\n Robot state: %d", this->robot_state);		

		if (var_class == PRINT_SENSOR_DATA)
		{
			ROS_INFO("\n Left Sensor: Range: min: %f, max: %f, current: %f, previous: %f",
					this->left_sensor_range_min, this->left_sensor_range_max, this->left_sensor_dist, this->left_sensor_dist_last);
			ROS_INFO("\n Right Sensor: Range: min: %f, max: %f, current: %f, previous: %f",
					this->right_sensor_range_min, this->right_sensor_range_max, this->right_sensor_dist, this->right_sensor_dist_last);
			ROS_INFO("\n Front Sensor: Range: min: %f, max: %f, current: %f, previous: %f",
					this->front_sensor_range_min, this->front_sensor_range_max, this->front_sensor_dist, this->front_sensor_dist_last);
		}
		else if (var_class == PRINT_ROTATION_STATE)
		{
			ROS_INFO("\n Robot State: %d, Rotation State: %d, Curr Ori: %f, Target Ori: %f",
					this->robot_state, this->rotation_state, this->orientation_z, this->target_orientation);
		}
		else if (var_class == PRINT_PID_STATE)
		{
			ROS_INFO("\n Current error: %f, Sum: %f, Prev error: %f, PID coeff val: %f",
					this->error, this->error_sum, this-> last_error, this->pid_coefficient);
		}
		else if (var_class == PRINT_POSE)
		{
			ROS_INFO("\n Current Pose: X: %f, Y: %f, Z: %f",
					this->pose_x, this->pose_y, this->pose_z);
			ROS_INFO("\n Current Orientation: X: %f, Y: %f, Z: %f",
					this->orientation_x, this->orientation_y, this->orientation_z);
		}

		return;
	}

	// Helper fn. to check if task is completed or not. For now, assume if all sensors detect no boundary constraint, then goal is acccomplished
	int check_if_reached_goal()
	{
		if ((this->front_sensor_dist > 1.0) && (this->right_sensor_dist > 1.0) && (this->left_sensor_dist > 1.0))
			return 1;
		
		return 0;
	}

	// State ROBOT_FIND_DIRECTION fn: Find direction to move
	void find_direction(geometry_msgs::Twist* vel_cmd)
	{
		// For debug:
		print_variables(PRINT_SENSOR_DATA);

		if (this->front_sensor_dist > (0.3 + this->sensor_error_thresh))
		{
			// Safe to move forward
			this->robot_state = ROBOT_MOVE_FORWARD;
		}
		else if ((this->left_sensor_dist > (0.3 + this->sensor_error_thresh)) && (this->left_sensor_dist >= this->right_sensor_dist))
		{
			// Turn Left
			this->robot_state = ROBOT_TURN_LEFT;
		}
		else if ((this->right_sensor_dist > (0.3 + this->sensor_error_thresh)) && (this->right_sensor_dist >= this->left_sensor_dist))
		{
			// Turn Right
			this->robot_state = ROBOT_TURN_RIGHT;
		}
		else if (check_if_reached_goal())
		{
			// Reached goal state. stop and celebrate
			this->robot_state = ROBOT_REACHED_GOAL;
		}
		else
		{
			// Rotate to find best direction to move
			vel_cmd->angular.z = 0.08;
		}
	}

	// State ROBOT_MOVE_FORWARD fn: PID control to move forward
	void move_forward(geometry_msgs::Twist* vel_cmd)
	{
		print_variables(PRINT_SENSOR_DATA);

		// Check if reached goal
		if (check_if_reached_goal())
		{
			this->robot_state = ROBOT_REACHED_GOAL;
			return;
		}

		// Check if going to collide or not
		if (this->front_sensor_dist < WALL_COLLISION_THRESH)
		{
			this->robot_state = ROBOT_STOP;
			return;
		}

		// Check if forward motion is possible or not, and whether turn is required or not 
		if (this->front_sensor_dist < WALL_TURN_THRESH)
		{
			this->robot_state = ROBOT_FIND_DIRECTION;
			return;
		}

		// Calculate the PID coefficient
		// Calculate the error in bot orientation w.r.t to optimum position of equidistant from both walls 
		this->error = this->left_sensor_dist - this->right_sensor_dist;

		// Check if whether left sensor reading should be considered valid or not. If not, re-calculate the error
		if (this->left_sensor_dist > FREE_SPACE_READING_LR_THRESH)
		{
			this->error = WALL_OPTIMUM_DIST - this->right_sensor_dist;
		}
		else if (this->right_sensor_dist > FREE_SPACE_READING_LR_THRESH)
		{
			this->error = this->left_sensor_dist - WALL_OPTIMUM_DIST;
		}

		// Update total error for I component
		this->error_sum = this->error_sum + this->error;

		if (this->error_sum > MAX_ERROR)
		{
			this->error_sum = MAX_ERROR;
		}

		// Calculate PID coefficient
		this->pid_coefficient = ((this->k_p * this->error) + (this->k_i * this->error_sum) + (this->k_d * (this->error - this->last_error)));
		this->last_error = this->error;

		print_variables(PRINT_PID_STATE);

		// Issue velocity command based on PID threshold value
		if (abs(this->pid_coefficient) > 0.02)
		{
			vel_cmd->linear.x = 0.3;
			vel_cmd->angular.z = this->pid_coefficient * 0.03;
		}
		else
		{
			vel_cmd->linear.x = 0.3;
		}
	}

	// State ROBOT_TURN_LEFT fn: Turn left
	void turn_left(geometry_msgs::Twist* vel_cmd)
	{
		print_variables(PRINT_SENSOR_DATA);
		print_variables(PRINT_ROTATION_STATE);

		// Initialize rotation
		if (this->rotation_state == ROBOT_ROT_INVALID)
		{
			ROS_INFO("\n Start rotation");
			this->rotation_state = ROBOT_ROT_INITIATE;
		}

		// Calculate target orientation direction
		if (this->rotation_state == ROBOT_ROT_INITIATE)
		{
			ROS_INFO("\n Go to a specific orientation");
			this->target_orientation = this->orientation_z + deg_to_rad(85);
			this->target_orientation = get_valid_theta(this->target_orientation);
			this->rotation_state = ROBOT_ROT_GOTO_TARGET_ORIENTATION;
			print_variables(PRINT_ROTATION_STATE);
		}

		// Check if target orientation reached
		if (this->rotation_state == ROBOT_ROT_GOTO_TARGET_ORIENTATION)
		{
			if (abs(this->orientation_z - this->target_orientation) < this->pose_error_thresh)
			{
				this->rotation_state = ROBOT_ROT_TARGET_ORIENTATION_REACHED;
				ROS_INFO("\n Turned Left");
			}
			else
			{
				vel_cmd->linear.x = 0.1;
				vel_cmd->angular.z = 0.4;
			}
		}

		// If target orientation reached, fine tune the rotation
		if (this->rotation_state == ROBOT_ROT_TARGET_ORIENTATION_REACHED)
		{
			this->rotation_state = ROBOT_ROT_ALIGN;
			print_variables(PRINT_ROTATION_STATE);
		}

		// Fine tune rotation
		if (this->rotation_state == ROBOT_ROT_ALIGN)
		{
			print_variables(PRINT_SENSOR_DATA);

			if (((this->right_sensor_dist < FREE_SPACE_READING_LR_THRESH) && (this->right_sensor_dist_last > this->right_sensor_dist)) ||
				((this->front_sensor_dist > FREE_SPACE_READING_LR_THRESH) && (this->front_sensor_dist_last < this->front_sensor_dist)))
			{
				vel_cmd->linear.x = 0.1;
				vel_cmd->angular.z = 0.2;
			}
			else
			{
				this->rotation_state = ROBOT_ROT_ALIGN_REACHED;
			}
		}

		// If fine tuning done, initiate forward motion
		if (this->rotation_state == ROBOT_ROT_ALIGN_REACHED)
		{
			print_variables(PRINT_ROTATION_STATE);
			this->rotation_state = ROBOT_ROT_COMPLETED;
		}

		if (this->rotation_state == ROBOT_ROT_COMPLETED)
		{
			this->robot_state = ROBOT_MOVE_FORWARD;
			this->rotation_state = ROBOT_ROT_INVALID;
		}

		return;
	}

	// State ROBOT_TURN_RIGHT fn: Turn right
	void turn_right(geometry_msgs::Twist* vel_cmd)
	{
		print_variables(PRINT_SENSOR_DATA);
		print_variables(PRINT_ROTATION_STATE);

		// Initialize rotation
		if (this->rotation_state == ROBOT_ROT_INVALID)
		{
			ROS_INFO("\n Start rotation");
			this->rotation_state = ROBOT_ROT_INITIATE;
		}

		// Calculate target orientation direction
		if (this->rotation_state == ROBOT_ROT_INITIATE)
		{
			ROS_INFO("\n Go to a specific orientation");
			this->target_orientation = this->orientation_z - deg_to_rad(85);
			this->target_orientation = get_valid_theta(this->target_orientation);
			this->rotation_state = ROBOT_ROT_GOTO_TARGET_ORIENTATION;
			print_variables(PRINT_ROTATION_STATE);
		}

		// Check if target orientation reached
		if (this->rotation_state == ROBOT_ROT_GOTO_TARGET_ORIENTATION)
		{
			if (abs(this->orientation_z - this->target_orientation) < this->pose_error_thresh)
			{
				this->rotation_state = ROBOT_ROT_TARGET_ORIENTATION_REACHED;
				ROS_INFO("\n Turned Right");
			}
			else
			{
				vel_cmd->linear.x = 0.1;
				vel_cmd->angular.z = -0.4;
			}
		}

		// If target orientation reached, fine tune the rotation
		if (this->rotation_state == ROBOT_ROT_TARGET_ORIENTATION_REACHED)
		{
			this->rotation_state = ROBOT_ROT_ALIGN;
			print_variables(PRINT_ROTATION_STATE);
		}

		// Fine tune rotation
		if (this->rotation_state == ROBOT_ROT_ALIGN)
		{
			print_variables(PRINT_SENSOR_DATA);

			if (((this->left_sensor_dist < FREE_SPACE_READING_LR_THRESH) && (this->left_sensor_dist_last > this->left_sensor_dist)) ||
				((this->front_sensor_dist > FREE_SPACE_READING_LR_THRESH) && (this->front_sensor_dist_last < this->front_sensor_dist)))
			{
				vel_cmd->linear.x = 0.1;
				vel_cmd->angular.z = -0.2;
			}
			else
			{
				this->rotation_state = ROBOT_ROT_ALIGN_REACHED;
			}
		}

		// If fine tuning done, initiate forward motion
		if (this->rotation_state == ROBOT_ROT_ALIGN_REACHED)
		{
			print_variables(PRINT_ROTATION_STATE);
			this->rotation_state = ROBOT_ROT_COMPLETED;
		}

		if (this->rotation_state == ROBOT_ROT_COMPLETED)
		{
			this->robot_state = ROBOT_MOVE_FORWARD;
			this->rotation_state = ROBOT_ROT_INVALID;
		}

		return;
	}
};

int main(int argc, char **argv)
{
	// Create ROS node
	ros::init(argc, argv, "iitm_bot_controller");
	ros::NodeHandle node_obj;

	// Update robot state every 1/100th second (10msec)
	ros::Rate loop_rate(30);

	// Initialize the BOT
	IITM_Bot bot_instance(ROBOT_START);

	// Publisher node to publish velocities commnad
	ros::Publisher vel_cmd_pub = node_obj.advertise<geometry_msgs::Twist>("/iitm_bot/cmd_vel", 10);

	// Subscriber node to subscribe to left, front and right sensors + odometry messages
	ros::Subscriber left_sensor_subs = node_obj.subscribe("/iitm_bot/ray/left_sensor", 30, &IITM_Bot::left_sensor_read_cb, &bot_instance);
	ros::Subscriber right_sensor_subs = node_obj.subscribe("/iitm_bot/ray/right_sensor", 30, &IITM_Bot::right_sensor_read_cb, &bot_instance);
	ros::Subscriber front_sensor_subs = node_obj.subscribe("/iitm_bot/ray/front_sensor", 30, &IITM_Bot::front_sensor_read_cb, &bot_instance);
	ros::Subscriber odom_subs = node_obj.subscribe("/iitm_bot/odom", 30, &IITM_Bot::odom_read_cb, &bot_instance);

	while (ros::ok())
	{
		// Vel cmd message to be published. Initialized to 0
		geometry_msgs::Twist vel_cmd;

		// Read all subscribed topics
		ros::spinOnce();

		// Based on Robot state, carry out the functionality
		if (bot_instance.robot_state == ROBOT_INVALID_STATE)
		{
			ROS_INFO("\nError! Alert! Stalling");
			vel_cmd.linear.x = 0;
			vel_cmd.linear.y = 0;
			vel_cmd.linear.z = 0;
			vel_cmd.angular.x = 0;
			vel_cmd.angular.y = 0;
			vel_cmd.angular.z = 0;
		}
		else if (bot_instance.robot_state == ROBOT_REACHED_GOAL)
		{
			ROS_INFO("\nReached Goal. Hurray");
			vel_cmd.linear.x = 0;
			vel_cmd.linear.y = 0;
			vel_cmd.linear.z = 0;
			vel_cmd.angular.x = 0;
			vel_cmd.angular.y = 0;
			vel_cmd.angular.z = 0;
		}
		else if (bot_instance.robot_state == ROBOT_START)
		{
			ROS_INFO("\nLet's Navigate. :)");
			bot_instance.robot_state = ROBOT_FIND_DIRECTION;
		}
		else if (bot_instance.robot_state == ROBOT_STOP)
		{
			ROS_INFO("\nWas going to collide. Phew");
			bot_instance.robot_state = ROBOT_FIND_DIRECTION;
		}
		else if (bot_instance.robot_state == ROBOT_FIND_DIRECTION)
		{
			bot_instance.find_direction(&vel_cmd);
		}
		else if (bot_instance.robot_state == ROBOT_MOVE_FORWARD)
		{
			bot_instance.move_forward(&vel_cmd);
		}
		else if (bot_instance.robot_state == ROBOT_TURN_LEFT)
		{
			bot_instance.turn_left(&vel_cmd);
		}
		else if (bot_instance.robot_state == ROBOT_TURN_RIGHT)
		{
			bot_instance.turn_right(&vel_cmd);
		}

		// For Debug: Current Robot state + Velocity command message being send
		ROS_INFO("\nUmang: Robot state: %d, cmd_vel linear: %f, %f, %f, angular: %f, %f, %f", bot_instance.robot_state, vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.linear.z, vel_cmd.angular.x, vel_cmd.angular.y, vel_cmd.angular.z);

		// Publish the velocity command
		vel_cmd_pub.publish(vel_cmd);
		loop_rate.sleep();
	}

	return 0;
}
