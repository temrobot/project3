#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <iostream>
#include <string>
#include <sstream>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_bot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pix_thresh = 250;
    int num_white_pix = 0;
    int sum_white_pix_index = 0;
    int white_pix_index = 0;
    int i = 0;
    int intensity = 0;
    int mean_index = 0;
    float linear_vel = 0;
    float max_linear_vel = 0.2;
    float angular_rate = 0.0;
    float turn_gain = -0.01;

    ROS_INFO_STREAM("Image height x width "<<img.height<<" x "<<img.width<<" Image encoding "<<img.encoding);

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    while ( i<(img.height * img.width * 3) )
    {
        //ROS_INFO_STREAM("pix RGB "<<int(img.data[i])<<" "<<int(img.data[i+1])<<" "<<int(img.data[i+2]));
        intensity = int(img.data[i]);        
        if (intensity > white_pix_thresh)
        {
            num_white_pix++;
            white_pix_index = i%(img.width*3);
            sum_white_pix_index = sum_white_pix_index + white_pix_index;
        }
        i = i+3;
    }
    if (num_white_pix != 0)
    {
        mean_index = sum_white_pix_index / (num_white_pix * 3);
        ROS_INFO_STREAM("mean_index "<<mean_index<<" num_white_pix "<<num_white_pix);
        // Then, identify if this pixel falls in the left, mid, or right side of the image
        linear_vel = 5000/(float(num_white_pix));
        if (linear_vel>max_linear_vel)
            linear_vel = max_linear_vel;

        angular_rate = turn_gain * (float(mean_index) - float(img.width)/2.0);
        ROS_INFO_STREAM("angular_rate "<<angular_rate<<" mean_index "<<mean_index);

        if (mean_index < 250)
            drive_robot(linear_vel, angular_rate);

        else if (mean_index < 550)
            drive_robot(linear_vel, angular_rate);

        else
            drive_robot(linear_vel, angular_rate);
    }
    else
    {
        ROS_INFO_STREAM("No white ball");
            drive_robot(0.0, 0.0);
    }

    


    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
