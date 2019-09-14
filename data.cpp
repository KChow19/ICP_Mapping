#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <laser_values/TutorialsConfig.h>

using namespace ros::serialization;
namespace ser = ros::serialization;

visualization_msgs::Marker points, line_strip, line_list;
visualization_msgs::Marker temp_line_list;

double pastError = 999999999999;

double xTranslation = 0;
double yTranslation = 0;
double theta = 0;
double pastXTranslation = 0;
double pastYTranslation = 0;
double pastTheta = 0;

//Initial partial derivatives
double xT_deriv = 0;
double yT_deriv = 0;
double theta_deriv = 0;

//Learning rate values
double xT_learning_rate = 0.0025;
double yT_learning_rate = 0.01;
double theta_learning_rate = 0.0004;

//TODO: Have the the updates stop when the gradient is small or when it reaches the lowest error
void update_weights()
{
    xT_deriv = 0;
    yT_deriv = 0;
    theta_deriv = 0;
    int N = line_list.points.size();

    for(int i = 0; i < N; i += 2)
    {
        //Calculate partial derivative
        //xTranslation
        xT_deriv = -(line_list.points.at(i+1).y * sin(theta)
                    - line_list.points.at(i+1).x * cos(theta)
                    - xTranslation + line_list.points.at(i).x) / 
                sqrt(pow(line_list.points.at(i+1).y * sin(theta)
                        - line_list.points.at(i+1).x * cos(theta)
                        - xTranslation + line_list.points.at(i).x, 2)
                   + pow(-line_list.points.at(i+1).y * cos(theta)
                        - line_list.points.at(i+1).x * sin(theta)
                        - yTranslation + line_list.points.at(i).y, 2));

        //yTranslation
        yT_deriv = -(-line_list.points.at(i+1).y * cos(theta) 
                    - line_list.points.at(i+1).x * sin(theta)
                    - yTranslation + line_list.points.at(i).y) / 
                 sqrt(pow(line_list.points.at(i+1).y * sin(theta)
                        - line_list.points.at(i+1).x * cos(theta)
                        - xTranslation + line_list.points.at(i).x, 2)
                    + pow(-line_list.points.at(i+1).y * cos(theta)
                        - line_list.points.at(i+1).x * sin(theta)
                        - yTranslation + line_list.points.at(i).y, 2));

        //Angle
        theta_deriv = (sin(theta) 
                        * (line_list.points.at(i+1).y * (line_list.points.at(i).y - yTranslation)
                        + line_list.points.at(i+1).x * (line_list.points.at(i).x - xTranslation))
                      + cos(theta)
                        *(line_list.points.at(i+1).y * (line_list.points.at(i).x - xTranslation)
                        + line_list.points.at(i+1).x * (yTranslation - line_list.points.at(i).y)))
                      /
                      sqrt(
                         pow(-line_list.points.at(i+1).y * sin(theta)
                            + line_list.points.at(i+1).x * cos(theta)
                            + xTranslation - line_list.points.at(i).x, 2)
                         +
                         pow(line_list.points.at(i+1).y * cos(theta)
                            + line_list.points.at(i+1).x * sin(theta)
                            + yTranslation - line_list.points.at(i).y, 2));

    }
    xTranslation += (xT_deriv / (double)N) * xT_learning_rate;
    yTranslation += (yT_deriv / (double)N) * yT_learning_rate;
    theta += (theta_deriv / (double)N) * theta_learning_rate;
    
}

double calcError()
{
    double e = 0;
    for(int i = 0; i < line_list.points.size(); i += 2)
    {
        e += sqrt(pow(line_list.points.at(i).x - line_list.points.at(i+1).x,2)
             + pow(line_list.points.at(i).y - line_list.points.at(i+1).y,2));
    }
    return e;
}

void translation(sensor_msgs::PointCloud& target, sensor_msgs::PointCloud& src)
{
    if(xTranslation != pastXTranslation || yTranslation != pastYTranslation || theta != pastTheta)
    {
        for(int i = 0; i < target.points.size(); i++)
        {
            src.points[i].x = target.points[i].x * cos(theta) - target.points[i].y * sin(theta) 
                                + xTranslation;
            src.points[i].y = target.points[i].x * sin(theta) + target.points[i].y * cos(theta) 
                                + yTranslation;
        }
        pastXTranslation = xTranslation;
        pastYTranslation = yTranslation;
        pastTheta = theta;
    }
}

void resetPointCloud(sensor_msgs::PointCloud& src, sensor_msgs::PointCloud& temp)
{
    for(int i = 0; i < src.points.size(); i++)
    {
        temp.points[i].x = src.points[i].x;
        temp.points[i].y = src.points[i].y;
    }
}

void callback(laser_values::TutorialsConfig &config, uint32_t level) 
{
    ROS_INFO("Reconfigure Request: %f %f %f %d", 
            config.x, config.y, config.theta,
            config.size);
    xTranslation = config.x;
    yTranslation = config.y;
    theta = config.theta;
}
void updatePointDiff(const sensor_msgs::PointCloud& display)
{
    int j = 0;
    for(int i = 0; i < display.points.size(); i++)
    {
        line_list.points.at(j).x = display.points[i].x;
        line_list.points.at(j).y = display.points[i].y;
        line_list.points.at(j).z = display.points[i].z;
        j+=2;
    }
}

void findClosestPoint(/*const visualization_msgs::Marker::ConstPtr& visual*/ const sensor_msgs::PointCloud& dataA, const sensor_msgs::PointCloud& dataB)
{
    //Clearing the list
    line_list.points.clear();

    for(int i = 0; i < dataB.points.size(); i++)
    {
        double lowestDifference = 9999999999.0;
        geometry_msgs::Point p;

        //TODO: Comment Here
        p.x = dataB.points[i].x;
        p.y = dataB.points[i].y;
        p.z = dataB.points[i].z;
        line_list.points.push_back(p);

        for(int j = 0; j < dataA.points.size(); j++)
        {
            double tempDiff = sqrt(pow((dataB.points[i].x - dataA.points[j].x),2)
                                + pow((dataB.points[i].y - dataA.points[j].y),2));
            if(tempDiff < lowestDifference)
            {
                lowestDifference = tempDiff;
                p.x = dataA.points[j].x;
                p.y = dataA.points[j].y;
                p.z = dataA.points[j].z;

            }
        }
        //TODO: Comment here
        line_list.points.push_back(p);
    }
}

int main(int argc, char **argv)
{      
    ros::init(argc, argv, "buddy"); //TODO: Change name to something useful
	ros::NodeHandle n("~");

    std::string filename1;
    std::string filename2;
    n.param<std::string>("my_file1", filename1, "data0.txt");
    n.param<std::string>("my_file2", filename2, "data1.txt");
    ROS_INFO("%s", filename1.c_str());
    ROS_INFO("%s", filename2.c_str());

    //Opening first file
    FILE* f1 = fopen(filename1.c_str(), "r");
    //Determine file size
    fseek(f1, 0, SEEK_END);
    size_t size = ftell(f1);
    std::vector<uint8_t> data(size);
    rewind(f1);
    fread(data.data(), sizeof(char), size, f1);
    
    //Opening second file
    FILE* f2 = fopen(filename2.c_str(), "r");
    //Determine file size
    fseek(f2, 0, SEEK_END);
    size = ftell(f2);
    std::vector<uint8_t> data2(size);
    rewind(f2);
    fread(data2.data(), sizeof(char), size, f2);
    
    //First file
    ser::IStream ros_stream(data.data(), data.size());
    sensor_msgs::LaserScan lazerScan;
    ros::serialization::Serializer<sensor_msgs::LaserScan>::read(
        ros_stream, lazerScan);

    //Second file
    ser::IStream ros_stream2(data2.data(), data2.size());
    sensor_msgs::LaserScan ls;
    ros::serialization::Serializer<sensor_msgs::LaserScan>::read(
        ros_stream2, ls);
    
    //Converting LaserScan message to PointCloud
    laser_geometry::LaserProjection projector_;

    //Creating PointCloud for dataA
    sensor_msgs::PointCloud cloud1;
    projector_.projectLaser(lazerScan, cloud1);

    //Creating PointCloud for dataB
    sensor_msgs::PointCloud cloud2;
    projector_.projectLaser(ls, cloud2);

    //Creating Display PointCloud for dataB
    sensor_msgs::PointCloud displayCloud;
    projector_.projectLaser(ls, displayCloud);

    //Creating Point Cloud publishers
	ros::Publisher dataA_pub = n.advertise<sensor_msgs::PointCloud>("/dataA", 1);
    ros::Publisher dataB_pub = n.advertise<sensor_msgs::PointCloud>("/dataB", 1);

    //Creating Visulizer publisher
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    //Visualizer
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "laser";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    //Setting id markers
    line_list.id = 2;

    //Setting marker types
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.01;
    line_list.scale.x = 0.01;

    // Line list is red
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    //Creating visualizer
    findClosestPoint(cloud1, cloud2);

    //Reset the display values
    resetPointCloud(cloud2, displayCloud);

    dynamic_reconfigure::Server<laser_values::TutorialsConfig> server;
    dynamic_reconfigure::Server<laser_values::TutorialsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Rate loop_rate(1);
	while (ros::ok())
    {
        //Publish point cloud data
        dataA_pub.publish(cloud1);
        dataB_pub.publish(displayCloud);
        
        //Publish Visualization data
        marker_pub.publish(line_list);

        //Update based on translation
        translation(cloud2, displayCloud);

        //Updating parameters
        if(pastError > calcError())
        {        
            for(int i = 0; i < 1000; i++)
            {
                update_weights();
            }
            pastError = calcError();
        }

        //Display partial values
        ROS_INFO("xT_deriv: [%f], yT_deriv: [%f], theta_deriv: [%f]", xT_deriv, yT_deriv, theta_deriv);

        //Update green lines
        updatePointDiff(displayCloud);

        //Print error
        ROS_INFO("Error: [%f]", calcError());

        ros::spinOnce();
	    loop_rate.sleep();
    }
	return 0;
}
