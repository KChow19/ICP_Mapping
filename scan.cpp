#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <termios.h>
#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace ros::serialization;
namespace ser = ros::serialization;

int fileNumber = 0;
std::string filename;

void saveData(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    uint32_t serial_size =
        ros::serialization::serializationLength(*scan);
    std::vector<uint8_t> buffer(serial_size);
    ser::OStream stream(buffer.data(), serial_size);
    ser::serialize(stream, *scan);

    //Outputting the text file
    ofstream myfile(filename);
    myfile.write(reinterpret_cast<const char*>(buffer.data()),
                 buffer.size());
    myfile.close();    
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guy"); //TODO: Change name to something useful
    ROS_INFO("Tuturu");
    ros::NodeHandle n("~");

    n.param<std::string>("my_file", filename, "data0.txt");
    ROS_INFO("%s", filename.c_str());

    //Subscriber for the /scan topic
    ros::Subscriber sub1 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, saveData);
    ros::spin();
}
