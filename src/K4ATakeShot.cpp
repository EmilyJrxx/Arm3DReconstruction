# include <ros/ros.h>

# include <message_filters/macros.h>
# include <message_filters/subscriber.h>
# include <sensor_msgs/image_encodings.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>
# include <sensor_msgs/PointCloud2.h>
# include <std_msgs/Header.h>

# include <opencv2/core.hpp>
# include <opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>

# include <pcl/io/ply_io.h>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl/filters/filter.h>

# include <iostream>
# include <fstream>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace pcl;

const string rgb_topic = "/rgb/image_raw";
const string depth_topic = "/depth_to_rgb/image_raw";
const string cloud_topic = "/points2";
const string output_dir = "";

void Callback(/*self designed msg, containing a single bool value*/){
    // Call for camera service (Actively)
    // Converting Camera data
    // Clipping scene cloud
    // Frame Transforming
    // Saving Data
    // Publish "Next move ready" signal
}

int main (int argc, char** argv){
    // ROS Initialization
    // - Subscriber: Camera data
    // - Subscriber: take a shot signal
    // - Publisher:  send signal -- Making Next move --> MoveAndTake.py

    return 0;
}