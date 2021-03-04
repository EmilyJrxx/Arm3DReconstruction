# include "camera.h"

# include <pcl_ros/transforms.h>
# include <std_msgs/Header.h>
# include <geometry_msgs/TransformStamped.h>
# include <opencv2/core.hpp>
# include <opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>

# include <pcl/io/ply_io.h>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl/filters/filter.h>

# include <iostream>
# include <fstream>

typedef pcl::PointXYZRGB PointT;

CameraController::~CameraController(){

}
CameraController::CameraController():
    rgb_topic(), 
    depth_topic(),
    cloud_topic(), 
    taketrigger_topic(),
    readytomove_topic(), // default value
    output_dir(),
    sync(MySyncPolicy(10))
{
    // TODO: check if ros::init executed, if not abort
    rgb_sub.subscribe(node, rgb_topic, 3);
    depth_sub.subscribe(node, depth_topic, 3);
    cloud_sub.subscribe(node, cloud_topic, 3);
    taketrigger_sub.subscribe(node, taketrigger_topic, 3);
    move_signal_pub = node.advertise<Arm3DReconstructionImpl::MoveReady> (readytomove_topic, 3);
    // TODO: check if it's OK when sync's SyncPolicy is not specified
    sync.connectInput(rgb_sub, depth_sub, cloud_sub, taketrigger_sub);
    sync.registerCallback(boost::bind(&CameraController::Callback, this, _1, _2, _3, _4, move_signal_pub));
    // TODO: when & where shall this tfListener be created
    static tf2_ros::TransformListener tfListener(tfBuffer);

    save_count = 0;
}

CameraController::CameraController(const std::string & _rgb_topic, 
                                   const std::string & _depth_topic,
                                   const std::string & _cloud_topic, 
                                   const std::string & _taketrigger_topic,
                                   const std::string & _readytomove_topic):
                                    rgb_topic(_rgb_topic), depth_topic(_depth_topic),
                                    cloud_topic(_cloud_topic), taketrigger_topic(_taketrigger_topic),
                                    readytomove_topic(_readytomove_topic),
                                    output_dir(""), // default value
                                    sync(MySyncPolicy(10))
{
    // TODO: check if ros::init executed, if not abort
    rgb_sub.subscribe(node, rgb_topic, 3);
    depth_sub.subscribe(node, depth_topic, 3);
    cloud_sub.subscribe(node, cloud_topic, 3);
    taketrigger_sub.subscribe(node, taketrigger_topic, 3);
    move_signal_pub = node.advertise<Arm3DReconstructionImpl::MoveReady> (readytomove_topic, 3);
    // TODO: check if it's OK when sync's SyncPolicy is not specified
    sync.connectInput(rgb_sub, depth_sub, cloud_sub, taketrigger_sub);
    sync.registerCallback(boost::bind(&CameraController::Callback, this, _1, _2, _3, _4, move_signal_pub));
    // TODO: when & where shall this tfListener be created
    static tf2_ros::TransformListener tfListener(tfBuffer);

    save_count = 0;  
}

CameraController::CameraController(const std::string & _rgb_topic, 
                                    const std::string & _depth_topic,
                                    const std::string & _cloud_topic, 
                                    const std::string & _taketrigger_topic,
                                    const std::string & _readytomove_topic,
                                    const std::string & _output_dir):
                                    rgb_topic(_rgb_topic), depth_topic(_depth_topic),
                                    cloud_topic(_cloud_topic), taketrigger_topic(_taketrigger_topic),
                                    readytomove_topic(_readytomove_topic),
                                    output_dir(_output_dir),
                                    sync(MySyncPolicy(10))
{
    // TODO: check if ros::init executed, if not abort
    rgb_sub.subscribe(node, rgb_topic, 3);
    depth_sub.subscribe(node, depth_topic, 3);
    cloud_sub.subscribe(node, cloud_topic, 3);
    taketrigger_sub.subscribe(node, taketrigger_topic, 3);
    move_signal_pub = node.advertise<Arm3DReconstructionImpl::MoveReady> (readytomove_topic, 3);
    // TODO: check if it's OK when sync's SyncPolicy is not specified
    sync.connectInput(rgb_sub, depth_sub, cloud_sub, taketrigger_sub);
    sync.registerCallback(boost::bind( &CameraController::Callback, this, _1, _2, _3, _4, move_signal_pub));

    // TODO: when & where shall this tfListener be created
    static tf2_ros::TransformListener tfListener(tfBuffer);

    save_count = 0;
}

void CameraController::Callback(const sensor_msgs::Image::ConstPtr & rgb, 
                                const sensor_msgs::Image::ConstPtr & depth,
                                const sensor_msgs::PointCloud2::ConstPtr & cloud,
                                const Arm3DReconstructionImpl::TakeShot::ConstPtr & signal,
                                const ros::Publisher & signal_publisher
                                )
{
    // Call for camera service (Actively)
    // Converting Camera data
    // Clipping scene cloud
    // Frame Transforming
    // Saving Data
    // Publish "Next move ready" signal
    // if (signal->if_take){
        cv_bridge::CvImagePtr rgb_ptr;
        cv_bridge::CvImagePtr depth_ptr;
        try{
            rgb_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return ;
        }
        try{
            depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return ;
        }
        cv::Mat rgb_frame = rgb_ptr->image;
        cv::Mat depth_frame = depth_ptr->image;
        pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*pcl_cloud, *cloud_filtered, indices);
        pcl::PointCloud<PointT>::Ptr cloud_transformed (new pcl::PointCloud<PointT>);
        // Cloud Transforming
        // -- receiving camera2base transformation from MoveAndTake.py
        // -- transforming p(base) = T(c->b)p(camera)
        geometry_msgs::TransformStamped c2b_transform = tfBuffer.lookupTransform(robot_frame, camera_frame, ros::Time(0));
        pcl_ros::transformPointCloud(*cloud_filtered, *cloud_transformed, c2b_transform.transform);

        // Data Writing
        std::string rgb_name = output_dir+"rgbs/"+std::to_string(++save_count)+".jpg";
        std::string depth_name = output_dir+"depth/"+std::to_string(save_count)+".exr"; //32FC1
        std::string cloud_name = output_dir+"clouds/"+std::to_string(save_count)+".ply";
        cv::imwrite(rgb_name, rgb_frame);
        cv::imwrite(depth_name, depth_frame);
        pcl::io::savePCDFileBinary(cloud_name, *cloud_transformed); // saving Transformed cloud
        std::cout << "Scene: " << save_count << "saved:" << cloud_name << std::endl;

        // Signal Publishing
        Arm3DReconstructionImpl::MoveReady move_signal;
        move_signal.header.seq = 0;
        move_signal.header.stamp = ros::Time::now();
        move_signal.header.frame_id = "/signal";
        move_signal.if_ready = true;
        // publisher.publish(move_signal);
    // }
    // else{
    //     std::cerr << "Not receiving Take a Shot signal, Aborting.\n";
    //     return;
    // }    
}

tf2_ros::Buffer& CameraController::getTf2Buffer(){
    return this->tfBuffer;
}

void CameraController::setTfFrameName(const std::string& _camera_frame,
                                      const std::string& _robot_frame)
{
    camera_frame = _camera_frame;
    robot_frame  = _robot_frame;
}