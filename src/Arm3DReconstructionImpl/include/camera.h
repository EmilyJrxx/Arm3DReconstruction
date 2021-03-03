# ifndef CAMERA_H
# define CAMERA_H

# include <ros/ros.h>
# include <message_filters/subscriber.h>
# include <message_filters/synchronizer.h>
# include <message_filters/sync_policies/approximate_time.h>
# include <sensor_msgs/image_encodings.h>
# include <sensor_msgs/Image.h>
# include <sensor_msgs/CameraInfo.h>
# include <sensor_msgs/PointCloud2.h>
# include <Arm3DReconstructionImpl/MoveReady.h>
# include <Arm3DReconstructionImpl/TakeShot.h>

# include <tf2_ros/transform_listener.h>
# include <iostream>

typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image,
        sensor_msgs::Image,
        sensor_msgs::PointCloud2, 
        Arm3DReconstructionImpl::TakeShot> MySyncPolicy;

class CameraController{
    public:
        CameraController();
        CameraController(const std::string & _rgb_topic, 
                         const std::string & _depth_topic,
                         const std::string & _cloud_topic, 
                         const std::string & _taketrigger_topic,
                         const std::string & _readytomove_topic);
        CameraController(const std::string & _rgb_topic, 
                         const std::string & _depth_topic,
                         const std::string & _cloud_topic, 
                         const std::string & _taketrigger_topic,
                         const std::string & _readytomove_topic,
                         const std::string & _output_dir);
        void Callback(sensor_msgs::Image::ConstPtr & rgb, 
                    sensor_msgs::Image::ConstPtr & depth,
                    sensor_msgs::PointCloud2::ConstPtr & cloud,
                    Arm3DReconstructionImpl::TakeShot::ConstPtr & signal,
                    const ros::Publisher & publisher);
        ~CameraController();
        tf2_ros::Buffer& getTf2Buffer();
        void setTfFrameName(const std::string& _camera_frame,
                            const std::string& _robot_frame);
    private:
        /*Topics*/
        std::string rgb_topic;
        std::string depth_topic;
        std::string cloud_topic;
        std::string taketrigger_topic;
        std::string readytomove_topic;
        /*Subcribers & Publishers*/
        ros::NodeHandle node;
        message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
        message_filters::Subscriber<sensor_msgs::Image> depth_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
        message_filters::Subscriber<Arm3DReconstructionImpl::TakeShot> taketrigger_sub;
        message_filters::Synchronizer<MySyncPolicy> sync;
        ros::Publisher move_signal_pub;

        tf2_ros::Buffer tfBuffer;

        std::string output_dir;
        int save_count;
};
# endif