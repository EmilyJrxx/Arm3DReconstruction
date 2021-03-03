# include <ros/ros.h>
# include <tf2_ros/transform_listener.h>
# include <geometry_msgs/TransformStamped.h>

const std::string source_frame = "camera_optical_frame";
const std::string target_frame = "base_link";

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");
    ros::NodeHandle node;
    ros::Publisher transform_publisher = 
        node.advertise<geometry_msgs::TransformStamped>("c2b_tf", 10);

    tf2_ros::Buffer tfbuffer;
    tf2_ros::TransformListener tfListener(tfbuffer);

    ros::Rate rate(10.0);
    while (node.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfbuffer.lookupTransform(target_frame, source_frame, ros::Time(0));
            // time – The time at which the value of the transform is desired. (0 will get the latest)
        }
        catch (tf2::TransformException &ex){
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        transform_publisher.publish(transformStamped);
        rate.sleep();
    }
    return EXIT_SUCCESS;
}