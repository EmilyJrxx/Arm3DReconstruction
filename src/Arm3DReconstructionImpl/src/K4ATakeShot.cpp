# include "camera.h"

using namespace std;

const string rgb_topic = "/rgb/image_raw";
const string depth_topic = "/depth_to_rgb/image_raw";
const string cloud_topic = "/points2";
const string taketrigger_topic = "/signal/TakeShot";
const string readytomove_topic = "/signal/MoveToNext";
const string cameraPose_topic = "/pose/camera_pose";
const string output_dir = "";

const string camera_frame = "";
const string base_frame = "";

int main(int argc, char** argv){
    ros::init(argc, argv, "ROS_3DArmReconstruction_K4ATakeShot");
    CameraController k4a_controller(rgb_topic, 
                                    depth_topic, 
                                    cloud_topic, 
                                    taketrigger_topic, 
                                    readytomove_topic, 
                                    output_dir);
    k4a_controller.setTfFrameName(camera_frame, base_frame);
    // up til here, subscribers and publishers are all set, tf listener set too.
    tf2_ros::TransformListener tfListener(k4a_controller.getTf2Buffer());

    ros::spin();

    return 0;
}