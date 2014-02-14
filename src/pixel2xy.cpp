#include <iostream>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
//#include <ardrone_autonomy/navdata_altitude.h>
//#include <tum_ardrone/filter_state.h>

class Pixel2XY {
  sensor_msgs::CameraInfo cam_info;
  image_geometry::PinholeCameraModel cam_model_;
  ros::Publisher pub;
  //double altitude;
  //ardrone_autonomy::navdata_altitude altitude;
  //tum_ardrone::filter_state state;
  geometry_msgs::PoseStampedConstPtr pose;
  
public:
  
  Pixel2XY(ros::NodeHandle n, std::string calib_file, std::string camera_name, std::string cmd_topic)
  {
    if ( !camera_calibration_parsers::readCalibration(calib_file, camera_name, cam_info) )
    { std::cout << calib_file << std::endl;
      ROS_ERROR("Pixel2XY: unable to read camera calibration file"); return; }
    if ( !cam_model_.fromCameraInfo(cam_info) )
    { ROS_ERROR("Pixel2XY: unable to create pinhole camera model !!"); return; }
    std::cout << "\n\nDefault POSE " << pose/*->pose.position.z*/ << "\n\n";
    pub = n.advertise<geometry_msgs::Point>(cmd_topic, 1000);
    
  }
  
  void updateCamInfo(const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    if ( !cam_model_.fromCameraInfo(cam_info) )
      ROS_ERROR("Pixel2XY: unable to create pinhole camera model !!");
    ROS_INFO("PIXEL2XY:  CAMERA INFO UPDATED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
  
//void updateAltitude(ardrone_autonomy::navdata_altitudeConstPtr & msg) { altitude = msg->est_state; }
//void updateAltitude(tum_ardrone::filter_stateConstPtr & msg){ altitude = msg->z; //state = *msg; }
  void updatePose(const geometry_msgs::PoseStampedConstPtr& msg) { pose = msg; }
  
  bool ready() { return cam_model_.initialized() && pose; }

  void transform(const geometry_msgs::PointConstPtr& pix)
  {
    cv::Point2d pix_raw(pix->x, pix->y);
    cv::Point2d pix_rect = cam_model_.rectifyPoint(pix_raw);
    cv::Point3d ray = cam_model_.projectPixelTo3dRay(pix_rect);
    double z = pose->pose.position.z;
    geometry_msgs::Point target;
    target.x = z * ray.x;
    target.y = z * ray.y;
    target.z = z;
    pub.publish(target);
  }
  
};

int main(int argc, char *argv[]) {
  std::string calib_file, camera_name, pix_topic, cmd_topic;
  ros::init(argc, argv, "pixel2xy"); //, ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  n.getParam("pixel2xy/calib_file",calib_file);
  n.param<std::string>("pixel2xy/camera_name",camera_name,"camera");
  n.getParam("pixel2xy/pix_topic", pix_topic);
  n.getParam("pixel2xy/cmd_topic", cmd_topic);
  
  Pixel2XY pixel2XY(n, calib_file, camera_name, cmd_topic);
  ros::Subscriber cam_info_sub = n.subscribe(std::string("ardrone/bottom/camera_info"), 5, &Pixel2XY::updateCamInfo, &pixel2XY);
  ros::Subscriber pose_sub = n.subscribe(std::string("ardrone/pose"), 1000, &Pixel2XY::updatePose, &pixel2XY);
  while (!pixel2XY.ready()) ros::spinOnce();
  ROS_INFO("PIXEL2XY ready");
  ros::Subscriber pix_sub = n.subscribe(pix_topic, 1000, &Pixel2XY::transform, &pixel2XY);
  ros::spin();
  return 0;
}