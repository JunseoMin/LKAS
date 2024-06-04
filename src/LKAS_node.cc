#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "olny_cv/bird_eye_view.h"
#include "olny_cv/xyc_control.h"
#include "cv_bridge/cv_bridge.h"
#include "xycar_msgs/xycar_motor.h"

class LKAS_node
{
public:
  LKAS_node(ros::NodeHandle &nh)
  :biv_constructor(),controller(false,0.0,0.0,0.0,0.0,0.0)
  {
    nh.getParam("Kp", Kp);
    nh.getParam("Ki", Ki);
    nh.getParam("Kd", Kd);
    nh.getParam("stanly", stanly);
    nh.getParam("s_gain", s_gain);

    nh.getParam("speed", speed);
    nh.getParam("debug", debug);

    ROS_INFO("===== LKAS Node START =====");
    ROS_INFO("default: PID");

    if(stanly)
    {
      ROS_INFO("== Stanly Drive selected ==");
      ROS_INFO("stanly gain: %f", s_gain);
      Kp = 0.0;
      Ki = 0.0;
      Kd = 0.0;
    }
    else
    {
      ROS_INFO("=== PID Drive selected ===");
      ROS_INFO("Kp parameter: %f", Kp);
      ROS_INFO("Ki parameter: %f", Ki);
      ROS_INFO("Kd parameter: %f", Kd);
      s_gain = 0.0;
    };

    ROS_INFO("speed: %f", speed);

    debug = false;

    // Initialize the subscriber
    image_sub_ = nh.subscribe("/usb_cam/image_raw", 10, &LKAS_node::Img_CB, this);
    xycar_pub = nh.advertise<xycar_msgs::xycar_motor>("/xycar_motor", 10);

    controller = Xyc_control(stanly, Kp, Ki, Kd, s_gain, speed);
  }

private:
  ros::Subscriber image_sub_;
  ros::Publisher xycar_pub;

  BirdEyeView biv_constructor;
  Xyc_control controller;

  xycar_msgs::xycar_motor output;
  cv_bridge::CvImagePtr cv_ptr;
  
  cv::Mat in_img;
  cv::Mat biv; //bird eye view image
  
  //debug options
  bool debug;

  // controller param
  bool stanly;
  
  // gain setting
  double Kp;
  double Ki;
  double Kd;
  double s_gain;

  // speed param
  double speed;

  // Callback function to process image data
  void Img_CB(const sensor_msgs::Image::ConstPtr& msg)
  {
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    // ROS_INFO_STREAM("New Image from " << frame_id);
    
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    in_img = cv_ptr->image;
    cv::imshow("original view", in_img);

    biv = biv_constructor.bird_eye_generator(in_img);
    // cv::imshow("Bird eye view(filtered)", biv);
    if (stanly){
      output = controller.set_control_stanly(biv);
    }
    else{
      output = controller.set_control(biv);
    }

    xycar_pub.publish(output);
    
    if (debug){
      cv::imwrite("/home/junseo/div_sim/km7_v2/sim_ws/src/olny_cv/images/filtered.png",biv);
      debug = false;
      ROS_INFO("image saved!");
    }
    cv::waitKey(1);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LKAS");
  // ros::NodeHandle nh;
  ros::NodeHandle nh("~");
  
  // Create an instance of LKAS_node
  LKAS_node lkas_node(nh);
  
  ros::spin();

  return 0;
}
