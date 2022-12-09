#include<ros/ros.h>
#include<std_msgs/Float64.h>


using namespace std;

class UWB
{
  public:
      ImageGrabber ( ros::NodeHandle& n ) : node ( n ) {
        left_image_pub.reset(new ros::Publisher(node.advertise<sensor_msgs::Image>("/euroc/left/image_raw", 50)));
        right_image_pub.reset(new ros::Publisher(node.advertise<sensor_msgs::Image>("/euroc/right/image_raw", 50)));

        left_image_info_pub.reset(new ros::Publisher(node.advertise<sensor_msgs::CameraInfo>("/euroc/left/camera_info", 50)));
        right_image_info_pub.reset(new ros::Publisher(node.advertise<sensor_msgs::CameraInfo>("/euroc/right/camera_info", 50)));
      }
      void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg) {
        left_image_pub->publish(*msg);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,  sensor_msgs::image_encodings::MONO8);
        cv::Mat img = cv_ptr->image;

        left_info.distortion_model = "plumb_bob";
        left_info.header = msg->header;
        left_info.height = img.rows;
        left_info.width = img.cols;

        // cv::remap(img,img,M1l,M2l,cv::INTER_LINEAR);
        // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
        // pub_msg->header = msg->header;
        // left_image_pub->publish(*pub_msg);
        left_image_info_pub->publish(left_info);
      }
      void GrabImageRight(const sensor_msgs::ImageConstPtr& msg) {
        right_image_pub->publish(*msg);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,  sensor_msgs::image_encodings::MONO8);
        cv::Mat img = cv_ptr->image;

        right_info.distortion_model = "plumb_bob";
        right_info.header = msg->header;
        right_info.height = img.rows;
        right_info.width = img.cols;

        // cv::remap(img,img,M1l,M2l,cv::INTER_LINEAR);
        // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
        // pub_msg->header = msg->header;
        // right_image_pub->publish(*pub_msg);
        right_image_info_pub->publish(right_info);
      }
    
      cv::Mat M1l,M2l,M1r,M2r;
      shared_ptr<ros::Publisher> dis12_pub, dis13_pub, dis, right_image_info_pub;
      sensor_msgs::CameraInfo left_info, right_info;
      ros::NodeHandle node;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "uwb_simulator");
  ros::NodeHandle n("~");
  cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      cerr << "ERROR: Wrong path to settings" << endl;
      return -1;
  }
  ImageGrabber igb ( n );

  cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
  std::vector<double> vector_k_l, vector_k_r, vector_p_l, vector_p_r, vector_r_l, vector_r_r, vector_d_l, vector_d_r;
  fsSettings["LEFT.K"] >> K_l;
  fsSettings["RIGHT.K"] >> K_r;
  vector_k_l.assign ( ( double* )K_l.datastart, ( double* )K_l.dataend );
  vector_k_r.assign ( ( double* )K_r.datastart, ( double* )K_r.dataend );
  boost::array<double, 9> array_K_l = { vector_k_l[0], vector_k_l[1], vector_k_l[2], vector_k_l[3], vector_k_l[4], vector_k_l[5], vector_k_l[6], vector_k_l[7], vector_k_l[8] };
  boost::array<double, 9> array_K_r = { vector_k_r[0], vector_k_r[1], vector_k_r[2], vector_k_r[3], vector_k_r[4], vector_k_r[5], vector_k_r[6], vector_k_r[7], vector_k_r[8] };


  fsSettings["LEFT.P"] >> P_l;
  fsSettings["RIGHT.P"] >> P_r;
  vector_p_l.assign ( ( double* )P_l.datastart, ( double* )P_l.dataend );
  vector_p_r.assign ( ( double* )P_r.datastart, ( double* )P_r.dataend );
  boost::array<double, 12> array_P_l = { vector_p_l[0], vector_p_l[1], vector_p_l[2], vector_p_l[3], vector_p_l[4], vector_p_l[5], vector_p_l[6], vector_p_l[7], vector_p_l[8], vector_p_l[9], vector_p_l[10], vector_p_l[11] };
  boost::array<double, 12> array_P_r = { vector_p_r[0], vector_p_r[1], vector_p_r[2], vector_p_r[3], vector_p_r[4], vector_p_r[5], vector_p_r[6], vector_p_r[7], vector_p_r[8], vector_p_r[9], vector_p_r[10], vector_p_r[11] };
  
  fsSettings["LEFT.R"] >> R_l;
  fsSettings["RIGHT.R"] >> R_r;
  vector_r_l.assign ( ( double* )R_l.datastart, ( double* )R_l.dataend );
  vector_r_r.assign ( ( double* )R_r.datastart, ( double* )R_r.dataend );
  boost::array<double, 9> array_R_l = { vector_r_l[0], vector_r_l[1], vector_r_l[2], vector_r_l[3], vector_r_l[4], vector_r_l[5], vector_r_l[6], vector_r_l[7], vector_r_l[8] };
  boost::array<double, 9> array_R_r = { vector_r_r[0], vector_r_r[1], vector_r_r[2], vector_r_r[3], vector_r_r[4], vector_r_r[5], vector_r_r[6], vector_r_r[7], vector_r_r[8] };

  fsSettings["LEFT.D"] >> D_l;
  fsSettings["RIGHT.D"] >> D_r;
  vector_d_l.assign ( ( double* )D_l.datastart, ( double* )D_l.dataend );
  vector_d_r.assign ( ( double* )D_r.datastart, ( double* )D_r.dataend );


  igb.left_info.D = vector_d_l;
  igb.right_info.D = vector_d_r;

  igb.left_info.K = array_K_l;
  igb.right_info.K = array_K_r;

  igb.left_info.P = array_P_l;
  igb.right_info.P = array_P_r;

  igb.left_info.R = array_R_l;
  igb.right_info.R = array_R_r;

  int rows_l = fsSettings["LEFT.height"];
  int cols_l = fsSettings["LEFT.width"];
  int rows_r = fsSettings["RIGHT.height"];
  int cols_r = fsSettings["RIGHT.width"];

  if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
          rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
  {
      cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
      return -1;
  }

  cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
  cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);


  ros::Subscriber sub_img_left = n.subscribe("/cam0/image_raw", 100, &ImageGrabber::GrabImageLeft, &igb);
  ros::Subscriber sub_img_right = n.subscribe("/cam1/image_raw", 100, &ImageGrabber::GrabImageRight, &igb);

  ros::spin();

  return 0;
}