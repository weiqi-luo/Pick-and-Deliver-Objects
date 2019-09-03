
#include <ros/ros.h>

// cv specific includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

// msg,srv specific includes
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/PointStamped.h>
#include <final_msg_srv/DesiredObject.h>
#include <final_msg_srv/Coordinates_3D.h>
#include <final_msg_srv/done.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/CameraInfo.h>

// actionclient specific includes
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/shared_ptr.hpp>

// tf specific includes
#include <tf/transform_listener.h>

// others
#include <vector>
#include <string.h>
#include <cv_bridge/cv_bridge.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> joint_control_client;
using namespace std;
using namespace cv;


class From2Dto3D
{
public:
      //! The node handle
      ros::NodeHandle nh_;
      //! Node handle in the private namespace
      ros::NodeHandle priv_nh_;

      //! Define publishers and subscribers
      ros::Publisher pub1;
      ros::Publisher pub2;
      ros::Subscriber sub1;
      ros::Subscriber sub2;
      ros::Subscriber cameraInfoSub_;
      ros::ServiceServer srv1;
      ros::ServiceServer srv2;
      ros::ServiceServer srv3;
      ros::ServiceServer srv4;
      ros::ServiceClient client2;

      geometry_msgs::PointStamped camera_point;
      geometry_msgs::PointStamped base_point;
      final_msg_srv::Coordinates_3D base_point_;
      final_msg_srv::DesiredObject msg_something;
      final_msg_srv::done msg_done;
      std::string object;
      int box_number;

      struct box_
      {
          string name;
          double prabability;
          int a;
          int b;
          int c;
          int d;
      }box[30];
      std::string box_name;
      double depth;

      std::string OPENCV_WINDOW;

      tf::TransformListener listener;

      bool gotInfo;

      float fx;
      float fy;
      float cx;
      float cy;
      float fovx;
      float fovy;
      int width;
      int height;

      cv_bridge::CvImagePtr img_cv;
      Vec3f point_cv;
      ros::Time timer;

      //-------------------- callbacks --------------------/

      //! Subscriber1 : Process clusters
      void processCloud(const sensor_msgs::ImageConstPtr &img);
      //! Subscriber2 : Process bounding boxes
      void processBox(const darknet_ros_msgs::BoundingBoxes r);
      //! Subscriber3 : Process depthCameraInfo
      void depthCameraInfo(const sensor_msgs::CameraInfoConstPtr &_info);
      //! Server123 : Signal of moving head
      bool processSignal1(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
      bool processSignal2(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
      bool processSignal3(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
      //! Server4 : Process object
      bool processObject(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
      //! funktion : move head
      void move_head(float data);
      //! funktion : move torso
      void move_torso(float data);
      //! funktion : get3DPoint
      cv::Vec3f get3DPoint(const cv::Mat &depthImage,int x, int y);

public:
      //-------------------- funktions --------------------/

      From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {
        camera_point.header.stamp = timer;
        camera_point.header.frame_id = "xtion_rgb_optical_frame";

        sub1 = nh_.subscribe("/xtion/depth_registered/image_raw", 10, &From2Dto3D::processCloud, this);
        sub2 = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &From2Dto3D::processBox, this);
        cameraInfoSub_= nh_.subscribe("/xtion/rgb/camera_info",10,&From2Dto3D::depthCameraInfo,this);
        srv1 = nh_.advertiseService("/desired_signal1",&From2Dto3D::processSignal1, this);
        srv2 = nh_.advertiseService("/desired_signal2",&From2Dto3D::processSignal2, this);
        srv3 = nh_.advertiseService("/desired_signal3",&From2Dto3D::processSignal3, this);
        srv4 = nh_.advertiseService("/desired_object",&From2Dto3D::processObject, this);
        pub1 = nh_.advertise<final_msg_srv::Coordinates_3D>("/point3D",10);
        pub2 = nh_.advertise<final_msg_srv::done>("point_done",10);
        client2 = nh_.serviceClient<final_msg_srv::DesiredObject>("/say_something");

        msg_done.done = 1;
        box_number = 0;
        depth = 1000;
        object = "cup";
        point_cv[0] = point_cv[1] = point_cv[2] = 0;

        ROS_INFO("from2Dto3D initialized ...");
      }

      ~From2Dto3D()
      {
      }
};


//-------------------- Subscriber1 : processCloud --------------------//

void From2Dto3D::processCloud(const sensor_msgs::ImageConstPtr &img)
{
  //  ROS_WARN_STREAM(__LINE__);
    timer = img->header.stamp;
    img_cv = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
  //  ROS_INFO("Process clusters");
    return;
}

//-------------------- Subscriber2 : process Boundingbox --------------------//

void From2Dto3D::processBox(const darknet_ros_msgs::BoundingBoxes r)
{
   // ROS_WARN_STREAM(__LINE__);
    box_number = r.bounding_boxes.size();
    for (int n=0;n<box_number;n++)
    {
        box[n].a = r.bounding_boxes[n].xmax;
        box[n].b = r.bounding_boxes[n].xmin;
        box[n].c = r.bounding_boxes[n].ymax;
        box[n].d = r.bounding_boxes[n].ymin;
        box[n].prabability = r.bounding_boxes[n].probability;
        box_name = r.bounding_boxes[n].Class;
        box[n].name = box_name;
        std::cout << box_name;
    }
    //ROS_INFO_STREAM("Processed boundingboxes:" << box_number);
    return;
}

//-------------------- Subscriber3 : process depthCameraInfo --------------------//

void From2Dto3D::depthCameraInfo(const sensor_msgs::CameraInfoConstPtr& _info)
{
    fx = _info->K.at(0);
    fy = _info->K.at(4);
    cx = _info->K.at(2);
    cy = _info->K.at(5);

    width  = _info->width;
    height = _info->height;
    fovx   = 2 * std::atan(width / (2 * fx));
    fovy   = 2 * std::atan(height / (2 * fy));

    ROS_INFO("Received camera info");
    gotInfo = true;
    cameraInfoSub_.shutdown();
}

//-------------------- Server1 : Signal of moving head and torso --------------------//

bool From2Dto3D::processSignal1(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    ROS_WARN_STREAM(__LINE__);
    From2Dto3D::move_torso(0.25);
    From2Dto3D::move_head(-0.17);
    return true;
}

//-------------------- Server2 : Signal of moving head and torso --------------------//

bool From2Dto3D::processSignal2(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    ROS_WARN_STREAM(__LINE__);
    From2Dto3D::move_torso(0.35);
    From2Dto3D::move_head(0.06);
    return true;
}

//-------------------- Server3 : Signal of moving head and torso --------------------//

bool From2Dto3D::processSignal3(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    ROS_WARN_STREAM(__LINE__);
    From2Dto3D::move_torso(0.15);
    From2Dto3D::move_head(0.0);
    return true;
}

//-------------------- Server4 : process desired Object --------------------//

bool From2Dto3D::processObject(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    int x=0;
    int y=0;
    bool xn=1;
    bool yn=1;
    bool zn=1;

    while (!gotInfo)
    {
        ros::Duration(1).sleep();
  //      ROS_WARN_STREAM(__LINE__);
    }

    for (int n=0;n<box_number;n++)
    {
        ROS_WARN_STREAM(__LINE__);
        if (object == box[n].name)
        {
            ROS_INFO("find the object!");
            x = (box[n].a + box[n].b)/2 ;
            y = (box[n].c + box[n].d)/2 ;
            ROS_WARN_STREAM("x:"<<x <<"  y:" << y);

            point_cv = From2Dto3D::get3DPoint(img_cv->image,x,y);

           // ROS_WARN_STREAM("pppx:"<<point_cv.val[0] <<"pppy:" << point_cv.val[1] << "pppz:" << point_cv.val[2]);

            if (point_cv.val[0] && point_cv.val[1] && point_cv.val[2])
            {
                camera_point.point.x = point_cv.val[0];
                camera_point.point.y = point_cv.val[1];
                camera_point.point.z = point_cv.val[2];
                listener.transformPoint("base_footprint",camera_point,base_point);

                if (depth > base_point.point.z)
                {
                    base_point_.x = base_point.point.x;
                    base_point_.y = base_point.point.y;
                    base_point_.z = base_point.point.z;

                    xn = isnan(base_point_.x);
                    yn = isnan(base_point_.y);
                    zn = isnan(base_point_.z);

                    depth = base_point.point.z;
                    ROS_WARN_STREAM("pppx:"<<base_point.point.x <<"pppy:" << base_point.point.y << "pppz:" << base_point.point.z);
                }
            }
        }
    }
    if (!xn && !yn && !zn)
    {
        // && base_point.point.x < 1.3 && base_point.point.x>0.5 && base_point.point.y < 0.5 && base_point.point.z < 1.3
        pub1.publish(base_point_);
        pub2.publish(msg_done);
        box_number = 0;
    }
    else
    {
        ROS_ERROR("the point is missing!");
    }
    depth = 1000;
    return true;
}

//-------------------- move head --------------------//

void From2Dto3D::move_head(float data)
{
    ROS_WARN_STREAM(__LINE__);
    // Create an arm controller action client to move the TIAGo's arm
    joint_control_client ArmClient("/head_controller/follow_joint_trajectory",true);
    while(!ArmClient.waitForServer(ros::Duration(5.0)))
        ROS_INFO("waiting for the move_base action server to come up");

    control_msgs::FollowJointTrajectoryGoal goal;
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("head_1_joint");
    goal.trajectory.joint_names.push_back("head_2_joint");
    // Two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);
    // Positions
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = 0;
    goal.trajectory.points[0].positions[1] = data;
    // Velocities
    goal.trajectory.points[0].velocities.resize(2);
    goal.trajectory.points[0].velocities[0] = 0.0;
    goal.trajectory.points[0].velocities[1] = 0.0;
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
    ArmClient.sendGoal(goal);
    ArmClient.waitForResult();
}

//-------------------- move torso --------------------//

void From2Dto3D::move_torso(float data)
{
    ROS_WARN_STREAM(__LINE__);
    // Create an arm controller action client to move the TIAGo's arm
    joint_control_client TorsoClient("/torso_controller/follow_joint_trajectory",true);
    while(!TorsoClient.waitForServer(ros::Duration(5.0)))
        ROS_INFO("waiting for the move_base action server to come up");

    control_msgs::FollowJointTrajectoryGoal goal;
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    // Two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);
    // Positions
    goal.trajectory.points[0].positions.resize(1);
    goal.trajectory.points[0].positions[0] = data;
    goal.trajectory.points[0].velocities.resize(1);
    goal.trajectory.points[0].velocities[0] = 0.0;
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
    TorsoClient.sendGoal(goal);
    TorsoClient.waitForResult();
}

//-------------------- get3Dpoint --------------------//

cv::Vec3f From2Dto3D::get3DPoint(const cv::Mat &depthImage,int x, int y)
{
    int x_off_low = 30, x_off_high = x_off_low + width - 60;
    int y_off_low = 30, y_off_high = y_off_low + height- 40;

    cv::Point p = cv::Point(x,y);

    if(!(p.x >= x_off_low && p.x < x_off_high &&
         p.y >= y_off_low && p.y < y_off_high))
    {
        ROS_WARN("Point must be inside the image (x=%d, y=%d), image range=(%d - %d, %d - %d)",
                 p.x, p.y,
                 x_off_low, x_off_high, y_off_low, y_off_high);
        return cv::Vec3f(
                    std::numeric_limits<float>::quiet_NaN (),
                    std::numeric_limits<float>::quiet_NaN (),
                    std::numeric_limits<float>::quiet_NaN ());
    }


    cv::Vec3f pt;

    // Use correct principal point from calibration
    float center_x = cx; //cameraInfo.K.at(2)
    float center_y = cy; //cameraInfo.K.at(5)

    bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    float unit_scaling = isInMM?0.001f:1.0f;
    float constant_x = unit_scaling / fx; //cameraInfo.K.at(0)
    float constant_y = unit_scaling / fy; //cameraInfo.K.at(4)

    float depth;
    bool isValid = false;

    if(isInMM)
    {
        depth = (float)depthImage.at<uint16_t>(y,x);
        isValid = depth != 0.0f;
    }
    else
    {
        depth = depthImage.at<float>(y,x);
        isValid = std::isfinite(depth);
    }

    if (!isValid)
    {
        pt.val[0] = pt.val[1] = pt.val[2] = 0;
    }
    else
    {
        // Fill in XYZ
        pt.val[0] = (float(x) - center_x) * depth * constant_x;
        pt.val[1] = (float(y) - center_y) * depth * constant_y;
        pt.val[2] = depth*unit_scaling;
    }

    return pt;
}


//-------------------- main --------------------//

int main(int argc, char** argv)
{
    ros::init(argc, argv, "from2dto3d");
    ros::NodeHandle nh;
    From2Dto3D node(nh);
    ROS_WARN_STREAM(__LINE__);
    ros::spin();
    return 0;
}

