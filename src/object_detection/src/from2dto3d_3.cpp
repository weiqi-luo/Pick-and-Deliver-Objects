
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
      ros::Subscriber cameraInfoSub;
      ros::ServiceServer srv1;
      ros::ServiceServer srv2;
      ros::ServiceServer srv3;
      ros::ServiceServer srv4;
      ros::ServiceClient client2;

      pcl::PointCloud<pcl::PointXYZRGB> cloud_;
      pcl::PointXYZRGB cloud_p;
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


      //-------------------- callbacks --------------------/

      //! Subscriber1 : Process clusters
      void processCloud(const sensor_msgs::PointCloud2ConstPtr &pc);
      //! Subscriber2 : Process bounding boxes
      void processBox(const darknet_ros_msgs::BoundingBoxes r);
      //! Subscriber3 : Process image_before
      void processImage_before(const sensor_msgs::Image ig);
      //! Subscriber3 : Process image_after
      void processImage_after(const sensor_msgs::ImagePtr &ig);

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


public:
      //-------------------- funktions --------------------/

      From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {
        camera_point.header.stamp = ros::Time(cloud_.header.stamp);
        camera_point.header.frame_id = "xtion_rgb_optical_frame";

        sub1 = nh_.subscribe("/xtion/depth_registered/points", 10, &From2Dto3D::processCloud, this);
        sub2 = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &From2Dto3D::processBox, this);
        srv1 = nh_.advertiseService("/desired_signal1",&From2Dto3D::processSignal1, this);
        srv2 = nh_.advertiseService("/desired_signal2",&From2Dto3D::processSignal2, this);
        srv3 = nh_.advertiseService("/desired_signal3",&From2Dto3D::processSignal3, this);
        srv4 = nh_.advertiseService("/desired_object",&From2Dto3D::processObject, this);
        pub1 = nh_.advertise<final_msg_srv::Coordinates_3D>("/point3D",10);
        pub2 = nh_.advertise<final_msg_srv::done>("process_done",10);
        client2 = nh_.serviceClient<final_msg_srv::DesiredObject>("/say_something");

        msg_done.done = 1;
        box_number = 0;
        depth = 1000;
        object = "cup";


       // OPENCV_WINDOW = "Image window";
       // cv::namedWindow(OPENCV_WINDOW);

        ROS_INFO("from2Dto3D initialized ...");
      }

      ~From2Dto3D()
      {
       //   cv::destroyWindow(OPENCV_WINDOW);
      }
};


//-------------------- Subscriber1 : processCloud --------------------//

void From2Dto3D::processCloud(const sensor_msgs::PointCloud2ConstPtr &pc)
{
    ROS_WARN_STREAM(__LINE__);
    pcl::fromROSMsg(*pc, cloud_);
    ROS_INFO("Process clusters");
    return;
}

//-------------------- Subscriber2 : process Boundingbox --------------------//

void From2Dto3D::processBox(const darknet_ros_msgs::BoundingBoxes r)
{
    ROS_WARN_STREAM(__LINE__);
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

void From2Dto3D::depthCameraInfo(const sensor_msgs::CameraInfoConstPtr &_info)
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
    From2Dto3D::move_torso(0.2);
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

    while (!cloud_.isOrganized())
    {
        ros::Duration(1).sleep();
        ROS_WARN_STREAM(__LINE__);
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

            cloud_p = cloud_.at(x,y);
            camera_point.point.x = cloud_p.x;
            camera_point.point.y = cloud_p.y;
            camera_point.point.z = cloud_p.z;
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

                ROS_INFO_STREAM("object="<< object <<"\n xn="<< xn <<"\n yn="<< yn <<"\n z="<< zn );
                ROS_INFO_STREAM("object="<< object <<"\n x="<< camera_point.point.x <<"\n y="<< camera_point.point.y <<"\n z="<< camera_point.point.z );
                ROS_INFO_STREAM("object="<< object <<"\n x="<< base_point.point.x <<"\n y="<< base_point.point.y <<"\n z="<< base_point.point.z );
            }
        }
    }
    if (!xn && !yn && !zn && base_point.point.x < 1.0 && base_point.point.x>0.5 && base_point.point.y < 0.5 && base_point.point.z < 1.1)
    {
        pub1.publish(base_point_);
        pub2.publish(msg_done);
        depth = 1000;
    }


    else
    {
        ROS_ERROR("the point is missing!");
    }
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
    //goal.trajectory.header.stamp = ros::Time(cloud_.header.stamp);
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
    //goal.trajectory.header.stamp = ros::Time(cloud_.header.stamp);
    TorsoClient.sendGoal(goal);
    TorsoClient.waitForResult();
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

