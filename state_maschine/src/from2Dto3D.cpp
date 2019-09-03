
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Char.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
//#include <perception_msgs/Rect.h>


// yolo specific includes
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Int8.h>


//#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <string.h>

#include <tutorial5_msg_srv/DesiredObject.h>
#include <tutorial5_msg_srv/Coordinates_3D.h>
#include <tutorial5_msg_srv/done.h>


// tf specific includes
#include <tf/transform_listener.h>


// move head specific includes
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/shared_ptr.hpp>
#include <ros/topic.h>


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

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
      ros::ServiceServer srv1;
      ros::ServiceServer srv2;

      //! Define the pointcloud structure and the bounding box local copy
      pcl::PointCloud<pcl::PointXYZRGB> cloud_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt;
      pcl::PointXYZRGB cloud_p;
      geometry_msgs::PointStamped camera_point;
      geometry_msgs::PointStamped base_point;
      tutorial5_msg_srv::Coordinates_3D base_point_;


      tutorial5_msg_srv::done done;

      std::string object;

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
      int box_number;


      tf::TransformListener listener;


      //-------------------- callbacks --------------------/

      //! Subscriber : Process clusters
      void processCloud(const sensor_msgs::PointCloud2ConstPtr &pc);
      //! Subscriber : Process the number of bounding boxes
      void processNumber(const std_msgs::Int8ConstPtr &r);
      //! Subscriber : Process bounding boxes
      void processRect(const darknet_ros_msgs::BoundingBoxes r);
      //! Server : Signal of moving head
      bool processSignal(tutorial5_msg_srv::DesiredObjectRequest &req, tutorial5_msg_srv::DesiredObjectResponse &res);
      //! Server : Process object
      bool processObject(tutorial5_msg_srv::DesiredObjectRequest &req, tutorial5_msg_srv::DesiredObjectResponse &res);
      //! funktion : move head
      void move_head();
      //! funktion : move torso
      void move_torso();

public:
      //-------------------- funktions --------------------/

      From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {
        camera_point.header.stamp = ros::Time::now();
        camera_point.header.frame_id = "xtion_rgb_optical_frame";

        sub1 = nh_.subscribe("/xtion/depth_registered/points", 10, &From2Dto3D::processCloud, this);
        sub2 = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &From2Dto3D::processRect, this);
        srv1 = nh_.advertiseService("desired_signal",&From2Dto3D::processSignal, this);
        srv2 = nh_.advertiseService("desired_object",&From2Dto3D::processObject, this);
        pub1 = nh_.advertise<tutorial5_msg_srv::done>("process_done",10);
        pub2 = nh_.advertise<tutorial5_msg_srv::Coordinates_3D>("point3D",10);


        box_number = 0;

        ROS_INFO("from2Dto3D initialized ...");
      }

      ~From2Dto3D() {}
};


//-------------------- Subscriber : processCloud --------------------//

void From2Dto3D::processCloud(const sensor_msgs::PointCloud2ConstPtr &pc)
{
    ROS_WARN_STREAM(__LINE__);
    pcl::fromROSMsg(*pc, cloud_);
    ROS_INFO("Processed clusters");
    return;
}



//-------------------- Subscriber : process Boundingbox --------------------//

void From2Dto3D::processRect(const darknet_ros_msgs::BoundingBoxes r)
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
    }
    ROS_INFO_STREAM("Processed boundingboxes:" << box_number);
    return;
}




//-------------------- Server : Signal of moving head --------------------//

bool From2Dto3D::processSignal(tutorial5_msg_srv::DesiredObjectRequest &req, tutorial5_msg_srv::DesiredObjectResponse &res)
{
    ROS_WARN_STREAM(__LINE__);
    From2Dto3D::move_head(); 
    From2Dto3D::move_torso();
    res.reply =1;
    done.done = 1;
    ros::Duration(2).sleep();
    pub1.publish(done);
}



//-------------------- Server : process desired Object --------------------//

bool From2Dto3D::processObject(tutorial5_msg_srv::DesiredObjectRequest &req, tutorial5_msg_srv::DesiredObjectResponse &res)
{
    object = req.x;
    int x=0;
    int y=0;
    bool xn=1;
    bool yn=1;
    bool zn=1;
    int width_= 10000;
    int height_ = 10000;

    while (!cloud_.isOrganized())
    {
        ros::Duration(1).sleep();
    }

    ROS_INFO_STREAM("The desired object is: " << object);

    for (int n=0;n<box_number;n++)
    {
        ROS_WARN_STREAM(__LINE__);
        if (object == box[n].name)
        {
            int width = box[n].b;
            int height = box[n].d;
            if (width < width_ && height < height_)
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

                base_point_.x = base_point.point.x;
                base_point_.y = base_point.point.y;
                base_point_.z = base_point.point.z;

                xn = isnan(base_point_.x);
                yn = isnan(base_point_.y);
                zn = isnan(base_point_.z);

                width_ = width;
                height_ = height;

                ROS_INFO_STREAM("object="<< object <<"\n xn="<< xn <<"\n yn="<< yn <<"\n z="<< zn );
                ROS_INFO_STREAM("object="<< object <<"\n x="<< camera_point.point.x <<"\n y="<< camera_point.point.y <<"\n z="<< camera_point.point.z );
                ROS_INFO_STREAM("object="<< object <<"\n x="<< base_point.point.x <<"\n y="<< base_point.point.y <<"\n z="<< base_point.point.z );
            }
        }
    }
    if (!xn && !yn && !zn)
    {
        pub2.publish(base_point_);
        width_=0;
        res.reply =1;
    }
    else
    {
        res.reply =0;
        ROS_ERROR("the point is missing!");
    }
    return true;
}




//-------------------- move head --------------------//

void From2Dto3D::move_head()
{
    ROS_WARN_STREAM(__LINE__);
    // Create an arm controller action client to move the TIAGo's arm
    arm_control_client ArmClient("/head_controller/follow_joint_trajectory",true);
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
    goal.trajectory.points[0].positions[1] = -0.4;
    // Velocities
    goal.trajectory.points[0].velocities.resize(2);
    goal.trajectory.points[0].velocities[0] = 0.0;
    goal.trajectory.points[0].velocities[1] = 0.0;
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
    goal.trajectory.header.stamp = ros::Time::now();
    ArmClient.sendGoal(goal);
    ArmClient.waitForResult();
}




//-------------------- move torso --------------------//

void From2Dto3D::move_torso()
{
    ROS_WARN_STREAM(__LINE__);
    // Create an arm controller action client to move the TIAGo's arm
    arm_control_client ArmClient("/torso_controller/follow_joint_trajectory",true);
    while(!ArmClient.waitForServer(ros::Duration(5.0)))
    ROS_INFO("waiting for the move_base action server to come up");

    control_msgs::FollowJointTrajectoryGoal goal;
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    // Two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);
    // Positions
    goal.trajectory.points[0].positions.resize(1);
    goal.trajectory.points[0].positions[0] = 0.3;
    goal.trajectory.points[0].velocities.resize(1);
    goal.trajectory.points[0].velocities[0] = 0.0;
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
    goal.trajectory.header.stamp = ros::Time::now();
    ArmClient.sendGoal(goal);
    ArmClient.waitForResult();
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
