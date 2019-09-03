
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


// yolo specific includes
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Int8.h>


//#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <string.h>

#include <final_msg_srv/DesiredObject.h>
#include <final_msg_srv/Coordinates_3D.h>
#include <final_msg_srv/done.h>


// tf specific includes
#include <tf/transform_listener.h>


// actionclient specific includes
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/shared_ptr.hpp>
#include <ros/topic.h>
#include <std_srvs/Empty.h>

// speak specific includes
#include <final_msg_srv/DesiredObject.h>

// gesture detection specific includes
#include <image_recognition_msgs/Recognize.h>


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> joint_control_client;
//typedef actionlib::SimpleActionClient<pal_interaction_msgs::TtsAction> speak_client;

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
      ros::Subscriber sub3;
      ros::Subscriber sub4;
      ros::ServiceServer srv1;
      ros::ServiceServer srv2;
      ros::ServiceClient client1;
      ros::ServiceClient client2;

      //! Define the pointcloud structure and the bounding box local copy
      pcl::PointCloud<pcl::PointXYZRGB> cloud_;
      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt;
      pcl::PointXYZRGB cloud_p;
      geometry_msgs::PointStamped camera_point;
      geometry_msgs::PointStamped base_point;
      final_msg_srv::Coordinates_3D base_point_;

      geometry_msgs::PointStamped right_hand;
      geometry_msgs::PointStamped left_hand;
      geometry_msgs::PointStamped right_hand_old;
      geometry_msgs::PointStamped left_hand_old;
      geometry_msgs::PointStamped right_hand_new;
      geometry_msgs::PointStamped left_hand_new;

      final_msg_srv::DesiredObject msg_something;
      final_msg_srv::done msg_done;
      image_recognition_msgs::Recognize msg_image;

      bool done;
      int count;
      double delta_right;
      double delta_left;

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
      std::string OPENCV_WINDOW;
      int box_number;

      tf::TransformListener listener;


      //-------------------- callbacks --------------------/

      //! Subscriber1 : Process clusters
      void processCloud(const sensor_msgs::PointCloud2ConstPtr &pc);
      //! Subscriber2 : Process bounding boxes
      void processRect(const darknet_ros_msgs::BoundingBoxes r);
      //! Subscriber3 : Process image_before
      void processImage_before(const sensor_msgs::Image ig);
      //! Subscriber3 : Process image_after
      void processImage_after(const sensor_msgs::ImagePtr &ig);
      //! Server1 : Signal of moving head
      bool processSignal(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
      //! Server2 : Process object
      bool processObject(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
      //! funktion : move head
      void move_head();
      //! funktion : move torso
      void move_torso();
      //! funktion : detect gesture
      void detect_gesture();
      //! funktion : bridge connected with openpose
      void bridge_openpose();
      //! funktion : bridge connected with opencv
      void bridge_opencv();

public:
      //-------------------- funktions --------------------/

      From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {
        camera_point.header.stamp = ros::Time(cloud_.header.stamp);
        camera_point.header.frame_id = "xtion_rgb_optical_frame";

        right_hand.header.stamp =  ros::Time(cloud_.header.stamp);
        left_hand.header.stamp = ros::Time(cloud_.header.stamp);
        right_hand.header.frame_id = "right_hand";
        left_hand.header.frame_id = "left_hand";

        sub1 = nh_.subscribe("/xtion/depth_registered/points", 10, &From2Dto3D::processCloud, this);
        sub2 = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &From2Dto3D::processRect, this);
        sub3 = nh_.subscribe("/xtion/rgb/image_rect_color", 10, &From2Dto3D::processImage_before, this);
        sub4 = nh_.subscribe("/result_image", 10, &From2Dto3D::processImage_after, this);
        srv1 = nh_.advertiseService("desired_signal",&From2Dto3D::processSignal, this);
        srv2 = nh_.advertiseService("desired_object",&From2Dto3D::processObject, this);
        pub1 = nh_.advertise<final_msg_srv::done>("process_done",10);
        pub2 = nh_.advertise<final_msg_srv::Coordinates_3D>("point3D",10);
        client1 = nh_.serviceClient<final_msg_srv::DesiredObject>("say_something");
        client2 = nh_.serviceClient<image_recognition_msgs::Recognize>("recognize");

        box_number = 0;
        done = 0;
        msg_done.done = 1;
        count = 0;

        object = "bottle";
        OPENCV_WINDOW = "Image window";

        right_hand.point.x = 0;
        right_hand.point.y = 0;
        right_hand.point.z = 0;

        left_hand.point.x = 0;
        left_hand.point.y = 0;
        left_hand.point.z = 0;

        cv::namedWindow(OPENCV_WINDOW);

        ROS_INFO("from2Dto3D initialized ...");
      }

      ~From2Dto3D()
      {
          cv::destroyWindow(OPENCV_WINDOW);
      }
};


//-------------------- Subscriber1 : processCloud --------------------//

void From2Dto3D::processCloud(const sensor_msgs::PointCloud2ConstPtr &pc)
{
    ROS_WARN_STREAM(__LINE__);
    pcl::fromROSMsg(*pc, cloud_);
    ROS_INFO("Processed clusters");

    if (done)
    {
        try
        {
            From2Dto3D:: detect_gesture();
            if (done)
            {
                ROS_WARN("please give me the book.");
                msg_something.request.x = "please give me the book.";
            }
            client1.call(msg_something);
        }
        catch (...)
        {
            ROS_WARN("please let me see you.");
            msg_something.request.x = "please let me see you.";
            client1.call(msg_something);
        }
    }

    return;
}

//-------------------- Subscriber2 : process Boundingbox --------------------//

void From2Dto3D::processRect(const darknet_ros_msgs::BoundingBoxes r)
{
    //ROS_WARN_STREAM(__LINE__);
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
    //ROS_INFO_STREAM("Processed boundingboxes:" << box_number);
    return;
}

//-------------------- Subscriber3 : process image_before --------------------//

void From2Dto3D::processImage_before(const sensor_msgs::Image ig)
{
    ROS_WARN_STREAM(__LINE__);
    ROS_INFO("Processed images");
    msg_image.request.image = ig;
    client2.call(msg_image);
}

//-------------------- Subscriber4 : process image_before --------------------//

void From2Dto3D::processImage_after(const sensor_msgs::ImagePtr &ig)
{
    ROS_WARN_STREAM(__LINE__);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(ig,sensor_msgs::image_encodings::RGB8);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow(OPENCV_WINDOW,cv_ptr->image);
    cv::waitKey(3);
}

//-------------------- Server1 : Signal of moving head --------------------//

bool From2Dto3D::processSignal(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    ROS_WARN_STREAM(__LINE__);
    From2Dto3D::move_head();
    From2Dto3D::move_torso();
    ros::Duration(2).sleep();
    done = 1;
}

//-------------------- Server2 : process desired Object --------------------//

bool From2Dto3D::processObject(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    int x=0;
    int y=0;
    bool xn=1;
    bool yn=1;
    bool zn=1;
    int width_= 0;

    while (!cloud_.isOrganized())
    {
        ros::Duration(1).sleep();
    }


    for (int n=0;n<box_number;n++)
    {
        ROS_WARN_STREAM(__LINE__);
        if (object == box[n].name)
        {
            int width = box[n].b;
            if (width > width_)
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
    }
    else
    {
        ROS_ERROR("the point is missing!");
    }
    return true;
}

//-------------------- move head --------------------//

void From2Dto3D::move_head()
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
    goal.trajectory.points[0].positions[1] = 0.14;
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

void From2Dto3D::move_torso()
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
    goal.trajectory.points[0].positions[0] = 0.2;
    goal.trajectory.points[0].velocities.resize(1);
    goal.trajectory.points[0].velocities[0] = 0.0;
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
    //goal.trajectory.header.stamp = ros::Time(cloud_.header.stamp);
    TorsoClient.sendGoal(goal);
    TorsoClient.waitForResult();
}

//-------------------- detect gesture --------------------//

void From2Dto3D::detect_gesture()
{
    ROS_WARN_STREAM(__LINE__);
    ROS_ERROR_STREAM(count);

    if (count)
    {
        right_hand_old = right_hand_new;
        left_hand_old = left_hand_new;
    }

    listener.transformPoint("base_footprint",right_hand,right_hand_new);
    listener.transformPoint("base_footprint",left_hand,left_hand_new);

    if (count)
    {
        ROS_INFO_STREAM("delta_right:" << delta_right << "delta_left" << delta_left);
        delta_right = right_hand_new.point.z - right_hand_old.point.z;
        delta_left = left_hand_new.point.z - left_hand_old.point.z;
        if ( delta_right>0.15 || delta_left>0.15 )
        {
            done = 0;
            ROS_WARN("please don't move.");
            msg_something.request.x = "please don't move.";
            client1.call(msg_something);
            pub1.publish(msg_done);
        }
    }
    count = count+1;
}

//-------------------- opencv bridge --------------------//

void From2Dto3D::bridge_opencv()
{

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

