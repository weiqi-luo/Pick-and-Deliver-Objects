#include <ros/ros.h>
#include <ros/console.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <string>
#include <vector>
#include <map>
#include <final_msg_srv/desired_position.h>
#include <final_msg_srv/done.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
//Eigen
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <std_srvs/Empty.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class navigation_server
{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    ros::ServiceServer srv;
    ros::ServiceServer srv_1;

    ros::ServiceClient client1;
    ros::ServiceClient client2;
    ros::Publisher pub1;
    ros::Publisher pub2;
    ros::Publisher pub3;
    ros::Subscriber sub;

    std_srvs::Empty srv1;
    std_srvs::Empty srv2;
    geometry_msgs::Twist msg1;
    final_msg_srv::done msg2;
    geometry_msgs::Twist msg3;

    move_base_msgs::MoveBaseGoal goal;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;

    int number;


    //----- Functions -----//

    navigation_server(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
        srv = nh_.advertiseService("move_base", &navigation_server::movebase, this);
        srv_1 = nh_.advertiseService("do_localization", &navigation_server::localization, this);

        client1 = nh_.serviceClient<std_srvs::Empty>("/global_localization");
        client2 = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        pub1 = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
        pub2 = nh_.advertise<final_msg_srv::done>("process_done",100);
        pub3 = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
        sub = nh_.subscribe("/xtion/depth_registered/points",10, &navigation_server::processC, this);

        goal.target_pose.header.stamp = ros::Time(cloud_.header.stamp);
        //goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";

        number = 0;

    }

    ~navigation_server() {}

    bool localization(std_srvs::EmptyResponse &req, std_srvs::EmptyResponse &res);

    bool movebase(final_msg_srv::desired_positionRequest &req, final_msg_srv::desired_positionResponse &res);

    //bool gototable(std_srvs::EmptyResponse &req, std_srvs::EmptyResponse &res);

    void processC(const sensor_msgs::PointCloud2::ConstPtr &pc);

};




    //----- localization -----//

bool navigation_server::localization(std_srvs::EmptyResponse &req, std_srvs::EmptyResponse &res)
{
    ros::NodeHandle np;
    ros::Rate rp(60);
    //ros::Duration(12).sleep();

    if (client1.call(srv1))
        ROS_INFO("starting localization");
    else
        ROS_ERROR("localization failed");

    int count=0;
    while (count <= 1518 && ros::ok()) //was 2000 before
    {
        count++;
        msg1.angular.z = 0.5;
        pub1.publish(msg1);
        ros::spinOnce();
        rp.sleep();
    }

    if (client2.call(srv2))
        ROS_INFO("costmap is cleared");
    else
        ROS_ERROR("failed to clear the costmap");

    return true;
}




//----- callback movebase -----//

bool navigation_server::movebase(final_msg_srv::desired_positionRequest &req, final_msg_srv::desired_positionResponse &res)
{
    ROS_WARN_STREAM(__LINE__);


goal.target_pose.pose.position.x = req.position_x;
goal.target_pose.pose.position.y = req.position_y;
goal.target_pose.pose.orientation.z = req.orientation_z;
goal.target_pose.pose.orientation.w = req.orientation_w;

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

while(!ac.waitForServer(ros::Duration(5.0)))
    ROS_INFO("waiting for the move_base action server to come up");

ROS_WARN_STREAM(__LINE__);

ac.sendGoal(goal);
ac.waitForResult();

    ROS_WARN_STREAM(__LINE__);

if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
{
    ROS_INFO("goal is achieved");
    msg2.done = true;
}
else
{
    ROS_ERROR("failed to achieve the goal");
    msg2.done = false;
}
pub2.publish(msg2);

res.reply = 1;
return true;
}


//----- callback gototable -----//



//-------------------- Subscriber : processCloud --------------------//

void navigation_server::processC(const sensor_msgs::PointCloud2::ConstPtr &pc)
{

    pcl::fromROSMsg(*pc, cloud_);
    return;
}



    //----- main -----//
int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_server");
    ros::NodeHandle nh;
    navigation_server node(nh);
    ros::spin();
    return 0;
}


