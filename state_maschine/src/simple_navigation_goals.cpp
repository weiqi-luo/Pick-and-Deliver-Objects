#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <string>
#include <vector>
#include <map>
#include <tutorial5_msg_srv/desired_position.h>
#include <tutorial5_msg_srv/done.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class simple_navigation_goals
{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

    ros::ServiceServer srv;
    ros::ServiceServer srv_;
    ros::ServiceClient client1;
    ros::ServiceClient client2;
    ros::Publisher pub1;
    ros::Publisher pub2;
    ros::Publisher pub3;

    std_srvs::Empty srv1;
    std_srvs::Empty srv2;
    geometry_msgs::Twist msg1;
    tutorial5_msg_srv::done msg2;
    geometry_msgs::Twist msg3;

    move_base_msgs::MoveBaseGoal goal;

    int number;


    //----- Functions -----//

    simple_navigation_goals(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
        srv = nh_.advertiseService("move_base", &simple_navigation_goals::movebase, this);
        srv_ = nh_.advertiseService("goto_table", &simple_navigation_goals::gototable, this);
        client1 = nh_.serviceClient<std_srvs::Empty>("/global_localization");
        client2 = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        pub1 = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);
        pub2 = nh_.advertise<tutorial5_msg_srv::done>("process_done",100);
        pub3 = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",100);

        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";

        number = 0;

    }

    ~simple_navigation_goals() {}

    void localization();

    bool movebase(tutorial5_msg_srv::desired_positionRequest &req, tutorial5_msg_srv::desired_positionResponse &res);

    bool gototable(std_srvs::EmptyResponse &req, std_srvs::EmptyResponse &res);

};




    //----- localization -----//

void simple_navigation_goals::localization()
{
    ros::NodeHandle np;
    ros::Rate rp(60);
    //ros::Duration(12).sleep();

    if (client1.call(srv1))
        ROS_INFO("starting localization");
    else
        ROS_ERROR("localization failed");

    int count=0;
    while (count <= 400 && ros::ok())
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
    number = number+1;
    return;
}




//----- callback movebase -----//

bool simple_navigation_goals::movebase(tutorial5_msg_srv::desired_positionRequest &req, tutorial5_msg_srv::desired_positionResponse &res)
{
    if (!number)
    simple_navigation_goals::localization();

goal.target_pose.pose.position.x = req.position_x;
goal.target_pose.pose.position.y = req.position_y;
goal.target_pose.pose.orientation.z = req.orientation_z;
goal.target_pose.pose.orientation.w = req.orientation_w;

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

while(!ac.waitForServer(ros::Duration(5.0)))
    ROS_INFO("waiting for the move_base action server to come up");

ac.sendGoal(goal);
ac.waitForResult();

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

bool simple_navigation_goals::gototable(std_srvs::EmptyResponse &req, std_srvs::EmptyResponse &res)
{
    ros::NodeHandle np;
    ros::Rate rp(60);
    int count = 0;
    while(count <= 420)
    {
        count++;
        msg3.angular.z=0.5;
        pub3.publish(msg3);
        ros::spinOnce();
        rp.sleep();
    }
    count = 0;
    while(count <= 58)
    {
        count++;
        msg3.angular.z = 0.0;
        msg3.linear.x=0.5;
        pub3.publish(msg3);
        ros::spinOnce();
        rp.sleep();
    }
}




    //----- main -----//
int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle nh;
    simple_navigation_goals node(nh);
    ros::spin();
    return 0;
}

