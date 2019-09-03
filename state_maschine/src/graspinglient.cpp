
/*********************************************************************
* STD INCLUDES
********************************************************************/
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <state_maschine/state_mashine.h>
#include <tutorial5_msg_srv/tiago_pick.h>


/*********************************************************************
* ROS INCLUDES
********************************************************************/





int main(int argc, char** argv)
{

    ros::init(argc, argv, "graspinglient");

    ros::NodeHandle n;
    ros::Rate r(60);

    //INITIALIZE THE CLIENT
    ros::ServiceClient cli_arma = n.serviceClient<tutorial5_msg_srv::desired3D_pos>("move_arm");




    //INITIALIZE THE SUBSCRIBER
    Tutorial_5::state_mashine SM;
    ros::Subscriber sub_3d_main = n.subscribe("point3D_from_main",10, &Tutorial_5::state_mashine::processCoord3D, &SM);
    ros::Subscriber sub_done_grasp = n.subscribe("pick_process_ready",10,&Tutorial_5::state_mashine::processPickdone,&SM);

    // MSG VARIABLE FOR THE SERVICE MESSAGE
    tutorial5_msg_srv::desired3D_pos desired_pos3d;
    //String variable for desired object
    //Float variable for navigation goals
    //Shelf

    float table_x=0;
    float table_y=0;
    float table_z=0.8;

    float desired_x;
    float desired_y;
    float desired_z;
    int flag = 0;



    ROS_WARN_STREAM(__LINE__);
    //Float coordinates for a point on table

    while(ros::ok())
    {
        if(flag==0)
        {
            desired_x=SM.msg3D.x;
            desired_y=SM.msg3D.y;
            desired_z=SM.msg3D.z;
            std::cout<<desired_x<<std::endl;
            std::cout<<desired_y<<std::endl;
            std::cout<<desired_y<<std::endl;
        }
        std::cout<<"flag = "<<flag<<std::endl;
        if(flag==1)
        {
            desired_x=table_x;
            desired_y=table_y;
            desired_z=table_z;
            std::cout<<desired_x<<std::endl;
            std::cout<<desired_y<<std::endl;
            std::cout<<desired_z<<std::endl;

        }

        if(SM.msgPickdone.done_pick==true)
        {
            ROS_WARN_STREAM(__LINE__);

            ROS_INFO("got_pose,sending service.....");

            desired_pos3d.request.x=desired_x;
            desired_pos3d.request.y=desired_y;
            desired_pos3d.request.z=desired_z;
            cli_arma.call(desired_pos3d);
            cli_arma.shutdown();
            if(flag==0)
            {flag=1;}

            else
            {flag=0;}
            SM.msgPickdone.done_pick=false;

        }


        ros::spinOnce();
        r.sleep();
    }


return 0;

  }






