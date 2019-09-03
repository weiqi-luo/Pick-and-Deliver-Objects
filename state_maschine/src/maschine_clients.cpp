
/*********************************************************************
* STD INCLUDES
********************************************************************/
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <state_maschine/state_mashine.h>
#include <tutorial5_msg_srv/tiago_pick.h>
#include <std_srvs/Empty.h>


/*********************************************************************
* ROS INCLUDES
********************************************************************/





int main(int argc, char** argv)
{

    ros::init(argc, argv, "maschine_clients");

    ROS_INFO_STREAM("Clients");

    ros::NodeHandle n;
    ros::Rate r(60);

    //INITIALIZE THE CLIENT
    ros::ServiceClient cli_obj = n.serviceClient<tutorial5_msg_srv::DesiredObject>("desired_object");
    ros::ServiceClient cli_hea = n.serviceClient<tutorial5_msg_srv::DesiredObject>("desired_signal");
    ros::ServiceClient cli_arm = n.serviceClient<tutorial5_msg_srv::desired3D_pos>("move_arm");
    ros::ServiceClient cli_nav = n.serviceClient<tutorial5_msg_srv::desired_position>("move_base");
    ros::ServiceClient cli_pre = n.serviceClient<std_srvs::Empty>("pick_gui");
    ros::ServiceClient cli_place = n.serviceClient<std_srvs::Empty>("place_gui");
    ros::ServiceClient cli_move = n.serviceClient<std_srvs::Empty>("goto_table");

    //Publisher
    ros::Publisher pub_3d = n.advertise<tutorial5_msg_srv::Coordinates_3D>("point3D_from_main",10);


    //INITIALIZE THE SUBSCRIBER
    Tutorial_5::state_mashine SM;
    ros::Subscriber sub_3d = n.subscribe("point3D",10, &Tutorial_5::state_mashine::processCoord3D, &SM);
    ros::Subscriber sub_pose = n.subscribe("current_pose",10, &Tutorial_5::state_mashine::processCurrentPos, &SM);
    ros::Subscriber sub_arm = n.subscribe("process_done",10, &Tutorial_5::state_mashine::processDone, &SM);

    // MSG VARIABLE FOR THE SERVICE MESSAGE
    tutorial5_msg_srv::DesiredObject desired_obj;
    tutorial5_msg_srv::desired3D_pos desired_pos3d;
    tutorial5_msg_srv::desired_position desired_pos;
    std_srvs::Empty prepare_grasp;
    tutorial5_msg_srv::Coordinates_3D coord_3d_main;
    std_srvs::Empty tiago_move;



    //String variable for desired object
    std::string obj_in;



    //Float variable for navigation goals
    //Shelf
    float position_x,position_y,position_z;
    float orientation_w,orientation_x,orientation_y,orientation_z;
    position_x = 2.45;
    position_y = 0.013195;
    position_z = 0;
    orientation_x = 0;
    orientation_y = 0;
    orientation_z = 0.69;
    orientation_w = 0.724;
    //Table
    float position_x1,position_y1,position_z1;
    float orientation_w1,orientation_x1,orientation_y1,orientation_z1;

    position_x1 = 1.55465734005;
    position_y1 = -1.16673576832;
    position_z1 = 0.0;
    orientation_x1 = 0.0;
    orientation_y1 = 0.0;
    orientation_z1 = -0.214431007662;
    orientation_w1 = 0.976739137617;

    //StartPoint
    float position_x0,position_y0,position_z0;
    float orientation_w0,orientation_x0,orientation_y0,orientation_z0;
    position_x0 = 0;
    position_y0 = 0;
    position_z0 = 0;
    orientation_x0 = 0;
    orientation_y0 = 0;
    orientation_z0 = 0;
    orientation_w0 = 0.999998045024;

    //3d Point of object
    float desired_x;
    float desired_y;
    float desired_z;

    //done or not
    bool done_process;



    //Float coordinates for a point on table
    float table_x,table_y,table_z;


    //define state space
    enum States {start_point,before_shelf,before_shelf_head_down,before_shelf_with_perception,
                 before_shelf_with_object,before_table_with_object,before_table_without_object};


    States activestate;
    activestate = start_point;


    while(ros::ok())
    {

        switch (activestate)
        {
            case start_point:
            {

                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                //ROS_INFO_STREAM("current state:"<<activestate);
                std::cin>>obj_in;
                desired_pos.request.position_x=position_x;
                desired_pos.request.position_y=position_y;
                desired_pos.request.position_z=position_z;
                desired_pos.request.orientation_w=orientation_w;
                desired_pos.request.orientation_x=orientation_x;
                desired_pos.request.orientation_y=orientation_y;
                desired_pos.request.orientation_z=orientation_z;

                ROS_INFO("starting the localization and navigation");
                cli_nav.call(desired_pos);

                ros::spinOnce();
                std::cout<<SM.msgDone.done<<std::endl;
                if (SM.msgDone.done==true)
                {
                       SM.msgDone.done=false;
                       activestate=before_shelf;
                }
                break;
            }
            case before_shelf:
            {
                ros::spinOnce();
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                desired_obj.request.x=obj_in;
                ROS_INFO("lowering head...");
                cli_hea.call(desired_obj);
                ROS_INFO_STREAM(SM.msgDone.done);
                done_process=SM.msgDone.done;
                std::cout<<done_process<<std::endl;
                if (done_process==true)
                {
                       SM.msgDone.done=false;
                       activestate=before_shelf_head_down;
                }

                break;
            }

            case before_shelf_head_down:
            {
                ros::spinOnce();
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                ROS_INFO("perception...");
                desired_obj.request.x=obj_in;
                cli_obj.call(desired_obj);
                if(SM.msg3D.x!=0 && SM.msg3D.y!=0 && SM.msg3D.z!=0)
                    {
                         desired_x=SM.msg3D.x;
                         desired_y=SM.msg3D.y;
                         desired_z=SM.msg3D.z;
                         std::cout<<desired_x<<std::endl;
                         std::cout<<desired_y<<std::endl;
                         std::cout<<desired_z<<std::endl;
                         coord_3d_main.x=desired_x;
                         coord_3d_main.y=desired_y;
                         coord_3d_main.z=desired_z;
                         pub_3d.publish(coord_3d_main);
                         SM.msgDone.done=false;
                         activestate=before_shelf_with_perception;

                    }


                break;
            }

            case before_shelf_with_perception:
            {
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                //ros::spinOnce();

                cli_pre.call(prepare_grasp);
                activestate=before_shelf_with_object;
                break;
            }



            case before_shelf_with_object:
            {
                ros::spinOnce();
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;

                ROS_INFO("Starting navigation");
                cli_move.call(tiago_move);
                activestate=before_table_with_object;

                break;
            }

            case before_table_with_object:
            {
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                cli_place.call(prepare_grasp);
                activestate=before_table_without_object;
                break;
            }
            case before_table_without_object:
            {
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                desired_pos.request.position_x=position_x0;
                desired_pos.request.position_y=position_y0;
                desired_pos.request.position_z=position_z0;
                desired_pos.request.orientation_w=orientation_w0;
                desired_pos.request.orientation_x=orientation_x0;
                desired_pos.request.orientation_y=orientation_y0;
                desired_pos.request.orientation_z=orientation_z0;
                if(cli_nav.call(desired_pos))
                {
                    ROS_INFO("Navigating to StartPoint");
                }
                ros::spinOnce();
                if (SM.msgDone.done==true)
                {
                       SM.msgDone.done=false;
                       activestate=start_point;
                }


                break;
            }



        }



       }
return 0;

    }





