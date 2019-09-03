
/*********************************************************************
* STD INCLUDES
********************************************************************/
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <final_state_machine/state_machine.h>
#include <final_msg_srv/tiago_pick.h>
#include <std_srvs/Empty.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
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
    ros::ServiceClient cli_obj = n.serviceClient<final_msg_srv::DesiredObject>("desired_object");
    ros::ServiceClient cli_hea = n.serviceClient<std_srvs::Empty>("desired_signal");
    ros::ServiceClient cli_arm = n.serviceClient<final_msg_srv::desired3D_pos>("move_arm");
    ros::ServiceClient cli_nav = n.serviceClient<final_msg_srv::desired_position>("move_base");
    ros::ServiceClient cli_pick = n.serviceClient<std_srvs::Empty>("pick_gui");
    ros::ServiceClient cli_place = n.serviceClient<std_srvs::Empty>("place_gui");
    ros::ServiceClient cli_face = n.serviceClient<std_srvs::Empty>("face_recognition");

    //Publisher
    ros::Publisher pub_3d = n.advertise<final_msg_srv::Coordinates_3D>("point3D_from_main",10);


    //INITIALIZE THE SUBSCRIBER
    final_project::state_machine SM;
    ros::Subscriber sub_3d = n.subscribe("point3D",10, &final_project::state_machine::processCoord3D, &SM);
    ros::Subscriber sub_pose = n.subscribe("current_pose",10, &final_project::state_machine::processCurrentPos, &SM);
    ros::Subscriber sub_arm = n.subscribe("process_done",10, &final_project::state_machine::processDone, &SM);
    ros::Subscriber sub_face = n.subscribe("face_information", 10, &final_project::state_machine::processFace, &SM);

    // MSG VARIABLE FOR THE SERVICE MESSAGE
    final_msg_srv::DesiredObject desired_obj;
    final_msg_srv::desired3D_pos desired_pos3d;
    final_msg_srv::desired_position desired_pos;
    std_srvs::Empty prepare_grasp;
    final_msg_srv::Coordinates_3D coord_3d_main;
    std_srvs::Empty tiago_move;
    std_srvs::Empty head_down;
    std_srvs::Empty face_recog;



    //String variable for desired object
    std::string obj_in;



    //Float variable for navigation goals
    //room 1
    float position_x1,position_y1,position_z1;
    float orientation_w1,orientation_x1,orientation_y1,orientation_z1;

    position_x1 = 0.140535116196;
    position_y1 = 0.0995450019836;
    position_z1 = 0.0;
    orientation_x1 = 0.0;
    orientation_y1 = 0.0;
    orientation_z1 = -0.00411474404786;
    orientation_w1 = 0.999991534405;

    //room 2
    float position_x2,position_y2,position_z2;
    float orientation_w2,orientation_x2,orientation_y2,orientation_z2;
    position_x2 = 0.101202249527;
    position_y2 = 2.7521469593;
    position_z2 = 0;
    orientation_x2 = 0;
    orientation_y2 = 0;
    orientation_z2 = 0.0178972696889;
    orientation_w2 = 0.999839831042;

    //room 3
    float position_x3,position_y3,position_z3;
    float orientation_w3,orientation_x3,orientation_y3,orientation_z3;
    position_x3 = 0.513135671616;
    position_y3 = 6.07820367813;
    position_z3 = 0;
    orientation_x3 = 0;
    orientation_y3 = 0;
    orientation_z3 = -0.708948840604;
    orientation_w3 = 0.705259910534;

    //room 4
    float position_x4,position_y4,position_z4;
    float orientation_w4,orientation_x4,orientation_y4,orientation_z4;
    position_x4 = 0.0682221651077;
    position_y4 = 1.43129086494;
    position_z4 = 0;
    orientation_x4 = 0;
    orientation_y4 = 0;
    orientation_z4 = 0.429491053289;
    orientation_w4 = 0.903071113005;

    //room 5
    float position_x5,position_y5,position_z5;
    float orientation_w5,orientation_x5,orientation_y5,orientation_z5;
    position_x5 = -1.43666386604;
    position_y5 = 7.83529090881;
    position_z5 = 0;
    orientation_x5 = 0;
    orientation_y5 = 0;
    orientation_z5 = -0.0159106027312;
    orientation_w5 = 0.999873418349;

    //Position table
    Eigen::MatrixXd Positions(7,5);
    Positions << position_x1, position_x2, position_x3, position_x4, position_x5,
                 position_y1, position_y2, position_y3, position_y4, position_y5,
                 position_z1, position_z2, position_z3, position_z4, position_z5,
                 orientation_w1, orientation_w2, orientation_w3, orientation_w4, orientation_w5,
                 orientation_x1, orientation_x2, orientation_x3, orientation_x4, orientation_x5,
                 orientation_y1, orientation_y2, orientation_y3, orientation_y4, orientation_y5,
                 orientation_z1, orientation_z2, orientation_z3, orientation_z4, orientation_z5;

    int goal_room;

    //3d Point of object
    float desired_x;
    float desired_y;
    float desired_z;

    //done or not
    bool done_process;



    //Float coordinates for a point on table
    float table_x,table_y,table_z;


    //define state space
    enum States {start_point,with_book,with_book_in_roomx,with_book_in_roomx_without_prof,
                 with_book_in_roomx_with_prof,without_book};


    States activestate;
    activestate = with_book;


    while(ros::ok())
    {

        switch (activestate)
        {
            case start_point:
            {

                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                cli_pick.call(prepare_grasp);
                activestate = with_book;
                break;
            }
            case with_book:
            {
                goal_room = 2;
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                desired_pos.request.position_x = Positions(0,goal_room);
                desired_pos.request.position_y = Positions(1,goal_room);
                desired_pos.request.position_z = Positions(2,goal_room);
                desired_pos.request.orientation_w = Positions(3,goal_room);
                desired_pos.request.orientation_x = Positions(4,goal_room);
                desired_pos.request.orientation_y = Positions(5,goal_room);
                desired_pos.request.orientation_z = Positions(6,goal_room);
                cli_nav.call(desired_pos);
                ROS_INFO("navigation");

                activestate=with_book_in_roomx;
                break;
            }

            case with_book_in_roomx:
            {
                ros::spinOnce();
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                ROS_INFO("face recognition");
                cli_face.call(face_recog);
                while(true)
                {
//                    if(SM.msgface.result == 1)
//                    {
//                        activestate = with_book_in_roomx_without_prof;
//                        SM.msgface = 0;
//                        break;
//                    }

//                    else if(SM.msgface.result == 2)
//                    {
//                        activestate = with_book_in_roomx_with_prof;
//                        SM.msgface = 0;
//                        break;
//                    }
//                    else
//                    {
//                        continue;
//                    }
                    float result;
                    std::cin>>result;
                    if(result == 1 )
                    {
                        activestate = with_book_in_roomx_with_prof;
                    }
                    else
                    {
                        activestate = with_book_in_roomx_without_prof;
                    }


                }


                break;
            }

            case with_book_in_roomx_without_prof:
            {
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                //ros::spinOnce();
                ROS_INFO("I need to go to another room");
                activestate=with_book;
                break;
            }



            case with_book_in_roomx_with_prof:
            {
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;

                cli_place.call(prepare_grasp);


                activestate=without_book;

                break;
            }

            case without_book:
            {
                std::cout << "[ INFO] [" << ros::Time::now() << "]: current state:" <<activestate<< std::endl;
                cli_place.call(prepare_grasp);
                activestate=without_book;
                break;
            }

         }



     }

return 0;

}





