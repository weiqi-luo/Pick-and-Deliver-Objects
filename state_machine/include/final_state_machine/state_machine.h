#ifndef STATEMACHINE_H
#define STATEMACHINE_H

/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <ros/console.h>

#include <final_msg_srv/Coordinates_3D.h>
#include <final_msg_srv/current_position.h>
#include <final_msg_srv/done.h>
#include <final_msg_srv/DesiredObject.h>
#include <final_msg_srv/desired3D_pos.h>
#include <final_msg_srv/desired_position.h>
#include <final_msg_srv/done_pick.h>
#include <final_msg_srv/face_message.h>


namespace final_project
{
   class state_machine
        {


      public:
      //! Define the Structure of local Variables
      final_msg_srv::Coordinates_3D msg3D;
      final_msg_srv::current_position msgPos;
      final_msg_srv::done msgDone;
      final_msg_srv::done_pick msgPickdone;
      final_msg_srv::face_message msgface;

      
        state_machine();
        ~state_machine();


      //------------------ Callbacks -------------------

     
      void processCoord3D(const final_msg_srv::Coordinates_3D::ConstPtr &coord_3d);
      void processCurrentPos(const final_msg_srv::current_position::ConstPtr &current_pos);
      void processDone(const final_msg_srv::done::ConstPtr &arm_done);
      void processPickdone(const final_msg_srv::done_pick::ConstPtr &pick_done);
      void processFace(const final_msg_srv::face_message::ConstPtr &face);


    
};

}

#endif // STATEMACHINE_H
