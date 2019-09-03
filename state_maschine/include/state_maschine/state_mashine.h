#ifndef STATEMASHINE_H
#define STATEMASHINE_H

/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <ros/console.h>

#include <tutorial5_msg_srv/Coordinates_3D.h>
#include <tutorial5_msg_srv/current_position.h>
#include <tutorial5_msg_srv/done.h>
#include <tutorial5_msg_srv/DesiredObject.h>
#include <tutorial5_msg_srv/desired3D_pos.h>
#include <tutorial5_msg_srv/desired_position.h>
#include <tutorial5_msg_srv/done_pick.h>


namespace Tutorial_5
{
   class state_mashine
        {


      public:
      //! Define the Structure of local Variables
      tutorial5_msg_srv::Coordinates_3D msg3D;
      tutorial5_msg_srv::current_position msgPos;
      tutorial5_msg_srv::done msgDone;
      tutorial5_msg_srv::done_pick msgPickdone;

      
        state_mashine();
        ~state_mashine();


      //------------------ Callbacks -------------------

     
      void processCoord3D(const tutorial5_msg_srv::Coordinates_3D::ConstPtr &coord_3d);
      void processCurrentPos(const tutorial5_msg_srv::current_position::ConstPtr &current_pos);
      void processDone(const tutorial5_msg_srv::done::ConstPtr &arm_done);
      void processPickdone(const tutorial5_msg_srv::done_pick::ConstPtr &pick_done);


    
};

}

#endif // STATEMASHINE_H
