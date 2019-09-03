#include<state_maschine/state_mashine.h>

namespace Tutorial_5 {
state_mashine::state_mashine()
{

     msgDone.done=false;
     msg3D.x=0;
     msg3D.y=0;
     msg3D.z=0;
     msgPickdone.done_pick=false;
}
state_mashine::~state_mashine()
{

}

void state_mashine::processCoord3D(const tutorial5_msg_srv::Coordinates_3D::ConstPtr &coord_3d)
{
       state_mashine::msg3D=*coord_3d;
}

void state_mashine::processCurrentPos(const tutorial5_msg_srv::current_position::ConstPtr &current_pos)
{
    state_mashine::msgPos=*current_pos;
}

void state_mashine::processDone(const tutorial5_msg_srv::done::ConstPtr &arm_done)
{
    state_mashine::msgDone=*arm_done;
}

void state_mashine::processPickdone(const tutorial5_msg_srv::done_pick::ConstPtr &pick_done)
{
    state_mashine::msgPickdone=*pick_done;
}



}
