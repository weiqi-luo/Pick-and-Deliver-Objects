#include<final_state_machine/state_machine.h>

namespace final_project {
state_machine::state_machine()
{

     msgDone.done = false;
     msg3D.x = 0;
     msg3D.y = 0;
     msg3D.z = 0;
     msgPickdone.done_pick = false;
     msgface.result = 0;
}
state_machine::~state_machine()
{

}

void state_machine::processCoord3D(const final_msg_srv::Coordinates_3D::ConstPtr &coord_3d)
{
       state_machine::msg3D = *coord_3d;
}

void state_machine::processCurrentPos(const final_msg_srv::current_position::ConstPtr &current_pos)
{
    state_machine::msgPos = *current_pos;
}

void state_machine::processDone(const final_msg_srv::done::ConstPtr &arm_done)
{
    state_machine::msgDone = *arm_done;
}

void state_machine::processPickdone(const final_msg_srv::done_pick::ConstPtr &pick_done)
{
    state_machine::msgPickdone = *pick_done;
}

 void state_machine::processFace(const final_msg_srv::face_message::ConstPtr &face)
 {
     state_machine::msgface = *face;
 }

}
