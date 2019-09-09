#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import python_state_machine as ps



def main():
    rospy.init_node('python_state_machine')

    sm = smach.StateMachine(outcomes=['mission_complete', 'cannot_find_prof'])

    with sm:
        smach.StateMachine.add('StartPoint', ps.StartPoint(),
                               transitions={'seeing': 'WithBookInRoomX'})

        smach.StateMachine.add('WithBookInRoomX', ps.WithBookInRoomX(),
                               transitions={'with_prof': 'WithBookInRoomXWithProf',
                                            'without_prof': 'WithBookInRoomXWithoutProf'})
        smach.StateMachine.add('WithBookInRoomXWithProf', ps.WithBookInRoomXWithProf(),
                               transitions={'give': 'WithoutBook'})
        smach.StateMachine.add('WithBookInRoomXWithoutProf', ps.WithBookInRoomXWithoutProf(),
                               transitions={'update_prob_table': 'cannot_find_prof'})
        smach.StateMachine.add('WithoutBook', ps.WithoutBook(), transitions={'done': 'mission_complete'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
