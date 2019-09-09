#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import python_state_machine as ps



def main():
    rospy.init_node('python_state_machine')

    sm = smach.StateMachine(outcomes=['mission_complete'])

    with sm:
        smach.StateMachine.add('WakeUp', ps.WakeUp(),
                               transitions={'started': 'DontknowWhereIam',
                                            'sleep': 'WakeUp'})
        smach.StateMachine.add('DontknowWhereIam', ps.DontknowWhereIam(),
                               transitions={'localized': 'StartPoint'})
        smach.StateMachine.add('StartPoint', ps.StartPoint(),
                               transitions={'seeing': 'WithPerception'})
        smach.StateMachine.add('WithPerception', ps.WithPerception(),
                               transitions={'pick': 'WithBook',
                                            'pick_failed':'StartPoint'})
        smach.StateMachine.add('WithBook', ps.WithBook(), transitions={'navigation': 'WithBookInRoomX'})
        smach.StateMachine.add('WithBookInRoomX', ps.WithBookInRoomX(),
                               transitions={'with_prof': 'WithBookInRoomXWithProf',
                                            'without_prof': 'WithBookInRoomXWithoutProf'})
        smach.StateMachine.add('WithBookInRoomXWithProf', ps.WithBookInRoomXWithProf(),
                               transitions={'give': 'WithoutBook'})
        smach.StateMachine.add('WithBookInRoomXWithoutProf', ps.WithBookInRoomXWithoutProf(),
                               transitions={'update_prob_table': 'WithBook'})
        smach.StateMachine.add('WithoutBook', ps.WithoutBook(), transitions={'done': 'mission_complete'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
