#!/usr/bin/env python
# # -*- coding: utf-8 -*-
import rospy
import numpy as np

rospy.init_node('a')
init_prob_table = np.array([0.3, 0.5, 0.2])
print('#----------------------------------------------------------------------------------#')
print('The probability table is: ', init_prob_table)
print('#----------------------------------------------------------------------------------#')
ind = np.argmax(init_prob_table)

print(ind)