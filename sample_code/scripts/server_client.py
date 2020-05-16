#!/usr/bin/env python

import rospy
import numpy as np


from sample_code.srv import WordCount

import sys

rospy.init_node('service_client')
rospy.wait_for_service('word_count')
word_counter = rospy.ServiceProxy('word_count', WordCount)
words = ''.join(sys.argv[1:])
word_count = word_counter(words)
print(words, '->', word_count.count)

