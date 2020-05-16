#!/usr/bin/env python

import rospy

from sample_code.srv import WordCount, WordCountResponse


def count_words(request):
    return len(request.words.split())

rospy.init_node('service_server')

servise = rospy.Service('word_count', WordCount, count_words)

rospy.spin()