#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import time

class DartV2Control():
    def __init__(self,dart_bot): # must be called with the dartv2 object
        self.__dart_bot = dart_bot
        print ("Control: Dartv2 robot is working as ",self.__dart_bot.exec_robot())
        # example : keep track of the last encoder values (maybe useful for odometry)
        self.encoders_front_left_last, self.encoders_front_right_last = self.__dart_bot.powerboard.get_front_encoders()
        self.encoders_time_last = time.time()

    # example get front encoders (and keep track of the last values)
    # usefull for computing delta encoders
    def get_front_encoders_now_and_last (self,init=False):
        left_last, right_last = self.encoders_front_left_last, self.encoders_front_right_last
        time_last = self.encoders_time_last
        left_now, right_now = self.__dart_bot.powerboard.get_front_encoders()
        time_now = time.time()
        # update the last values
        self.encoders_front_left_last, self.encoders_front_right_last = left_now, right_now 
        self.encoders_time_last = time_now 
        # if init is True, the last values are set to the now values
        if init:
            self.encoders_front_left_last, self.encoders_front_right_last = left_now, right_now 
            self.encoders_time_last = time_now
        return  left_last, right_last, time_last, left_now, right_now, time_now
    
    # example : stop the robot
    def stop(self):
        self.__dart_bot.powerboard.stop()
        print ("Control: Dartv2 robot is stopped")


