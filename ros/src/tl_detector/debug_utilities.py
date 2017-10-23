#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 21 00:18:35 2017

@author: student
"""

import numpy as np
import matplotlib.pyplot as plt

def plot_waypoints_2D(waypoints):
    num = len(waypoints)
    x = []
    y = []
    color = []
    area = []
    alpha = []
    for i in range(0,num,100):
        wp = waypoints[i]
        x.append(wp.pose.pose.position.x)
        y.append(wp.pose.pose.position.y)
        temp = np.double(i)/np.double(num)
        color.append(temp)
        area.append(temp)
        alpha.append(temp)
    
    plt.scatter(x, y, marker='o', s=area, c=color, alpha=0.5)
    plt.savefig('base_waypoints.png')

