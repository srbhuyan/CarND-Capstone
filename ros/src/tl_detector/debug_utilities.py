#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 21 00:18:35 2017

@author: student
"""

import numpy as np
import matplotlib.pyplot as plt

def plot_waypoints_2D(waypoints):
    '''
    Plot the base way points
    '''
    num = len(waypoints)
    x = []
    y = []
    color = []
    for i in range(0,num,100):
        wp = waypoints[i]
        x.append(wp.pose.pose.position.x)
        y.append(wp.pose.pose.position.y)
        temp = np.double(i)/np.double(num)
        color.append(temp)
    
    plt.scatter(x, y, marker='^', s=30, c=color, alpha=0.5, 
                edgecolors='none', cmap=plt.cm.cool)
    plt.savefig('base_waypoints.png')
    
def plot_wp_vc_2D(waypoints,pose):
    '''
    Plot way points and the ego car
    '''
    # Plot the way points
    n_wp = len(waypoints)
    x_wp = []
    y_wp = []
    c_wp = []
    for i in range(0,n_wp,100):
        wp = waypoints[i]
        x_wp.append(wp.pose.pose.position.x)
        y_wp.append(wp.pose.pose.position.y)
        temp_wp = np.double(i)/np.double(n_wp)
        c_wp.append(temp_wp)
    
    plt.scatter(x_wp, y_wp, marker='^', s=30, c=c_wp, alpha=0.5, 
                edgecolors='none', cmap=plt.cm.cool)
    # Plot the car
    x_vc = [pose.pose.position.x]
    y_vc = [pose.pose.position.y]
    plt.scatter(x_vc, y_vc, marker='s', s=30, c='r', alpha=0.5)
    plt.savefig('wp_vc_2D.png')

