#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 30 08:25:47 2020

@author: stefan
"""

import serial
import json

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
from numpy import sin, cos
import math
from itertools import product, combinations

import multiprocessing
import collections


ser = serial.serial_for_url('/dev/ttyACM2', timeout=1, baudrate=115200)
N_values = 100;
plt.style.use('seaborn')
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    
def serial_read(q_out):
    while True:
        try:
            ser_line = ser.readline()
            #print(ser_line)
            q_out.put(ser_line)
        except Exception as e:
            if hasattr(e, 'message'):
                print('serial:', e.message)
            else:
                print('serial:',e)
    
    
def data_parse(q_in, q_out):
    while True:
        try:
            line = q_in.get()
            #print(bytes(line).decode())
            message = json.loads(bytes(line).decode())
            #print(json.dumps(message, indent=2))
            val = message.get('values')
            acc = [val.get('x'), val.get('y'), val.get('z')]
            q_out.put(acc)
        except Exception as e:
            if hasattr(e, 'message'):
                print('parse:', e.message)
            else:
                print('parse:', e)
                
                
def plot_update(q_in):
    t = np.linspace(-5, 0, N_values)
    data = collections.deque(maxlen=N_values)
    for i in range(N_values):
        data.append([None]*3)
    
    plt.ion()
    fig = plt.figure(figsize=(4,8))
    ax = fig.add_subplot(212)
    data_array = np.array(data)
    ln = [None]*3
    ln[0] = plt.plot(t, data_array[:,0], label='x')[0]
    ln[1] = plt.plot(t, data_array[:,1], label='y')[0]
    ln[2] = plt.plot(t, data_array[:,2], label='z')[0]
    ax.legend()
    
    lim_3d = [-1.5, 1.5]
    ticks_3d = np.linspace(-1, 1, 3)
    ax_3d = fig.add_subplot(211, projection='3d')
    ax_3d.set_facecolor('white')
    ax_3d.w_xaxis.set_pane_color([0.915, 0.915, 0.95]) # seaborn background
    ax_3d.w_yaxis.set_pane_color([0.915, 0.915, 0.95])
    ax_3d.w_zaxis.set_pane_color([0.915, 0.915, 0.95])
    
    fig.canvas.draw()
    plt.show(block=False)
    
    while True:
        try:
            while not q_in.empty():
                acc = q_in.get()
                data.append(acc)
            
            # 3D position
            [c, b, a] = calc_angles(data[-1][0], data[-1][1], data[-1][2])
            [x, y, z] = get_cube(0)
            
            vec = np.array([[1.4,0,0], [0,1.4,0], [0,0,1.4]])
            origin = np.array([[0,0,0], [0,0,0], [0,0,0]])            
            vec_rot = vec * Rx(a) * Ry(b) * Rz(c)
            
            d = [-1, 1]
            theta = np.radians(30)
            ax_3d.clear()
            for s, e in combinations(np.array(list(product(d,d,d))), 2):
                if np.sum(np.abs(s-e)) == d[1]-d[0]:
                    s_rotated = (s * Rx(a) * Ry(b) * Rz(c)).tolist()[0]
                    e_rotated = (e * Rx(a) * Ry(b) * Rz(c)).tolist()[0]
                    ax_3d.plot3D(*zip(s_rotated,e_rotated), color=colors[3], linewidth=1)
            
            for i in range(3):
                ax_3d.quiver(origin[i,0], origin[i,1], origin[i,2], 
                             vec_rot[i,0], vec_rot[i,1], vec_rot[i,2], 
                             color=colors[i])
            ax_3d.set_xlim(lim_3d)
            ax_3d.set_ylim(lim_3d)
            ax_3d.set_zlim(lim_3d)
            ax_3d.set_xticks(ticks_3d)
            ax_3d.set_yticks(ticks_3d)
            ax_3d.set_zticks(ticks_3d)
            ax_3d.set_xlabel('x')
            ax_3d.set_ylabel('y')
            ax_3d.set_zlabel('z')
                
            # accerlation vs time
            data_array = np.array(data)
            for i in range(len(ln)):
                ln[i].set_ydata(data_array[:,i])
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(-5, 0)
            fig.canvas.flush_events() 
        except Exception as e:
            if hasattr(e, 'message'):
                print('plot:', e.message)
            else:
                print('plot:', e)
    
    
def calc_angles(x, y, z):
    if x == None or y == None or z == None:
        return [0, 0, 0]
    
    alpha = math.atan2(x, y)
    beta = math.atan2(x, z)
    gamma = math.atan2(y, z)
    return [alpha, beta, gamma]

def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, cos(theta),-sin(theta)],
                   [ 0, sin(theta), cos(theta)]])  
def Ry(theta):
  return np.matrix([[ cos(theta), 0, sin(theta)],
                   [ 0           , 1, 0           ],
                   [-sin(theta), 0, cos(theta)]])
def Rz(theta):
  return np.matrix([[ cos(theta), -sin(theta), 0 ],
                   [ sin(theta), cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

    
if __name__ == '__main__':
    
    queue_raw = multiprocessing.Queue(100)
    queue_parsed = multiprocessing.Queue(100)
    
    serial = multiprocessing.Process(None, serial_read, args=(queue_raw,))
    parser = multiprocessing.Process(None, data_parse, args=(queue_raw, queue_parsed,))
    plotter = multiprocessing.Process(None, plot_update, args=(queue_parsed,))    
    
    plotter.start()
    parser.start()
    serial.start()