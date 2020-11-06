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


ser = serial.serial_for_url('/dev/ttyACM0', timeout=1, baudrate=115200)
N_values = 100;
plt.style.use('seaborn')
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    
def serial_read(q_out):
    while True:
        try:
            ser_line = ser.readline()
            q_out.put(ser_line)
        except Exception as e:
            if hasattr(e, 'message'):
                print('serial:', e.message)
            else:
                print('serial:', e)
                
                
def plot_update(q_in):
    t = np.linspace(-5, 0, N_values)
    data = collections.deque(maxlen=N_values)
    data_gyro = collections.deque(maxlen=N_values)
    for i in range(N_values):
        data.append([None]*3)
        data_gyro.append([None]*3)
    
    plt.ion()
    fig = plt.figure(figsize=(4,10))
    
    # 3D plot set-up 
    lim_3d = [-1.5, 1.5]
    ticks_3d = np.linspace(-1, 1, 3)
    ax_3d = fig.add_subplot(311, projection='3d')
    ax_3d.set_title('Position')
    ax_3d.set_facecolor('white')
    ax_3d.w_xaxis.set_pane_color([0.915, 0.915, 0.95]) # seaborn background
    ax_3d.w_yaxis.set_pane_color([0.915, 0.915, 0.95])
    ax_3d.w_zaxis.set_pane_color([0.915, 0.915, 0.95])
    
    # Acceleration time plot
    ax = fig.add_subplot(312)
    ax.set_title('Acceleration vs Time')
    ax.set_xlabel('t / s')
    ax.set_ylabel('a / g')
    data_array = np.array(data)
    ln = [None]*3
    ln[0] = plt.plot(t, data_array[:,0], label='x')[0]
    ln[1] = plt.plot(t, data_array[:,1], label='y')[0]
    ln[2] = plt.plot(t, data_array[:,2], label='z')[0]
    ax.legend()
    
    # Gyroscope time plot
    ax_gyro = fig.add_subplot(313)
    ax_gyro.set_title('Angular Velocity vs Time')
    ax_gyro.set_xlabel('t / s')
    ax_gyro.set_ylabel('v / °/s')
    data_array_gyro = np.array(data)
    ln_gyro = [None]*3
    ln_gyro[0] = plt.plot(t, data_array_gyro[:,0], label='x')[0]
    ln_gyro[1] = plt.plot(t, data_array_gyro[:,1], label='y')[0]
    ln_gyro[2] = plt.plot(t, data_array_gyro[:,2], label='z')[0]
    ax_gyro.legend()
    
    fig.canvas.draw()
    plt.show(block=False)
    plt.tight_layout()

    while True:
        try:
            while not q_in.empty():
                line = q_in.get()
                message = json.loads(bytes(line).decode())
                if message.get('meas') == 'acc':
                    val = message.get('values')
                    acc = [val.get('x'), val.get('y'), val.get('z')]
                    data.append(acc)
                elif message.get('meas') == 'gyro':
                    val = message.get('values')
                    gyro = [val.get('x'), val.get('y'), val.get('z')]
                    data_gyro.append(gyro)
            
            # 3D position
            [c, b, a] = calc_angles(data[-1][0], data[-1][1], data[-1][2])
            [x, y, z] = get_cube()
            
            vec = np.array([[1.4,0,0], [0,1.4,0], [0,0,1.4]])
            origin = np.array([[0,0,0], [0,0,0], [0,0,0]])            
            vec_rot = vec * Rx(a) * Ry(b) * Rz(c)
            
            d = [-1, 1]
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
            ax_3d.set_title('Position')
                
            # accerlation vs time
            data_array = np.array(data)
            for i in range(len(ln)):
                ln[i].set_ydata(data_array[:,i])
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(-5, 0)
            
            # accerlation vs time
            data_array_gyro = np.array(data_gyro)
            for i in range(len(ln_gyro)):
                ln_gyro[i].set_ydata(data_array_gyro[:,i])
            ax_gyro.relim()
            ax_gyro.autoscale_view()
            ax_gyro.set_xlim(-5, 0)
            
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

def get_cube():   
    phi = np.arange(1,10,2)*np.pi/4
    Phi, Theta = np.meshgrid(phi, phi)

    x = np.cos(Phi)*np.sin(Theta)
    y = np.sin(Phi)*np.sin(Theta)
    z = np.cos(Theta)/np.sqrt(2)
    return x,y,z
    
if __name__ == '__main__':
    
    queue_raw = multiprocessing.Queue(100)
    
    serial = multiprocessing.Process(None, serial_read, args=(queue_raw,))
    plotter = multiprocessing.Process(None, plot_update, args=(queue_raw,))    
    
    plotter.start()
    serial.start()
