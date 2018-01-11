#! /usr/bin/python
# -*- coding: utf-8 -*-

import math, time, pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def init():
    x = np.linspace(0.0,2.0*math.pi,100)
    y = np.sin(x)
    fig = plt.figure(1)
    ax = fig.add_subplot(111)
    ax.plot(x,y)
    pickle.dump(ax, file('images/background.pickle', 'w'))

def loop():
    for i in range(5):
        ax = pickle.load(file('images/background.pickle'))
        ax.add_patch( patches.Rectangle((math.pi/2.0+math.pi/10.0*i,0.2),math.pi,0.3))
        plt.pause(0.2)

if __name__ == '__main__':
    init()
    loop()
