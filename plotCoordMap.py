#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  plotCoordMap.py
#  
#  Copyright 2026  <rpi32@rpi32>
#  
import math
import matplotlib.pyplot as plt


def plot_vectors(data):
    """
    Plots vectors from geographic points.

    Parameters:
        data (tuple): A tuple of entries where each entry is:
                      ((lat, lon), heading_degrees, speed_m_per_s)

    Returns:
        None
    """

    fig, ax = plt.subplots()

    for entry in data:
        (lat, lon), heading_deg, speed = entry
        # ~ print(entry)
        
        #scale speed to lat/lon map size
        speed = max(-5, min(speed, 5))/130000
        
        # Convert heading (compass: 0=N, 90=E) to radians
        # Convert so that 0 degrees points "up" (north) and rotates clockwise
        heading_rad = math.radians(heading_deg)

        # Calculate delta components
        # Latitude corresponds to Y-axis, longitude to X-axis
        dx = speed * math.sin(heading_rad)  # east-west (longitude)
        dy = speed * math.cos(heading_rad)  # north-south (latitude)

        # Plot starting point
        ax.plot(lon, lat, 'bo')

        # Plot vector line
        ax.plot([lon, lon + dx], [lat, lat + dy], 'r-')

    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("Velocity Vectors from Geographic Points")
    ax.grid(True)

    plt.show()
    
    return

if __name__ == '__main__':
    testData = [[(42.4277107, -97.1024817),314,0.11], [(42.4277568, -97.1025289),128,0.75]]
    plot_vectors(testData)
