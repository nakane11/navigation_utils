#!/usr/bin/env python
import argparse
import matplotlib.pyplot as plt
import numpy as np

# inflation_radius = "/move_base_node/global_costmap/inflation_layer/inflation_radius"

parser = argparse.ArgumentParser(description='inflation')
parser.add_argument('--cost_scaling_factor', '-csf', type=float, required=True)
parser.add_argument('--inflation_radius', '-ir', type=float, required=True)
args = parser.parse_args()

cost_scaling_factor =args.cost_scaling_factor
inflation_radius = args.inflation_radius
inscribed_radius = 1.0
INSCRIBED_INFLATED_OBSTACLE = 254

def J(distance):
    return np.exp(-1 * cost_scaling_factor * (distance - inscribed_radius)) * (INSCRIBED_INFLATED_OBSTACLE - 1)

x = np.linspace(inscribed_radius, inflation_radius, 100)
y = J(x)

plt.plot(x, y)
plt.xlabel("distance")
plt.ylabel("cost")
plt.show()

