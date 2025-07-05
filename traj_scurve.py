# S curve Program
# 
# Book: Trajectory Planning for Automatic Machines and Robots --
# Authors: Luigi Biagiotti · Claudio Melchiorri
# Section: 3.2.2 Trajectory with preassigned acceleration and velocity
# Eqns: 3.8, 3.9, 3.10

import numpy as np
import matplotlib.pyplot as plt

import argparse

deltaT = 0.01 # 10 millisec

desc = """Program to calculate s curve given starting and ending positions
as well as maximum achievable velocity and acceleration. Units in counts and seconds. """
def parse_args():
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument("--q0", "-q0",type=float, default=0.0, help='Starting position in counts')
    parser.add_argument("--q1", "-q1", type=float, default=10000.0, help='Ending position in counts')
    parser.add_argument("--vm", "-vm", type=float, default=8000.0, help='Maximum velocity achievable in counts/sec')
    parser.add_argument("--am", "-am", type=float, default=20000.0, help='Maximum acceleration achievable in counts/sec^2')
    args = parser.parse_args()

    return args.q0, args.q1, args.vm, args.am

def is_trap(h, vm, am):
    if h - vm*vm/am > 0:
        return True
    else:
        return False


def traj_times(q0, q1, vm, am):
    h = np.abs(q1 - q0)
    is_it_trap = is_trap(h, vm, am)
    if is_it_trap:
        Ta = vm/am
        T = (h*am + vm*vm)/(am*vm)
        return is_it_trap, Ta, T
    else:
        Ta = np.sqrt(h/am)
        T = 2.0*Ta
        return is_it_trap, Ta, T


def traj(t, q0, q1, am, T, Ta, is_it_trap):
    dir_move = (q1-q0)/np.abs(q1-q0)
    if is_it_trap:
        if (t>=0) and (t<=Ta):
            return q0 + dir_move*0.5*am*t*t, dir_move*am*t, dir_move*am
        elif (t>Ta) and (t<=T-Ta):
            return q0 + dir_move*am*Ta*(t-Ta/2.0), dir_move*am*Ta, 0.0
        elif (t>T-Ta) and (t<=T):
            return q1 - dir_move*0.5*am*(T-t)*(T-t), dir_move*am*Ta-dir_move*am*(t-(T-Ta)), -dir_move*am
        else:
            return q1, 0, 0
    else:
        if (t>=0) and (t<=Ta):
            return q0 + dir_move*0.5*am*t*t, dir_move*am*t, dir_move*am
        elif (t>Ta) and (t<=T):
            return q1 - dir_move*0.5*am*(T-t)*(T-t), dir_move*am*Ta-dir_move*am*(t-(T-Ta)), -dir_move*am
        else:
            return q1, 0, 0

vec_traj = np.vectorize(traj)


if __name__ == "__main__":
    #args = parse_args()
    q0, q1, vm, am = parse_args()
    h = np.abs(q1 - q0)

    #print(is_trap(h, vm, am))
    #print(q0, q1, h, am, vm)
    is_it_trap, Ta, T = traj_times(q0, q1, vm, am)
    #print(is_it_trap, Ta, T)
    npts = int(T/deltaT) + 1
    #print(npts)
    ts = np.linspace(0,T,npts)
    pos, vel, acc = vec_traj(ts, q0, q1, am, T, Ta, is_it_trap)


    fig, ax = plt.subplots(3, sharex=True)
    ax[0].plot(ts, pos)
    ax[0].plot(ts, pos,'r+',label='Pos')
    ax[1].plot(ts, vel)
    ax[1].plot(ts, vel,'r+', label='Velocity')
    ax[2].plot(ts, acc)
    ax[2].plot(ts, acc,'r+', label='Accel')
    ax[2].set_xlabel('Time')
    for ax1 in ax:
        ax1.grid()
        ax1.legend()
    

    plt.show()




