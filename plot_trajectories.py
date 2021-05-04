import matplotlib.pyplot as plt
import numpy as np
import os

def plot_trajectories(log_dir):
    all_traj = []
    plt.figure(0)
    plt.figure(1)
    plt.figure(2)
    for fn in os.listdir(log_dir):
        if fn.startswith('traj'):
            # fn = fn_list[i]
            traj_name = f"{log_dir}/{fn}"
            with open(traj_name,'r') as traj_file:
                traj = [] 
                for line in traj_file:
                    line = line.replace('\n','')
                    line = line.rstrip().split(' ')
                    line = [float(j) for j in line] 
                    traj.append(line)

                all_traj.append(traj)

                traj = np.array(traj)
                plt.figure(0)
                plt.plot(traj[:,2], traj[:,1], '.')
                plt.figure(1)
                plt.plot(traj[:,5], traj[:,4], '.')
                plt.figure(2)
                plt.plot(np.sqrt((traj[:,1]-traj[:,4])**2+(traj[:,2]-traj[:,5])**2)) 

    plt.show()

if __name__ == "__main__":
    log_directory = './log/log_2021-05-03_17-11-03'
    plot_trajectories(log_directory)