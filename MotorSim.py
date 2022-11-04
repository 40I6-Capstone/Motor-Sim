import Discrete_LTI_Sim as dlti
import numpy as np
import matplotlib.pyplot as plt
import os

MAX_STEPS = int(10E5)

ts = 0.001  # seconds
d_eps = 0.01  # position epsilon
v_eps = 0.005  # velocity epsilon


# if is_test == 0:
free_speed = float(input("FreeSpeed? (rpm)\n"))*2*np.pi/60  # rad/s
stall_torque = float(input("Stall Torque? (N*m)\n"))  # Nm
stall_current = float(input("Stall Current? (mA)\n"))  # A
free_current = float(input("Free Current? (mA)\n"))  # A
mass = float(input("Mass of chassis? (kg)\n"))  # kg
dist = float(input("Distance per trip? (m)\n"))  # m
wheel_rad = float(input("Wheel Radius? (m)\n"))  # m
name = input("file name prefix?\n")
show_graphs = input("Do you want graph pop-ups? (y/n)")

# else:
#     name = 'test'
#     free_speed = 200*2*np.pi/60
#     stall_torque = 0.0784532
#     stall_current = 1000
#     mass = 1
#     dist = 3.9
#     wheel_rad = 0.0254
#     free_current = 150

stall_alpha = stall_torque/(wheel_rad**2 * mass)


# States
# pos (rad)
# vel (rad/s)
# accel (rad/s^2)
# current drawn(mAs) ( so far how much current has been drawn)
# current draw (mA) convert accel state
A = [[1, ts, 0, 0, 0],
     [0, 1, ts, 0, 0],
     [0, -stall_alpha/free_speed, 0, 0, 0],
     [0, 0, 0, 1, ts],
     [0, (free_current-stall_current)/free_speed, 0, 0, 0]]

B = [0, 0, 0, 0, 0]  # Input transform matrix

K = [0, 0, stall_alpha, 0, stall_current]

# Output transform matrix (pos,vel,accel)
# pos: return as m
# vel: return as m/s
# accel: return as m/s^2
# total current drawn: return as Ah
# current draw: return
C = [[wheel_rad, 0, 0, 0, 0],
     [0, wheel_rad, 0, 0, 0],
     [0, 0, wheel_rad, 0, 0],
     [0, 0, 0, 1/60, 0],
     [0, 0, 0, 0, 1]]
# Input on Ouput transform matrix
D = [0, 0, 0, 0, 0]
# No input
u = [0, 0, 0, 0, 0]
# Initial motor state
motorx_0 = [0, 0, 0, 0, 0]

lti = dlti.DiscLTI(A,
                   B,
                   C,
                   D,
                   K,
                   motorx_0)


def endCondition(travelled, dest):
    return abs(travelled-dest) < d_eps


accelerated = False


def checkTopSpeed(time, dist, speed):
    global time_to_topspeed
    global dist_to_topspeed
    global accelerated
    if (not accelerated and abs(speed-free_speed*wheel_rad) < v_eps):
        time_to_topspeed = time
        dist_to_topspeed = dist
        accelerated = True


# Simulate
if not (name == ''):
    dir = os.path.join(os.getcwd(), 'data', name)
    relative_dir = 'data/'+name
else:
    dir = os.path.join(os.getcwd(), 'data')
    relative_dir = 'data'

if not os.path.exists(dir):
    os.makedirs(relative_dir)

file = open(relative_dir+"/MotorSimData", "w+")
output = []
valid_end = False

for step in range(0, MAX_STEPS, 1):
    lti.update(u)
    output.append(np.insert(lti.y, 0, round(step*ts, 4)))
    for val in output[step]:
        file.write(format(val, '5.4f')+"\t")
    file.write("\n")
    checkTopSpeed(round(step*ts, 4), lti.y[0], lti.y[1])
    if (endCondition(lti.y[0], dist)):
        valid_end = True
        break
file.close()

if valid_end:
    # Summarize Results
    summary = open(relative_dir+"/Summary.txt", "w+")
    summary.write("Summary for Test: "+name+"\n\n")
    summary.write("Trip Time: " + format(output[-1][0], '5.4f')+"s\n")
    summary.write("Time to top speed: " +
                  format(time_to_topspeed, '5.4f')+"s\n")
    summary.write("Current Drawn for trip: " +
                  format(output[-1][4], '6.4f')+"mAh\n")
    summary.write("Free Spead Theoretical: " +
                  format(free_speed*wheel_rad, '5.4f')+"m/s\n")
    summary.write("Top Speed Reached: " +
                  format(output[-1][2], '5.4f')+"m/s\n")
    summary.close()
else:
    print("BAD RESULTS, DID NOT CONVERGE IN TIME")


# Plot
x_points = [i[0] for i in output]
fig, axs = plt.subplots(2)
fig.suptitle('Velocity & Accleration')

axs[0].plot(x_points, [i[2] for i in output], label="vel")
axs[0].axhline(y=free_speed*wheel_rad, linestyle='--',
               color="green", label="Free Speed")
axs[0].axvline(x=time_to_topspeed, linestyle='--',
               color="green", label="Time to Free Speed")
axs[0].set_yticks(np.linspace(0, free_speed*wheel_rad, 10))
axs[0].set_ylabel("m/s")


axs[1].plot(x_points, [i[3] for i in output], label="accel")
axs[1].set_yticks(
    np.linspace(0, stall_alpha*wheel_rad, 10))
axs[1].set_ylabel("m/s^2")


for (index, plot) in enumerate(axs):
    plot.set_xticks(np.linspace(0, output[-1][0], 10))
    plot.set_xlabel("s")
    plot.legend()
    plot.grid()

fig.savefig(relative_dir+'/VelocityAndAcceleration.png')

plt.figure(2)
plt.title("Position")
plt.plot(x_points, [i[1] for i in output], label="pos")
plt.ylabel("m")
plt.xlabel("s")
plt.grid()
plt.legend()
plt.xticks(np.linspace(0, output[-1][0], 10))
plt.yticks(np.linspace(0, dist, 10))
plt.figure(2).savefig(relative_dir+'/Position.png')


plt.figure(3)
plt.title("Current Drawn")
plt.plot(x_points, [i[4] for i in output], label="current")
plt.ylabel("mAh")
plt.xlabel("s")
plt.grid()
plt.legend()
plt.xticks(np.linspace(0, output[-1][0], 10))
plt.yticks(np.linspace(0, output[-1][4], 10))
plt.figure(3).savefig(relative_dir+"/Current.png")

if(show_graphs == 'y' or show_graphs == 'yes'):
    plt.show()

