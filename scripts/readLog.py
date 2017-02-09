import struct
import numpy as np
import matplotlib.pyplot as plt

usec=0
setpoint_roll=1
setpoint_pitch=2
setpoint_yaw=3
setpoint_w_roll=4
setpoint_w_pitch=5
setpoint_w_yaw=6
setpoint_x=7
setpoint_y=8
setpoint_z=9
setpoint_v_x=10
setpoint_v_y=11
setpoint_v_z=12
state_roll=13
state_pitch=14
state_yaw=15
state_w_roll=16
state_w_pitch=17
state_w_yaw=18
state_x=19
state_y=20
state_z=21
state_v_x=22
state_v_y=23
state_v_z=24
acc_x=25
acc_y=26
acc_z=27
gyro_x=28
gyro_y=29
gyro_z=30
extPos_x=31
extPos_y=32
extPos_z=33
extPos_roll=34
extPos_pitch=35
extPos_yaw=36
extPos_v_x=37
extPos_v_y=38
extPos_v_z=39
receivedExternalPosition=40

fmt = "QfffffffffffffffffffffffffffffffffffffffB"

result = np.empty((0,41))

with open("log.bin", "rb") as f:
	for data in struct.iter_unpack(fmt, f.read()):
		result = np.vstack([result, np.array(data)])

print(result)
print(np.mean(np.diff(result[:,usec])))
print(1/250*1e6)

np.savetxt("log.csv", result, delimiter=",", header="usec,setpoint_roll,setpoint_pitch,setpoint_yaw,setpoint_w_roll,setpoint_w_pitch,setpoint_w_yaw,setpoint_x,setpoint_y,setpoint_z,setpoint_v_x,setpoint_v_y,setpoint_v_z,state_roll,state_pitch,state_yaw,state_w_roll,state_w_pitch,state_w_yaw,state_x,state_y,state_z,state_v_x,state_v_y,state_v_z,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,extPos_x,extPos_y,extPos_z,extPos_roll,extPos_pitch,extPos_yaw,extPos_v_x,extPos_v_y,extPos_v_z,receivedExternalPosition")

# acc_x
#print(np.mean(result[:,30]), np.std(result[:,30]))
time = (result[:,usec] - result[0,usec]) / 1e6

# xyz plot
plt.figure(1)
plt.subplot(311)
plt.plot(time, result[:,setpoint_x], label = "setpoint")
plt.plot(time, result[:,state_x], label = "state")
plt.ylabel("x [m]")
plt.xlabel("t [s]")
plt.legend()

plt.subplot(312)
plt.plot(time, result[:,setpoint_y], label = "setpoint")
plt.plot(time, result[:,state_y], label = "state")
plt.ylabel("y [m]")
plt.xlabel("t [s]")

plt.subplot(313)
plt.plot(time, result[:,setpoint_z], label = "setpoint")
plt.plot(time, result[:,state_z], label = "state")
plt.ylabel("z [m]")
plt.xlabel("t [s]")

plt.savefig('xyz.png', bbox_inches='tight')
plt.show()

# rpy plot
plt.figure(2)
plt.subplot(311)
plt.plot(time, result[:,setpoint_roll], label = "setpoint")
plt.plot(time, result[:,state_roll], label = "state")
plt.ylabel("roll [deg]")
plt.xlabel("t [s]")

plt.subplot(312)
plt.plot(time, result[:,setpoint_pitch], label = "setpoint")
plt.plot(time, result[:,state_pitch], label = "state")
plt.ylabel("pitch [deg]")
plt.xlabel("t [s]")

plt.subplot(313)
plt.plot(time, result[:,setpoint_yaw], label = "setpoint")
plt.plot(time, result[:,state_yaw], label = "state")
plt.ylabel("yaw [deg]")
plt.xlabel("t [s]")

plt.savefig('rpy.png', bbox_inches='tight')
plt.show()

# v plot
plt.figure(1)
plt.subplot(311)
plt.plot(time, result[:,setpoint_v_x], label = "setpoint")
plt.plot(time, result[:,state_v_x], label = "state")
plt.ylabel("v_x [m/s]")
plt.xlabel("t [s]")

plt.subplot(312)
plt.plot(time, result[:,setpoint_v_y], label = "setpoint")
plt.plot(time, result[:,state_v_y], label = "state")
plt.ylabel("v_y [m/s]")
plt.xlabel("t [s]")

plt.subplot(313)
plt.plot(time, result[:,setpoint_v_z], label = "setpoint")
plt.plot(time, result[:,state_v_z], label = "state")
plt.ylabel("v_z [m/s]")
plt.xlabel("t [s]")

plt.savefig('v.png', bbox_inches='tight')
plt.show()

# w plot
plt.figure(2)
plt.subplot(311)
plt.plot(time, result[:,setpoint_w_roll], label = "setpoint")
plt.plot(time, result[:,state_w_roll], label = "state")
plt.ylabel("roll [rad/s]")
plt.xlabel("t [s]")

plt.subplot(312)
plt.plot(time, result[:,setpoint_w_pitch], label = "setpoint")
plt.plot(time, result[:,state_w_pitch], label = "state")
plt.ylabel("pitch [rad/s]")
plt.xlabel("t [s]")

plt.subplot(313)
plt.plot(time, result[:,setpoint_w_yaw], label = "setpoint")
plt.plot(time, result[:,state_w_yaw], label = "state")
plt.ylabel("yaw [rad/s]")
plt.xlabel("t [s]")

plt.savefig('w.png', bbox_inches='tight')
plt.show()

# extpos xyz plot
plt.figure(1)
plt.subplot(311)
plt.plot(time, result[:,extPos_x])
plt.ylabel("ext x [m]")
plt.xlabel("t [s]")
plt.legend()
# plot line if vicon measurement was present
# print(np.flatnonzero(result[:,receivedExternalPosition]))
for idx in np.flatnonzero(result[:,receivedExternalPosition]):
	# print(idx, time[idx])
	plt.axvline(x=time[idx])

plt.subplot(312)
plt.plot(time, result[:,extPos_y])
plt.ylabel("ext y [m]")
plt.xlabel("t [s]")

plt.subplot(313)
plt.plot(time, result[:,extPos_z])
plt.ylabel("ext z [m]")
plt.xlabel("t [s]")

plt.savefig('ext_xyz.png', bbox_inches='tight')
plt.show()

# ext rpy plot
plt.figure(2)
plt.subplot(311)
plt.plot(time, result[:,extPos_roll])
plt.ylabel("ext roll [deg]")
plt.xlabel("t [s]")

plt.subplot(312)
plt.plot(time, result[:,extPos_pitch])
plt.ylabel("ext pitch [deg]")
plt.xlabel("t [s]")

plt.subplot(313)
plt.plot(time, result[:,extPos_yaw])
plt.ylabel("ext yaw [deg]")
plt.xlabel("t [s]")

plt.savefig('ext_rpy.png', bbox_inches='tight')
plt.show()

# ext v plot
plt.figure(1)
plt.subplot(311)
plt.plot(time, result[:,extPos_v_x])
plt.ylabel("ext v_x [m/s]")
plt.xlabel("t [s]")

plt.subplot(312)
plt.plot(time, result[:,extPos_v_y])
plt.ylabel("ext v_y [m/s]")
plt.xlabel("t [s]")

plt.subplot(313)
plt.plot(time, result[:,extPos_v_z])
plt.ylabel("ext v_z [m/s]")
plt.xlabel("t [s]")

plt.savefig('ext_v.png', bbox_inches='tight')
plt.show()