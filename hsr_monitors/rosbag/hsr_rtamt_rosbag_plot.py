import rosbag
import matplotlib.pyplot as plt

bag = rosbag.Bag('system_all_green.bag')

fig = plt.figure(figsize=(12, 8))
ax1 = fig.add_subplot(4, 1, 1)
ax2 = fig.add_subplot(4, 1, 2)
ax3 = fig.add_subplot(4, 1, 3)
ax4 = fig.add_subplot(4, 1, 4)

times = []
values = []
for topic, rob, t in bag.read_messages(topics=['/rtamt/system/collEgoObs_gt']):
    times.append(rob.header.stamp.to_sec())
    values.append(rob.value)
ax1.plot(times, values)

times = []
values = []
for topic, rob, t in bag.read_messages(topics=['/rtamt/perception/errLoc']):
    times.append(rob.header.stamp.to_sec())
    values.append(rob.value)
ax2.plot(times, values)

times = []
values = []
for topic, rob, t in bag.read_messages(topics=['/rtamt/planner/collGlobalPathObs']):
    times.append(rob.header.stamp.to_sec())
    values.append(rob.value)
ax3.plot(times, values)

times = []
values = []
for topic, rob, t in bag.read_messages(topics=['/rtamt/controller/referrLocGlobalPath']):
    times.append(rob.header.stamp.to_sec())
    values.append(rob.value)
ax4.plot(times, values)


bag.close()
plt.savefig('system_all_green.pdf', bbox_inches='tight')
plt.show()