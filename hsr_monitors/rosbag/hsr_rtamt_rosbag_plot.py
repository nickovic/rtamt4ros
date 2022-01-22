import rosbag
import matplotlib.pyplot as plt

def robMessgae2TimeValueList(bag, topic):
    times = []
    values = []
    for topic, rob, t in bag.read_messages(topics=[topic]):
        times.append(rob.header.stamp.to_sec())
        values.append(rob.value)
    return times, values

def colorDefinition(values):
    color = 'blue'
    if min(values)<0:
        color = 'red'
    return color


bagName = 'system_all_green'
#bagName = 'planner_fail'
bag = rosbag.Bag( bagName + '.bag')
plt.rcParams.update({'font.size': 12})
fig = plt.figure(figsize=(6, 6))
xlim = (0, 45)
ylim = (-0.2, 1.0)
linewidth = 3
ax1 = fig.add_subplot(4, 1, 1)
ax2 = fig.add_subplot(4, 1, 2)
ax3 = fig.add_subplot(4, 1, 3)
ax4 = fig.add_subplot(4, 1, 4)

times, values = robMessgae2TimeValueList(bag, '/rtamt/system/collEgoObs_gt')
ax1.set(xlim=xlim, ylim=ylim)
ax1.axhspan(-10, 0, color = 'pink', alpha = 0.5)
ax1.tick_params(labelbottom=False)
ax1.set_ylabel('System')
color = colorDefinition(values)
ax1.plot(times, values, color=color, linewidth=linewidth)

times, values = robMessgae2TimeValueList(bag, '/rtamt/perception/errLoc')
ax2.set(xlim=xlim, ylim=ylim)
ax2.axhspan(-10, 0, color = 'pink', alpha = 0.5)
ax2.tick_params(labelbottom=False)
ax2.set_ylabel('Perception')
color = colorDefinition(values)
ax2.plot(times, values, color=color, linewidth=linewidth)

times, values = robMessgae2TimeValueList(bag, '/rtamt/planner/collGlobalPathObs')
ax3.set(xlim=xlim, ylim=ylim)
ax3.axhspan(-10, 0, color = 'pink', alpha = 0.5)
ax3.tick_params(labelbottom=False)
ax3.set_ylabel('Planner')
color = colorDefinition(values)
ax3.plot(times, values, color=color, linewidth=linewidth)

times, values = robMessgae2TimeValueList(bag, '/rtamt/controller/referrLocGlobalPath')
ax4.set(xlim=xlim, ylim=ylim)
ax4.axhspan(-10, 0, color = 'pink', alpha = 0.5)
ax4.set_ylabel('Controller')
ax4.set_xlabel('Time [sec]')
color = colorDefinition(values)
ax4.plot(times, values, color=color, linewidth=linewidth)


bag.close()
plt.savefig(bagName +'.pdf', bbox_inches='tight')
plt.show()