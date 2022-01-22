from cycler import cycler

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
#plt.rcParams.update({'font.size': 12})
fig = plt.figure(figsize=(6, 5))
# Sience+IEEE plot
# https://github.com/garrettj403/SciencePlots/blob/master/styles/journals/ieee.mplstyle
plt.rcParams['axes.prop_cycle'] = cycler('color', ['k', 'r', 'b', 'g']) + cycler('ls', ['-', '--', ':', '-.'])
plt.xlim([0, 45])
plt.ylim([-0.2, 1.0])
plt.xlabel('Time [sec]', fontsize=14)
plt.ylabel('Robustness', fontsize=14)
plt.axhspan(-10, 0, color = 'pink', alpha = 0.5)
linewidth = 2

times, values = robMessgae2TimeValueList(bag, '/rtamt/system/collEgoObs_gt')
plt.plot(times, values, linewidth=linewidth, label='System')

times, values = robMessgae2TimeValueList(bag, '/rtamt/perception/errLoc')
plt.plot(times, values, linewidth=linewidth, label='Perception')

times, values = robMessgae2TimeValueList(bag, '/rtamt/planner/collGlobalPathObs')
plt.plot(times, values, linewidth=linewidth, label='Planner')

times, values = robMessgae2TimeValueList(bag, '/rtamt/controller/referrLocGlobalPath')
plt.plot(times, values, linewidth=linewidth, label='controller')


bag.close()
plt.legend(fontsize=14)
plt.savefig(bagName +'.pdf', bbox_inches='tight')
plt.show()