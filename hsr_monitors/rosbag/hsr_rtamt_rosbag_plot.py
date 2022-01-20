import rosbag

bag = rosbag.Bag('system_all_green.bag')
for topic, msg, t in bag.read_messages(topics=['/rtamt/system/collEgoObs_gt']):
    print(msg)
bag.close()