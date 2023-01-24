#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json
import matplotlib.pyplot as plt
import csv

saved_data = {}


def debug_callback(msg):
    # callback to save all data to the global dict
    global saved_data
    msg_dict = json.loads(msg.data)
    saved_data.append(msg_dict)

    '''
    for key in msg_dict.keys():
        if key in saved_data.keys():
            saved_data[key].append(msg_dict[key])
        else:
            saved_data[key] = [msg_dict[key]]
    '''

def timer_callback():
    # timer callback to update the plots

    ''' plt.close('all')
    for key in saved_data.keys():
        plt.figure()
        plt.title(key)
        x = saved_data[key]['time']
        y = saved_data[key]['data']
        plt.plot(x, y)
    plt.show() '''

    with open('output.csv', 'w') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=saved_data.keys())
        writer.writeheader()
        writer.writerows(saved_data)


def main():
    rospy.init_node('debug_server', anonymous=True)
    rospy.Subscriber("/debug", String, debug_callback)
    rospy.Timer(rospy.Duration(5), timer_callback)
    plt.show(block=True)


if __name__ == '__main__':
    main()
