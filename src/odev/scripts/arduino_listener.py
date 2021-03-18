#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from odev.msg import floatStamped
from ackermann_msgs.msg import AckermannDriveStamped

def cmd_callback(data, serial_port):
    array = bytearray(struct.pack("f", float(data.drive.speed)))
    array.extend(bytearray(struct.pack("f", float(data.drive.steering_angle))))
    rospy.loginfo(array)
    serial_port.write(array)

serial_port = rospy.get_param('/arduino_listener/arduino_port', '/dev/ttyUSB0')

ser = serial.Serial(serial_port, 57600)

speedL_pub = rospy.Publisher('speedL_stamped', floatStamped, queue_size=10)
speedR_pub = rospy.Publisher('speedR_stamped', floatStamped, queue_size=2)
ack_cmd_sud = rospy.Subscriber('ackermann_cmd', AckermannDriveStamped, cmd_callback, ser)

rospy.init_node('arduino_listener', anonymous=True)
rate = rospy.Rate(100)
speedL = floatStamped()
speedR = floatStamped()
header = Header(0, rospy.Time.now(), "")


while not rospy.is_shutdown():
    if ser.in_waiting != 0:
        line = ser.readline()
        if len(line) != 9:
            continue
        speedL.data = struct.unpack('f', line[0:4])[0]
        speedR.data = struct.unpack('f', line[4:8])[0]
        speedL_pub.publish(speedL)
        speedR_pub.publish(speedR)
        header.stamp = rospy.Time.now()
        header.seq += 1
        speedL.header = header
        speedR.header = header
    rate.sleep()
ser.close()
