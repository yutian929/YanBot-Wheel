#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import csv
import os
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
from datetime import datetime


def parse_timestamp(timestamp_str):
    """
    Parse the timestamp from the CSV file into a ROS time.
    The format of the timestamp in the CSV is assumed to be ISO 8601.
    """
    dt = datetime.strptime(timestamp_str, "%Y-%m-%dT%H:%M:%S.%f%z")
    # Convert to seconds and nanoseconds
    secs = int(dt.timestamp())
    nsecs = int((dt.timestamp() - secs) * 1e9)
    return rospy.Time(secs, nsecs)


def read_csv_and_publish():
    rospy.init_node("imu_publisher", anonymous=True)

    # Create publishers
    imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)
    mag_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=10)

    # Get the path to the CSV file relative to the script's directory
    script_dir = os.path.dirname(
        os.path.realpath(__file__)
    )  # Get the current script directory
    csv_file_path = os.path.join(
        script_dir, "data", "data_imu9_1.csv"
    )  # Build the full path
    print(f"csv_file: {csv_file_path}")

    # Read CSV file
    with open(csv_file_path, "r") as csv_file:
        csv_reader = csv.DictReader(csv_file)
        rate = rospy.Rate(30)  # Default rate if timestamps are inconsistent

        for row in csv_reader:
            if rospy.is_shutdown():
                break

            # Parse the timestamp from the CSV
            try:
                ros_time = parse_timestamp(row["loggingTime(txt)"])
            except Exception as e:
                rospy.logwarn(f"Failed to parse timestamp: {e}")
                continue

            # Create IMU message
            imu_msg = Imu()
            header = Header()
            header.stamp = ros_time  # Use the parsed timestamp
            header.frame_id = "imu_link"  # Adjust the frame ID as needed

            imu_msg.header = header
            imu_msg.orientation.x = float(row["motionQuaternionX(R)"])
            imu_msg.orientation.y = float(row["motionQuaternionY(R)"])
            imu_msg.orientation.z = float(row["motionQuaternionZ(R)"])
            imu_msg.orientation.w = float(row["motionQuaternionW(R)"])

            imu_msg.angular_velocity.x = float(row["motionRotationRateX(rad/s)"])
            imu_msg.angular_velocity.y = float(row["motionRotationRateY(rad/s)"])
            imu_msg.angular_velocity.z = float(row["motionRotationRateZ(rad/s)"])

            imu_msg.linear_acceleration.x = (
                float(row["motionUserAccelerationX(G)"]) * 9.81
            )  # Convert G to m/s^2
            imu_msg.linear_acceleration.y = (
                float(row["motionUserAccelerationY(G)"]) * 9.81
            )
            imu_msg.linear_acceleration.z = (
                float(row["motionUserAccelerationZ(G)"]) * 9.81
            )

            # Create MagneticField message
            mag_msg = MagneticField()
            mag_msg.header = header  # Same timestamp and frame as IMU
            mag_msg.magnetic_field.x = (
                float(row["motionMagneticFieldX(µT)"]) * 1e-6
            )  # Convert µT to Tesla
            mag_msg.magnetic_field.y = float(row["motionMagneticFieldY(µT)"]) * 1e-6
            mag_msg.magnetic_field.z = float(row["motionMagneticFieldZ(µT)"]) * 1e-6

            # Publish the messages
            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)

            rate.sleep()


if __name__ == "__main__":
    try:
        read_csv_and_publish()
    except rospy.ROSInterruptException:
        pass
