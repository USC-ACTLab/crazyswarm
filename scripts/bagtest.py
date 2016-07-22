import argparse
import os
import rosbag
import tf
import numpy as np


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file", help="input bag file")
    args = parser.parse_args()

    output_file = os.path.splitext(os.path.basename(args.bag_file))[0] + ".csv"

    bag = rosbag.Bag(args.bag_file)

    # matrix = np.empty([0, 10])
    last_est = None
    # last_pos = None
    start_time = None
    print("t, statex, goalx, viconx, statez, goalz, viconz")
    for topic, msg, t in bag.read_messages(topics=['/cf06/log1']):
        time = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9
        vals = [str(v) for v in msg.values];
        print("{}, {}".format(time, ", ".join(vals)))
        # print(time)
        # print(msg.values)
        # if start_time is None:
        #     start_time = t.to_sec()
        # print(msg)
        # if topic == "/cf06/log1":
        #     for m in msg.transforms:
        #         if m.child_frame_id == "/vicon/cf_config1/cf_config1":
        #             if last_est:
        #                 # row = np.array([])
        #                 row = np.append(row, t.to_sec() - start_time)
        #                 row = np.append(row, [m.transform.translation.x, m.transform.translation.y, m.transform.translation.z])
        #                 row = np.append(row, [last_est.translation.x, last_est.translation.y, last_est.translation.z])
        #                 quaternion = (
        #                     m.transform.rotation.x,
        #                     m.transform.rotation.y,
        #                     m.transform.rotation.z,
        #                     m.transform.rotation.w)
        #                 euler = tf.transformations.euler_from_quaternion(quaternion)
        #                 row = np.append(row, [euler[0], euler[1], euler[2]])

        #                 matrix = np.append(matrix, [row], axis=0)
        #         if m.child_frame_id == "tracker_test1":
        #             last_est = m.transform
    bag.close()

    # np.savetxt(output_file, matrix, delimiter=",", header="t,x_vicon,y_vicon,z_vicon,x_ext,y_est,z_est,roll_vicon,pitch_vicon,yaw_vicon")

