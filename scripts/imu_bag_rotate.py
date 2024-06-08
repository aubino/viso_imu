import rosbag
import rospy
import numpy as np
from sensor_msgs.msg import Imu

def rotate_vector(vector, rotation_matrix):
    return np.dot(rotation_matrix, vector)

def main():
    input_bag = 'imu.bag'
    output_bag = 'imu_rotated.bag'
    topic = '/bmi088/raw'
    
    # Define your rotation matrix here (example: 90 degrees rotation around z-axis)
    angle = np.deg2rad(90)
    R = np.array([[np.cos(angle), -np.sin(angle), 0],
                  [np.sin(angle), np.cos(angle), 0],
                  [0, 0, 1]])

    with rosbag.Bag(output_bag, 'w') as outbag:
        with rosbag.Bag(input_bag, 'r') as inbag:
            for topic, msg, t in inbag.read_messages(topics=[topic]):
                if isinstance(msg, Imu):
                    # Rotate the linear acceleration
                    lin_acc = np.array([msg.linear_acceleration.x,
                                        msg.linear_acceleration.y,
                                        msg.linear_acceleration.z])
                    rotated_lin_acc = rotate_vector(lin_acc, R)
                    msg.linear_acceleration.x = rotated_lin_acc[0]
                    msg.linear_acceleration.y = rotated_lin_acc[1]
                    msg.linear_acceleration.z = rotated_lin_acc[2]

                    # Rotate the angular velocity
                    ang_vel = np.array([msg.angular_velocity.x,
                                        msg.angular_velocity.y,
                                        msg.angular_velocity.z])
                    rotated_ang_vel = rotate_vector(ang_vel, R)
                    msg.angular_velocity.x = rotated_ang_vel[0]
                    msg.angular_velocity.y = rotated_ang_vel[1]
                    msg.angular_velocity.z = rotated_ang_vel[2]

                outbag.write(topic, msg, t)

if __name__ == '__main__':
    main()
