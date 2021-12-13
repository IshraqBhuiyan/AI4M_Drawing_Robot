#!/usr/bin/env python3

import rospy
import ros_numpy
import open3d as o3d
import numpy as np
from trakstar_ros.msg import Sensor
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2, PointField
from pynput import keyboard
import time

pointcloud = None
stop = False
deleting = False
last_point = None
save = False
point_dist_threshold = 0.001
num_points_delete=10
cmd_key =False

def on_press(key):
    global stop, deleting, save, cmd_key
    try:
        if(key.name=='alt'):
            cmd_key = True
    except:
        pass
    if(cmd_key):
        try:
            if(key.char=='p'):
                print("Paused")
                stop = True
            elif(key.char=='c'):
                print("Continue")
                stop = False
            elif(key.char=='s'):
                print("Save")
                stop=True
                save=True
            elif(key.char=='d'):
                print("Deleting")
                deleting = True
        except:
            pass

def on_release(key):
    global cmd_key
    try:
        if(key.name=='alt'):
            cmd_key=False
    except:
        pass

def o3d_to_ros(cloud_o3d):
  cloud_np = np.asarray(cloud_o3d.points)
  num_points = cloud_np.shape[0]
  data = np.zeros(num_points, dtype=([('x', np.float32),
        ('y', np.float32),
        ('z', np.float32)
        ]))
  data['x'] = cloud_np[:,0]
  data['y'] = cloud_np[:,1]
  data['z'] = cloud_np[:,2]

  ros_msg = ros_numpy.msgify(PointCloud2, data)
  ros_msg.header.stamp = rospy.Time.now()
  ros_msg.header.frame_id = "tracker"
  ros_msg.height = 1
  ros_msg.width = num_points
  ros_msg.fields=[]

  ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
  ros_msg.fields.append(PointField(
                          name="y",
                          offset=4,
                          datatype=PointField.FLOAT32, count=1))
  ros_msg.fields.append(PointField(
                          name="z",
                          offset=8,
                          datatype=PointField.FLOAT32, count=1))
  ros_msg.is_bigendian = False
  ros_msg.point_step = 12
  ros_msg.row_step = ros_msg.point_step * num_points
  ros_msg.is_dense = True
  return ros_msg

def dist(p1, p2):
    return np.sqrt(np.sum((p1-p2)**2))

def trakstar_callback(msg):
    global pointcloud, last_point, stop, deleting
    if(not stop and not deleting):
        point = np.zeros(3)
        point[0] = msg.pose_array[0].position.x
        point[1] = msg.pose_array[0].position.y
        point[2] = msg.pose_array[0].position.z
        if((last_point is None) or dist(point, last_point)>point_dist_threshold):
            pointcloud.points.append(point)
            last_point = point

    #rospy.loginfo("Pointcloud Size %d"%(len(pointcloud.points)))

if __name__ == "__main__":
    rospy.init_node("tracker_processing")
    pointcloud = o3d.geometry.PointCloud()
    #pointcloud = o3d.io.read_point_cloud("Ellipsoid.pcd")
    rospy.Subscriber("/TrakSTAR/Sensor", Sensor,trakstar_callback)
    pc_pub = rospy.Publisher("/tracker_cloud", PointCloud2, queue_size=10)
    listener = keyboard.Listener(on_press = on_press, on_release=on_release)
    listener.start()
    rate = rospy.Rate(4)

    while(not rospy.is_shutdown() and not save):
        # num_points = len(pointcloud.points)
        if(deleting):
            pointcloud.points = pointcloud.points[:-num_points_delete]
            deleting=False
        pc_msg = o3d_to_ros(pointcloud)
        pc_pub.publish(pc_msg)
        rate.sleep()
    #listener.stop()
    listener.stop()
    if(save):
        pcd_file=input("Enter filename")
        o3d.io.write_point_cloud(pcd_file, pointcloud)

    #pointcloud.estimate_normals()
    #pcd_outlier,_ = pointcloud.remove_statistical_outlier(nb_neighbors = 100, std_ratio=2.0)
    #pcd_outlier.paint_uniform_color([0,0,1])
    #pointcloud.paint_uniform_color([0,1,0])
    #print(len(pcd_outlier.points))
    #radii = [0.005, 0.01, 0.02, 0.04]
    #rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    #pointcloud, o3d.utility.DoubleVector(radii))
    #o3d.visualization.draw_geometries([pcd_outlier])
    #vis.destroy_window()