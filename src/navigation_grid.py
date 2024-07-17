#!/usr/bin/env python

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from concurrent.futures import ThreadPoolExecutor

class PointCloudFilter:
    def __init__(self):
        self.sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.point_cloud_callback)
        self.pub = rospy.Publisher("/filtered_point_cloud_to_follow", PointCloud2, queue_size=1)
        self.voxel_size = 0.05  # Tamanho do voxel em metros 
        self.distance_threshold = 1.0  # Distância máxima em metros
        self.executor = ThreadPoolExecutor(max_workers=4)

    def point_cloud_callback(self, msg):
        # Converter PointCloud2 para lista de pontos
        points_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        if not points_list:
            rospy.logwarn("No points within the distance threshold.")
            return

        # Dividir a lista de pontos em 4 partes
        points_chunks = np.array_split(points_list, 4)

        # Processar cada parte em uma thread separada
        futures = [self.executor.submit(self.process_chunk, chunk) for chunk in points_chunks]

        # Coletar os resultados das threads
        filtered_points = []
        for future in futures:
            filtered_points.extend(future.result())

        # Criar a point cloud reduzida com os pontos filtrados
        downsampled_pcd = self.create_voxel_grid(filtered_points)

        # Converter de volta para PointCloud2
        filtered_cloud = self.convert_to_ros_pointcloud2(downsampled_pcd, msg.header)

        # Publicar a point cloud filtrada
        self.pub.publish(filtered_cloud)

    def process_chunk(self, points):
        # Filtrar pontos que estão a mais de um metro de distância
        filtered_points = [point for point in points if np.linalg.norm(point) <= self.distance_threshold]
        return filtered_points

    def create_voxel_grid(self, points):
        if not points:
            return o3d.geometry.PointCloud()

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))

        # Aplicar filtro de voxel grid
        downsampled_pcd = pcd.voxel_down_sample(self.voxel_size)
        return downsampled_pcd

    def convert_to_ros_pointcloud2(self, pcd, header):
        points = np.asarray(pcd.points)
        ros_cloud = pc2.create_cloud_xyz32(header, points)
        return ros_cloud

if __name__ == "__main__":
    rospy.init_node("point_cloud_filter")
    PointCloudFilter()
    rospy.spin()
