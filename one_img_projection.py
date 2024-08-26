import cv2
import numpy as np
import open3d as o3d
import argparse
import os

# 设置命令行参数解析
parser = argparse.ArgumentParser(description="Process image and point cloud data.")
parser.add_argument('-img', '--image', type=str, default="img/0.png", help="Path to the image file")
parser.add_argument('-pcd', '--pointcloud', type=str, default="pcd/0.pcd", help="Path to the point cloud file")
parser.add_argument('-para', '--parameters', type=str, default="para/", help="Folder path to the parameters (intrinsic.txt and extrinsic.txt)")
parser.add_argument('-s', '--save', type=str, default="result/projection.png", help="Path to save the projected image")

args = parser.parse_args()

# 读取intrinsic.txt 和 extrinsic.txt
intrinsic_file = os.path.join(args.parameters, 'intrinsic.txt')
extrinsic_file = os.path.join(args.parameters, 'extrinsic.txt')

# 读取intrinsic矩阵和distortion系数
def parse_line(line):
    """Helper function to parse a line and convert to floats with 5 decimal places."""
    return [round(float(val.strip().replace(';', '')), 5) for val in line.split()]

with open(intrinsic_file, 'r') as f:
    lines = f.readlines()
    intrinsic_matrix = np.array([parse_line(line) for line in lines[1:4]])
    distortion_coeffs = np.array(parse_line(lines[6]))

# 读取extrinsic矩阵并提取旋转矩阵和平移矩阵
with open(extrinsic_file, 'r') as f:
    lines = f.readlines()
    extrinsic_matrix = np.array([parse_line(line) for line in lines[1:4]])

R_mat = extrinsic_matrix[:, :3]
tvec = extrinsic_matrix[:, 3]

# Convert rotation matrix to rotation vector
rvec, _ = cv2.Rodrigues(R_mat)

# Read image and point cloud
image_origin = cv2.imread(args.image)
cloud_origin = o3d.io.read_point_cloud(args.pointcloud)

# Extract 3D points within a certain range
pts_3d = []
for point_3d in np.asarray(cloud_origin.points):
    if 0 < point_3d[0] < 100 and -1 < point_3d[2] < 100:
        pts_3d.append((point_3d[0], point_3d[1], point_3d[2]))

# 初始化最小值和最大值为第一个点的x坐标
min_x = 0
max_x = 0

# 遍历所有点，更新最小值和最大值
for point_3d in pts_3d:
    x = point_3d[0]
    if x < min_x:
        min_x = x
    elif x > max_x:
        max_x = x

max_depth = max_x
min_depth = min_x

# 定义函数根据深度获取颜色
def get_color(cur_depth):
    scale = (max_depth - min_depth) / 10
    if cur_depth < min_depth:
        return (0, 0, 255)  # 返回蓝色
    elif cur_depth < min_depth + scale:
        green = int((cur_depth - min_depth) / scale * 255)
        return (0, green, 255)  # 返回蓝到黄的渐变色
    elif cur_depth < min_depth + scale * 2:
        red = int((cur_depth - min_depth - scale) / scale * 255)
        return (0, 255, 255 - red)  # 返回黄到红的渐变色
    elif cur_depth < min_depth + scale * 4:
        blue = int((cur_depth - min_depth - scale * 2) / scale * 255)
        return (blue, 255, 0)  # 返回红到绿的渐变色
    elif cur_depth < min_depth + scale * 7:
        green = int((cur_depth - min_depth - scale * 4) / scale * 255)
        return (255, 255 - green, 0)  # 返回绿到黄的渐变色
    elif cur_depth < min_depth + scale * 10:
        blue = int((cur_depth - min_depth - scale * 7) / scale * 255)
        return (255, 0, blue)  # 返回黄到蓝的渐变色
    else:
        return (255, 0, 255)  # 返回紫色

# Project 3D points into image view
pts_2d, _ = cv2.projectPoints(np.array(pts_3d), rvec, tvec, intrinsic_matrix, distortion_coeffs)
image_project = image_origin.copy()

for i, point_2d in enumerate(pts_2d):
    x, y = point_2d.ravel()
    x, y = int(x), int(y)
    if 0 <= x < image_origin.shape[1] and 0 <= y < image_origin.shape[0]:
        cur_depth = pts_3d[i][0]  # 获取当前点的深度
        color = get_color(cur_depth)  # 根据深度获取颜色
        image_project[y, x] = color  # 设置点云的颜色

cv2.imshow("origin image", image_origin)
cv2.imshow("project image", image_project)

# 如果提供了保存路径，则保存图片
if args.save:
    cv2.imwrite(args.save, image_project)

cv2.waitKey(0)
