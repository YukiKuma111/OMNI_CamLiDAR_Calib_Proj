import os
import cv2
import numpy as np
import open3d as o3d
import argparse

# 设置命令行参数解析
parser = argparse.ArgumentParser(description="Process image and point cloud data.")
parser.add_argument('-img', '--image', type=str, default="../Livox-SDK/ws_livox/data/img/", help="Directory containing image files")
parser.add_argument('-pcd', '--pointcloud', type=str, default="../Livox-SDK/ws_livox/data/pcdFiles/", help="Directory containing point cloud files")
parser.add_argument('-para', '--parameters', type=str, default="../Livox-SDK/ws_livox/data/parameters/", help="Folder path to the parameters (intrinsic.txt and extrinsic.txt)")
parser.add_argument('-s', '--save', type=str, default="../Livox-SDK/ws_livox/data/projection/", help="Directory to save the projected images")

args = parser.parse_args()

# 获取intrinsic和extrinsic文件路径
intrinsic_file = os.path.join(args.parameters, 'intrinsic.txt')
extrinsic_file = os.path.join(args.parameters, 'extrinsic.txt')

# 读取intrinsic矩阵和distortion系数
def parse_line(line):
    return [round(float(val.strip().replace(';', '')), 5) for val in line.split()]

with open(intrinsic_file, 'r') as f:
    lines = f.readlines()
    camera_matrix = np.array([parse_line(line) for line in lines[1:4]])
    distCoeffs = np.array(parse_line(lines[6]))

# 读取extrinsic矩阵并提取旋转矩阵和平移矩阵
with open(extrinsic_file, 'r') as f:
    lines = f.readlines()
    extrinsic_matrix = np.array([parse_line(line) for line in lines[1:4]])

R_mat = extrinsic_matrix[:, :3]
tvec = extrinsic_matrix[:, 3]
rvec, _ = cv2.Rodrigues(R_mat)

# 获取指定目录中的文件列表并排序
def get_sorted_files(directory, extension):
    return sorted([f for f in os.listdir(directory) if f.endswith(extension)])

# 从文件名中提取时间戳
def extract_timestamp(filename):
    return float('.'.join(filename.split('.')[:-1]))

# 匹配图像文件和点云文件
def match_files(image_files, pcd_files, offset=0.1):
    matched_files = []
    for img_file in image_files:
        img_timestamp = extract_timestamp(img_file)
        print(img_timestamp)
        closest_pcd_file = None
        closest_diff = float('inf')
        
        for pcd_file in pcd_files:
            pcd_timestamp = extract_timestamp(pcd_file) / 1000000
            diff = abs(img_timestamp - pcd_timestamp)
            
            if diff < closest_diff and diff <= offset:
                closest_diff = diff
                closest_pcd_file = pcd_file
        
        if closest_pcd_file:
            matched_files.append((img_file, closest_pcd_file))
            pcd_files.remove(closest_pcd_file)

    return matched_files

# 输入目录
img_dir = args.image  # 图像文件目录
img_save_dir = args.save  # 保存投影图像的目录
pcd_dir = args.pointcloud  # 点云文件目录

# 获取目录中的文件列表
image_files = get_sorted_files(img_dir, '.png')
pcd_files = get_sorted_files(pcd_dir, '.pcd')

# 匹配图像文件和点云文件
matched_files = match_files(image_files, pcd_files)

# 处理匹配的文件
for img_file, pcd_file in matched_files:
    image_origin = cv2.imread(os.path.join(img_dir, img_file))
    cloud_origin = o3d.io.read_point_cloud(os.path.join(pcd_dir, pcd_file))

    pts_3d = []
    for point_3d in np.asarray(cloud_origin.points):
        if 0 < point_3d[0] < 100 and -1 < point_3d[2] < 100:    # 筛选可视化点云点x,y,z坐标的范围
            pts_3d.append((point_3d[0], point_3d[1], point_3d[2]))

    min_x = min(point[0] for point in pts_3d)
    max_x = max(point[0] for point in pts_3d)
    max_depth = max_x
    min_depth = min_x

    def get_color(cur_depth):
        scale = (max_depth - min_depth) / 10
        if cur_depth < min_depth:
            return (0, 0, 255)
        elif cur_depth < min_depth + scale:
            green = int((cur_depth - min_depth) / scale * 255)
            return (0, green, 255)
        elif cur_depth < min_depth + scale * 2:
            red = int((cur_depth - min_depth - scale) / scale * 255)
            return (0, 255, 255 - red)
        elif cur_depth < min_depth + scale * 4:
            blue = int((cur_depth - min_depth - scale * 2) / scale * 255)
            return (blue, 255, 0)
        elif cur_depth < min_depth + scale * 7:
            green = int((cur_depth - min_depth - scale * 4) / scale * 255)
            return (255, 255 - green, 0)
        elif cur_depth < min_depth + scale * 10:
            blue = int((cur_depth - min_depth - scale * 7) / scale * 255)
            return (255, 0, blue)
        else:
            return (255, 0, 255)

    pts_2d, _ = cv2.projectPoints(np.array(pts_3d), rvec, tvec, camera_matrix, distCoeffs)
    image_project = image_origin.copy()

    for i, point_2d in enumerate(pts_2d):
        x, y = point_2d.ravel()
        x, y = int(x), int(y)
        if 0 <= x < image_origin.shape[1] and 0 <= y < image_origin.shape[0]:
            cur_depth = pts_3d[i][0]
            color = get_color(cur_depth)
            image_project[y, x] = color

    output_filename = os.path.join(img_save_dir, img_file)
    cv2.imwrite(output_filename, image_project)
