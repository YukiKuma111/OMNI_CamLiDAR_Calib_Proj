import cv2
import numpy as np
import open3d as o3d
import argparse
import os

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description="Project point cloud data on image.")
    parser.add_argument('-img', '--image', type=str, default="../Livox-SDK/ws_livox/data/photo/0.png", help="Path to the image file")
    parser.add_argument('-pcd', '--pointcloud', type=str, default="../Livox-SDK/ws_livox/data/pcdFiles/0.pcd", help="Path to the point cloud file")
    parser.add_argument('-ex', '--extrinsic', type=str, default="../Livox-SDK/ws_livox/data/pcdFiles/parameters/extrinsic.txt", help="Folder path to the extrinsic.txt")
    parser.add_argument('-s', '--save', type=str, default="../Livox-SDK/ws_livox/data/projection/projection.png", help="Path to save the projected image")
    return parser.parse_args()

def parse_line(line):
    """解析一行数据并转换为浮点数"""
    return [round(float(val.strip().replace(';', '')), 5) for val in line.split()]

def transform_point(point, R, t):
    """将点云进行旋转和平移变换"""
    return np.dot(R, point) + t

def equirectangular_projection(pts_3d, W, H, error_point):
    """将3D点投影到2D像素坐标"""
    pts_2d = []
    for pt in pts_3d:
        x, y, z = pt
        try:
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                raise ValueError(f"NaN value detected in point: {pt}")
            
            theta = np.arctan2(y, x) * (-1)
            phi = np.arcsin(z / np.linalg.norm([x, y, z]))

            u = int((theta + np.pi) / (2 * np.pi) * W)
            v = int((np.pi / 2 - phi) / np.pi * H)

            pts_2d.append((u, v))

        except ValueError as e:
            error_point += 1
            print(f"Error: {e}, skipping point: {pt}")
        except ZeroDivisionError:
            print(f"Error: Division by zero for point: {pt}, skipping.")
        except Exception as e:
            print(f"Unexpected error for point {pt}: {e}, skipping.")
    
    return pts_2d

def get_color(cur_depth, min_depth, max_depth):
    """根据深度获取对应颜色"""
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

def project_pcd2img():
    args = parse_args()

    # 读取extrinsic.txt
    extrinsic_file = os.path.join(args.extrinsic)

    # 读取图像的宽度和高度
    image_origin = cv2.imread(args.image)
    H, W, _ = image_origin.shape

    # 读取extrinsic矩阵并提取旋转矩阵和平移矩阵
    with open(extrinsic_file, 'r') as f:
        lines = f.readlines()
        extrinsic_matrix = np.array([parse_line(line) for line in lines[1:4]])

    R_mat = extrinsic_matrix[:, :3]
    tvec = extrinsic_matrix[:, 3]

    # 读取点云数据
    cloud_origin = o3d.io.read_point_cloud(args.pointcloud)

    # 对点云中的点进行旋转和平移变换
    pts_3d = [transform_point(point_3d, R_mat, tvec) for point_3d in np.asarray(cloud_origin.points)]

    # 获取点的2D像素坐标
    error_point = 0
    pts_2d = equirectangular_projection(pts_3d, W, H, error_point)

    # 找出深度的最小值和最大值
    min_depth = min(point_3d[0] for point_3d in pts_3d)
    max_depth = max(point_3d[0] for point_3d in pts_3d)

    # 初始化投影后的图像
    image_project = image_origin.copy()

    # 根据深度给点上色并在图像上绘制
    for i, point_2d in enumerate(pts_2d):
        x, y = point_2d
        if 0 <= x < W and 0 <= y < H:
            cur_depth = pts_3d[i][0]
            color = get_color(cur_depth, min_depth, max_depth)
            image_project[y, x] = np.clip(color, 0, 255)

    # 显示原始图像和投影后的图像
    cv2.imshow("origin image", image_origin)
    cv2.imshow("project image", image_project)

    # 保存投影图像
    if args.save:
        cv2.imwrite(args.save, image_project)

    cv2.waitKey(0)

if __name__ == "__main__":
    project_pcd2img()
