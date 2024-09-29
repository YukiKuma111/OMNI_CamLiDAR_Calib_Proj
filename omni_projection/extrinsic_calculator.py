import argparse
import numpy as np

def parse_extrinsic(file_path):
    """ 读取txt文件并解析外参矩阵 """
    with open(file_path, 'r') as f:
        lines = f.readlines()
        # 提取4x4外参矩阵的前三行数据
        extrinsic_matrix = np.array([list(map(float, line.split())) for line in lines[1:4]])
        extrinsic_matrix = np.vstack((extrinsic_matrix, [0, 0, 0, 1]))  # 添加最后一行[0, 0, 0, 1]
    return extrinsic_matrix

def save_extrinsic(file_path, extrinsic_matrix):
    """ 将计算的外参矩阵保存到txt文件 """
    with open(file_path, 'w') as f:
        f.write('extrinsic\n')
        for row in extrinsic_matrix:
            f.write(' '.join(map(str, row)) + '\n')

def calculate_lidar_to_lidar_extrinsic(lidar1_to_cam, lidar2_to_cam):
    """ 计算2号激光雷达到1号激光雷达的外参矩阵 """
    # 外参矩阵的逆矩阵
    cam_to_lidar1 = np.linalg.inv(lidar1_to_cam)
    # 计算2号激光雷达到1号激光雷达的外参：lidar2_to_lidar1 = cam_to_lidar1 * lidar2_to_cam
    lidar2_to_lidar1 = np.dot(cam_to_lidar1, lidar2_to_cam)
    return lidar2_to_lidar1

def main():
    # 设置命令行参数
    parser = argparse.ArgumentParser(description="计算两个激光雷达之间的外参转换")
    parser.add_argument('-ex1', '--lidar1_to_cam', type=str, required=True, help="1号激光雷达到相机的外参文件路径")
    parser.add_argument('-ex2', '--lidar2_to_cam', type=str, required=True, help="2号激光雷达到相机的外参文件路径")
    parser.add_argument('-out', '--output', type=str, default="lidar2_to_lidar1_extrinsic.txt", help="输出外参文件路径")

    args = parser.parse_args()

    # 读取两个外参文件
    lidar1_to_cam = parse_extrinsic(args.lidar1_to_cam)
    lidar2_to_cam = parse_extrinsic(args.lidar2_to_cam)

    # 计算2号激光雷达到1号激光雷达的外参
    lidar2_to_lidar1 = calculate_lidar_to_lidar_extrinsic(lidar1_to_cam, lidar2_to_cam)

    # 保存结果到txt文件
    save_extrinsic(args.output, lidar2_to_lidar1)
    print(f"2号激光雷达到1号激光雷达的外参已保存到: {args.output}")

if __name__ == "__main__":
    main()
