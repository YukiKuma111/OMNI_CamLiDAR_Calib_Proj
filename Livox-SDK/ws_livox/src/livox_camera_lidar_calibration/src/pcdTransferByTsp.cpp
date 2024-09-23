#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>

#include "CustomMsg.h"
#include "common.h"

using namespace std;

struct pointData {
    float x;
    float y;
    float z;
    int i;
};

vector<pointData> vector_data;
livox_ros_driver::CustomMsg livox_cloud;
string input_bag_path, output_path;

string formatNsec(uint32_t nsec);
void loadAndSavePointcloud();
void writeTitle(const string filename, unsigned long point_num);
void writePointCloud(const string filename, const vector<pointData> singlePCD);
void dataSave(const string filename);

string formatNsec(uint32_t nsec) {
    // 将nsec转换为字符串
    string nsec_str = to_string(nsec);
    
    // 检查nsec是否是9位数
    if (nsec_str.length() < 9) {
        // 如果nsec不足9位，则在前面补0
        nsec_str = string(9 - nsec_str.length(), '0') + nsec_str;
    }
    
    // 返回处理后的字符串
    return nsec_str;
}

void loadAndSavePointcloud() {
    fstream file_;
    file_.open(input_bag_path, ios::in);
    if (!file_) {
        cout << "File " << input_bag_path << " does not exit" << endl;
        return;
    }
    ROS_INFO("Start to load the rosbag %s", input_bag_path.c_str());
    rosbag::Bag bag;
    try {
        bag.open(input_bag_path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    // types.push_back(string("livox_ros_driver/CustomMsg"));
    types.push_back(string("livox_ros_driver2/CustomMsg")); // Modified by yuku
    rosbag::View view(bag, rosbag::TypeQuery(types));

    int cloudCount = 0;
    double last_save_time = 0;
    for (const rosbag::MessageInstance& m : view) {
        livox_cloud = *(m.instantiate<livox_ros_driver::CustomMsg>());
        
        ros::Time ros_time = livox_cloud.header.stamp;
        double current_time = ros_time.sec + ros_time.nsec / 1000000000;

        if (cloudCount == 0 || (current_time - last_save_time) >= 0) {
            vector_data.clear(); // 清空数据
            for (uint i = 0; i < livox_cloud.point_num; ++i) {
                pointData myPoint;
                myPoint.x = livox_cloud.points[i].x;
                myPoint.y = livox_cloud.points[i].y;
                myPoint.z = livox_cloud.points[i].z;
                myPoint.i = livox_cloud.points[i].reflectivity;

                vector_data.push_back(myPoint);
            }

            stringstream ss;
            ss << output_path << ros_time.sec << "." << formatNsec(ros_time.nsec) << ".pcd";
            dataSave(ss.str()); // 以时间戳作为文件名保存

            last_save_time = current_time; // 更新上次保存时间
            cloudCount++;
        }

    }

    vector_data.clear();
}

void writeTitle(const string filename, unsigned long point_num) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    } else {
        outfile << "# .PCD v.7 - Point Cloud Data file format" << endl;
        outfile << "VERSION .7" << endl;
        outfile << "FIELDS x y z intensity" << endl;
        outfile << "SIZE 4 4 4 4" << endl;
        outfile << "TYPE F F F F" << endl;
        outfile << "COUNT 1 1 1 1" << endl;
        outfile << "WIDTH " << long2str(point_num) << endl;
        outfile << "HEIGHT 1" << endl;
        outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
        outfile << "POINTS " << long2str(point_num) << endl;
        outfile << "DATA ascii" << endl;
    }
    ROS_INFO("Save file %s", filename.c_str());
}

void writePointCloud(const string filename, const vector<pointData> singlePCD) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    } else {
        for (unsigned long i = 0; i < singlePCD.size(); ++i) {
            outfile << float2str(singlePCD[i].x) << " " << float2str(singlePCD[i].y) << " " << float2str(singlePCD[i].z) << " " << int2str(singlePCD[i].i) << endl;
        }
    }
}

void dataSave(const string filename) {
    writeTitle(filename, vector_data.size());
    writePointCloud(filename, vector_data);
}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_bag_path", input_bag_path)) {
        cout << "Can not get the value of input_bag_path" << endl;
        exit(1);
    } else {
        cout << input_bag_path << endl;
    }
    if (!ros::param::get("output_pcd_path", output_path)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcdTransfer");
    getParameters();

    loadAndSavePointcloud();

    ROS_INFO("Finish all!");
    return 0;
}
