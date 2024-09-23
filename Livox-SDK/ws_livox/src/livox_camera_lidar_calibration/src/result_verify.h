#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "common.h"

using namespace std;

void getTheoreticalUV(float* theoryUV, const vector<float> &intrinsic, const vector<float> &extrinsic, double x, double y, double z);
void getTheoreticalUVOmni(float* theoryUV, const vector<float> &intrinsic, const vector<float> &extrinsic, double x, double y, double z, int width, int height);

void getUVError(const string &intrinsic_path, const string &extrinsic_path, const string &lidar_path, const string &photo_path, float* error, int threshold);
void getUVOmniError(const string &intrinsic_path, const string &extrinsic_path, const string &lidar_path, const string &photo_path, float* error, int threshold, int width, int height);

void getUVErrorNewIntrinsic(const string &extrinsic_path, const string &lidar_path, const string &photo_path, float* error, int threshold, const vector<float> &intrinsic);
void getUVOmniErrorNewIntrinsic(const string &extrinsic_path, const string &lidar_path, const string &photo_path, float* error, int threshold, const vector<float> &intrinsic, int width, int height);

// read mesured value and use theoretical U,V calculated to get the total error
void getUVError(const string &intrinsic_path, const string &extrinsic_path, const string &lidar_path, const string &photo_path, float* error, int threshold) {
    ifstream inFile_lidar;
    ifstream inFile_photo;

    inFile_lidar.open(lidar_path);
    inFile_photo.open(photo_path);
    string lineStr_lidar;
    string lineStr_photo;

    int count = 0;
    float errorTotalU = 0;
    float errorTotalV = 0;
    float errorU = 0;
    float errorV = 0;

    vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    vector<float> extrinsic;
    getExtrinsic(extrinsic_path, extrinsic);

    while(getline(inFile_lidar, lineStr_lidar) && getline(inFile_photo, lineStr_photo)) {
        if (lineStr_lidar.size() > 10 && lineStr_photo.size() > 10) {
            double x, y, z, dataU, dataV;
            string str;
            stringstream line_lidar(lineStr_lidar);
            stringstream line_photo(lineStr_photo);

            line_lidar >> str;
            x = str2double(str);

            line_lidar >> str;
            y = str2double(str);

            line_lidar >> str;
            z = str2double(str);

            line_photo >> str;
            dataU = str2double(str);

            line_photo >> str;
            dataV = str2double(str);

            float theoryUV[2] = {0, 0};
            getTheoreticalUV(theoryUV, intrinsic, extrinsic, x, y, z);

            errorU = abs(dataU - theoryUV[0]);
            errorV = abs(dataV - theoryUV[1]);
            if (errorU + errorV > threshold) {
                cout << "Data " << count << " has a error bigger than the threshold" << endl;
                cout << "xyz are " << x << " " << y << " " << z << endl;
                cout << "ErrorU is " << errorU << " errorV is " << errorV << endl;
                cout << "**********************" << endl;
            }
            errorTotalU += errorU;
            errorTotalV += errorV;
            ++count;
        }
        else if(lineStr_lidar.size() < 1 && lineStr_photo.size() < 1) {  // stop reading the data when there is an empty line
            break;
        }
        else if ((lineStr_lidar.size() < 10 && lineStr_photo.size() > 10) || (lineStr_lidar.size() > 10 && lineStr_photo.size() < 10)) {
            cout << "Lidar data and photo data not aligned!" << endl;
            exit(1);
        }
    }
    inFile_lidar.close();
    inFile_photo.close();

    error[0] = errorTotalU/count;
    error[1] = errorTotalV/count;
}

// Modified by yuku
void getUVOmniError(const string &intrinsic_path, const string &extrinsic_path, const string &lidar_path, const string &photo_path, float* error, int threshold, int width, int height) {
    ifstream inFile_lidar;
    ifstream inFile_photo;

    inFile_lidar.open(lidar_path);
    inFile_photo.open(photo_path);
    string lineStr_lidar;
    string lineStr_photo;

    int count = 0;
    float errorTotalU = 0;
    float errorTotalV = 0;
    float errorU = 0;
    float errorV = 0;

    vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    vector<float> extrinsic;
    getExtrinsic(extrinsic_path, extrinsic);

    while(getline(inFile_lidar, lineStr_lidar) && getline(inFile_photo, lineStr_photo)) {
        if (lineStr_lidar.size() > 10 && lineStr_photo.size() > 10) {
            double x, y, z, dataU, dataV;
            string str;
            stringstream line_lidar(lineStr_lidar);
            stringstream line_photo(lineStr_photo);

            line_lidar >> str;
            x = str2double(str);

            line_lidar >> str;
            y = str2double(str);

            line_lidar >> str;
            z = str2double(str);

            line_photo >> str;
            dataU = str2double(str);

            line_photo >> str;
            dataV = str2double(str);

            float theoryUV[2] = {0, 0};
            // Modified by yuku
            getTheoreticalUVOmni(theoryUV, intrinsic, extrinsic, x, y, z, width, height);

            errorU = abs(dataU - theoryUV[0]);
            errorV = abs(dataV - theoryUV[1]);
            if (errorU + errorV > threshold) {
                cout << "Data " << count << " has a error bigger than the threshold" << endl;
                cout << "xyz are " << x << " " << y << " " << z << endl;
                cout << "ErrorU is " << errorU << " errorV is " << errorV << endl;
                cout << "**********************" << endl;
            }
            errorTotalU += errorU;
            errorTotalV += errorV;
            ++count;
        }
        else if(lineStr_lidar.size() < 1 && lineStr_photo.size() < 1) {  // stop reading the data when there is an empty line
            break;
        }
        else if ((lineStr_lidar.size() < 10 && lineStr_photo.size() > 10) || (lineStr_lidar.size() > 10 && lineStr_photo.size() < 10)) {
            cout << "Lidar data and photo data not aligned!" << endl;
            exit(1);
        }
    }
    inFile_lidar.close();
    inFile_photo.close();

    error[0] = errorTotalU/count;
    error[1] = errorTotalV/count;
}

void getUVErrorNewIntrinsic(const string &extrinsic_path, const string &lidar_path, const string &photo_path, float* error, int threshold, const vector<float> &intrinsic) {
    ifstream inFile_lidar;
    ifstream inFile_photo;

    inFile_lidar.open(lidar_path);
    inFile_photo.open(photo_path);
    string lineStr_lidar;
    string lineStr_photo;

    int count = 0;
    float errorTotalU = 0;
    float errorTotalV = 0;
    float errorU = 0;
    float errorV = 0;

    vector<float> extrinsic;
    getExtrinsic(extrinsic_path, extrinsic);

    while(getline(inFile_lidar, lineStr_lidar) && getline(inFile_photo, lineStr_photo)) {
        if (lineStr_lidar.size() > 10 && lineStr_photo.size() > 10) {  // ignore the index
            double x, y, z, dataU, dataV;
            string str;
            stringstream line_lidar(lineStr_lidar);
            stringstream line_photo(lineStr_photo);

            line_lidar >> str;
            x = str2double(str);

            line_lidar >> str;
            y = str2double(str);

            line_lidar >> str;
            z = str2double(str);

            line_photo >> str;
            dataU = str2double(str);

            line_photo >> str;
            dataV = str2double(str);

            float theoryUV[2] = {0, 0};
            getTheoreticalUV(theoryUV, intrinsic, extrinsic, x, y, z);

            errorU = abs(dataU - theoryUV[0]);
            errorV = abs(dataV - theoryUV[1]);
            if (errorU + errorV > threshold) {
                cout << "Data " << count << " has a error bigger than the threshold" << endl;
                cout << "xyz are " << x << " " << y << " " << z << endl;
                cout << "errorU is " << errorU << " errorV is " << errorV << endl;
                cout << "**********************" << endl;
            }
            errorTotalU += errorU;
            errorTotalV += errorV;
            ++count;
        }
        else if(lineStr_lidar.size() < 1 && lineStr_photo.size() < 1) {  // stop reading the data when there is an empty line
            break;
        }
        else if ((lineStr_lidar.size() < 10 && lineStr_photo.size() > 10) || (lineStr_lidar.size() > 10 && lineStr_photo.size() < 10)) {
            cout << "Lidar data and photo data not aligned!" << endl;
            exit(1);
        }
    }
    inFile_lidar.close();
    inFile_photo.close();

    error[0] = errorTotalU/count;
    error[1] = errorTotalV/count;
}

// Modified by yuku
void getUVOmniErrorNewIntrinsic(const string &extrinsic_path, const string &lidar_path, const string &photo_path, float* error, int threshold, const vector<float> &intrinsic, int width, int height) {
    ifstream inFile_lidar;
    ifstream inFile_photo;

    inFile_lidar.open(lidar_path);
    inFile_photo.open(photo_path);
    string lineStr_lidar;
    string lineStr_photo;

    int count = 0;
    float errorTotalU = 0;
    float errorTotalV = 0;
    float errorU = 0;
    float errorV = 0;

    vector<float> extrinsic;
    getExtrinsic(extrinsic_path, extrinsic);

    while(getline(inFile_lidar, lineStr_lidar) && getline(inFile_photo, lineStr_photo)) {
        if (lineStr_lidar.size() > 10 && lineStr_photo.size() > 10) {  // ignore the index
            double x, y, z, dataU, dataV;
            string str;
            stringstream line_lidar(lineStr_lidar);
            stringstream line_photo(lineStr_photo);

            line_lidar >> str;
            x = str2double(str);

            line_lidar >> str;
            y = str2double(str);

            line_lidar >> str;
            z = str2double(str);

            line_photo >> str;
            dataU = str2double(str);

            line_photo >> str;
            dataV = str2double(str);

            float theoryUV[2] = {0, 0};
            getTheoreticalUVOmni(theoryUV, intrinsic, extrinsic, x, y, z, width, height);

            errorU = abs(dataU - theoryUV[0]);
            errorV = abs(dataV - theoryUV[1]);
            if (errorU + errorV > threshold) {
                cout << "Data " << count << " has a error bigger than the threshold" << endl;
                cout << "xyz are " << x << " " << y << " " << z << endl;
                cout << "errorU is " << errorU << " errorV is " << errorV << endl;
                cout << "**********************" << endl;
            }
            errorTotalU += errorU;
            errorTotalV += errorV;
            ++count;
        }
        else if(lineStr_lidar.size() < 1 && lineStr_photo.size() < 1) {  // stop reading the data when there is an empty line
            break;
        }
        else if ((lineStr_lidar.size() < 10 && lineStr_photo.size() > 10) || (lineStr_lidar.size() > 10 && lineStr_photo.size() < 10)) {
            cout << "Lidar data and photo data not aligned!" << endl;
            exit(1);
        }
    }
    inFile_lidar.close();
    inFile_photo.close();

    error[0] = errorTotalU/count;
    error[1] = errorTotalV/count;
}

// calculate theoretical U and V from x,y,z
void getTheoreticalUV(float* theoryUV, const vector<float> &intrinsic, const vector<float> &extrinsic, double x, double y, double z) {
    // set the intrinsic and extrinsic matrix
    double matrix1[3][3] = {{intrinsic[0], intrinsic[1], intrinsic[2]}, {intrinsic[3], intrinsic[4], intrinsic[5]}, {intrinsic[6], intrinsic[7], intrinsic[8]}}; 
    double matrix2[3][4] = {{extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]}, {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]}, {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]}};
    double matrix3[4][1] = {x, y, z, 1};
    
    // transform into the opencv matrix
    cv::Mat matrixIn(3, 3, CV_64F, matrix1);
    cv::Mat matrixOut(3, 4, CV_64F, matrix2);
    cv::Mat coordinate(4, 1, CV_64F, matrix3);
    
    // calculate the result of u and v
    cv::Mat result = matrixIn*matrixOut*coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);

    theoryUV[0] = u / depth;
    theoryUV[1] = v / depth;
}

// Modified by yuku
void getTheoreticalUVOmni(float* theoryUV, const vector<float>& intrinsic, const vector<float>& extrinsic, double x, double y, double z, int width, int height) {
    // 设置外参矩阵 (extrinsic)
    double matrix1[3][3] = {{intrinsic[0], intrinsic[1], intrinsic[2]}, {intrinsic[3], intrinsic[4], intrinsic[5]}, {intrinsic[6], intrinsic[7], intrinsic[8]}}; 
    double matrix2[3][4] = {
        {extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]},
        {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]},
        {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]}
    };
    double matrix3[4][1] = {x, y, z, 1};

    // 设置输入的3D点 (x, y, z)
    cv::Mat coordinate(4, 1, CV_64F, matrix3);

    // 计算激光雷达点在相机坐标系下的坐标
    cv::Mat result = cv::Mat(3, 1, CV_64F);
    cv::Mat intrinsicMat(3, 3, CV_64F, matrix1);
    cv::Mat extrinsicMat(3, 4, CV_64F, matrix2);
    result = extrinsicMat * coordinate;

    // 提取相机坐标系下的X, Y, Z坐标
    double xc = result.at<double>(0, 0);
    double yc = result.at<double>(1, 0);
    double zc = result.at<double>(2, 0);

    // 将相机坐标系下的点转换为球面坐标 (θ, φ)
    double theta = atan2(yc, xc) * (-1);    // 方位角 θ，范围在 [-π, π]
    double phi = atan2(sqrt(xc * xc + yc * yc + zc * zc), zc);  // 仰角 φ，范围在 [0, π]

    // 计算U和V坐标 (等角度投影下的像素坐标)
    float u = (theta + M_PI) / (2 * M_PI) * width;  // 将θ从 [-π, π] 映射到 [0, width]
    float v = phi / M_PI * height;                  // 将φ从 [0, π] 映射到 [0, height]

    theoryUV[0] = u;
    theoryUV[1] = v;
}