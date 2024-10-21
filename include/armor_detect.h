#pragma once
#ifndef UNTITLED1_ARMOR_DETECT_H
#define UNTITLED1_ARMOR_DETECT_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
using namespace std;
using namespace cv;

class light
{
public:
    light(){};
    ~light(){};
    void setColor();//设置颜色:0 为蓝色，1为红色
    void myimgprocess(int &low1,int &low2);//图像预处理
    void myfindContours();//找到所有轮廓
    void recordLights(vector<vector<Point>> contours);//框出灯条的矩形
    void selectLights(vector<RotatedRect>boundRect);//筛选出竖直方向的轮廓近似为灯条
    void sortLights(vector<RotatedRect>boundRect);//对灯条进行排序，方便后续处理
    void matchLights(vector<RotatedRect> Rect_after_sort);//配对灯条进一步筛选
    void LightsPointsAfterSort();
    void LightsPointsAfterMatch();
    void myclear();//清理vector************************添加成员之后不要忘记清理***********************************************
public:
    int color=0;//0是蓝色，1是红色
    int low1 = 145;int low2 = 26;//用于图像处理部分二值化的阈值
    double min_area=20;//筛选灯条步骤的最小面积
    double max_center_angle=60;double max_sub_angle=30;//灯条配对步骤的最大中心点偏角以及最大左右灯条角度差
    String path = "/home/lian/task5/Task3Videos/drone_blue.avi";
    Mat img,mask,imgGray,after_process;
    vector<vector<Point>> contours;//用于记录检测到的轮廓，其中每一个向量储存着这个轮廓的若干点坐标
    vector<Vec4i> hierarchy;//用于记录拓扑信息
    vector<RotatedRect> boundRect;//记录最初灯条的矩形
    vector<RotatedRect> boundRect_after_select;//筛选灯条
    vector<RotatedRect> boundRect_after_sort;//排序后的灯条
    vector<vector<Point2f>> light_points;//记录排序后的灯条角点
    vector<RotatedRect> light_left;//记录配对后的左灯条
    vector<RotatedRect> light_right;//记录配对后的右灯条
    vector<vector<Point2f>> light_points_left;//记录配对后的左灯条角点
    vector<vector<Point2f>> light_points_right;//记录配对后的右灯条角点
    //左右灯条角点顺序均是右下起始，逆时针
};
class armor
{
    public:
    armor(){};
    ~armor(){};
    void recordPoints(Mat &img,vector<vector<Point2f>> light_points_left,vector<vector<Point2f>> light_points_right);//记录装甲板的四个角点
    void myDraw(Mat &img);//连线装甲板角点
    void myclear();//清理vector
    void selectArmor();
    void PnP(Mat &img);
    double max_sub_top_bottom=12;//装甲板上下两条线最大角度差
    double max_height_width_ratio=5;//装甲板宽高比最大值
    vector<vector<Point2f>> points;//装甲板的四个角点
    //角点顺序：右下起始，逆时针

};
class Kalman{
public:
    void initKalman(int max_armor_num);
    void useKalman(Mat &img,vector<vector<Point2f>> points);
    vector<KalmanFilter> kalman;
    int points_num;int each_state_dim;int each_measure_dim;
    int stateDim;int measureDim;
};



#endif //UNTITLED1_ARMOR_DETECT_H
