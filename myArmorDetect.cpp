//#include<iostream>
//#include<opencv2/opencv.hpp>
//#include<opencv2/imgproc.hpp>
//#include<vector>
//#include"include/armor_detect.h"
//using namespace std;
//using namespace cv;
//
//light l;
//armor a;
//
//void myCreateTrackbar()
//{
//    namedWindow("Trackbars", WINDOW_NORMAL);
//    createTrackbar("low1", "Trackbars", &l.low1, 255);
//    createTrackbar("low2", "Trackbars", &l.low2, 255);
//}
//int main()
//{
//    Kalman k;
//    k.initKalman(20);
//    //myCreateTrackbar();
//    VideoCapture cap(l.path);
//    while(true)
//    {
//        cap>>l.img;
//        l.setColor();//设置颜色，0为蓝色，1为红色
//        l.myimgprocess(l.low1,l.low2);//图像预处理得到二值化图像
//        l.myfindContours();//寻找轮廓
//        l.recordLights(l.contours);//记录轮廓矩形
//        l.selectLights(l.boundRect);//筛选竖向轮廓为灯条
//        if(!l.boundRect_after_select.empty())
//        {
//            l.sortLights(l.boundRect_after_select);//灯条排序
//            l.LightsPointsAfterSort();
//            l.matchLights(l.boundRect_after_sort);//灯条配对
//            l.LightsPointsAfterMatch();
//        }
//        a.recordPoints(l.img,l.light_points_left,l.light_points_right);//记录装甲板四个角点
//        a.selectArmor();
//        a.myDraw(l.img);//绘制装甲板
//        k.useKalman(l.img,a.points);
//        imshow("img", l.img);
//        l.myclear();
//        a.myclear();
//        int c = waitKey(0);
//        if(c==27)
//        {
//            continue;
//            destroyAllWindows();
//            break;
//        }
//
//    }
//    return 0;
//}
