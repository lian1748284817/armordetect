//#include<iostream>
//#include<opencv2/opencv.hpp>
//#include<opencv2/imgproc.hpp>
//#include<vector>
//using namespace std;
//using namespace cv;
//
//String path = "/home/lian/task5/Task3Videos/drone_blue.avi";
//Mat imgHSV,mask,imgCanny,imgdilate;
//vector<vector<int>> hsv={};
////hmax,him,smax,smin,vmax,vmin
//int hmin = 0;int hmax=179;int smin=0;int smax=255;int vmin=0;int vmax=255;
//
//
//void myColorDetect()
//{
//    namedWindow("Trackbars", (640, 200));
//    createTrackbar("Hue Min", "Trackbars", &hmin, 179);
//    createTrackbar("Hue Max", "Trackbars", &hmax, 179);
//    createTrackbar("Sat Min", "Trackbars", &smin, 255);
//    createTrackbar("Sat Max", "Trackbars", &smax, 255);
//    createTrackbar("Val Min", "Trackbars", &vmin, 255);
//    createTrackbar("Val Max", "Trackbars", &vmax, 255);
//}
//int main()
//{
//    VideoCapture cap(path);
//    Mat img;
//    myColorDetect();
//    while(true)
//    {
//        cap>>img;
//        cvtColor(img, imgHSV, COLOR_BGR2HSV);
//
//
//        Scalar lower(hmin, smin, vmin);
//        Scalar upper(hmax, smax, vmax);
//        inRange(imgHSV, lower, upper, mask);
//        cout<<hmin<<' '<<hmax<<' '<<smin<<' '<<smax<<' '<<vmin<<' '<<vmax<<endl;
//
//        imshow("img", img);
//        imshow("mask", mask);
//        int c=waitKey(1);
//        if(c==27)
//        {
//            break;
//        }
//    }
//
//
//
//    return 0;
//}
//
