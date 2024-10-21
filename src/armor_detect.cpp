#include"../include/armor_detect.h"
#include<iostream>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
void light::setColor()
{
    color = 0;
}
void light::myimgprocess(int &low1,int &low2) {
    //通道分离
    Mat imgchannels[3];
    after_process=img.clone();
    split(after_process,imgchannels);
    if(color == 0)
    {
        after_process = imgchannels[0] - imgchannels[2];
        threshold(imgchannels[0],imgGray,low1,255,THRESH_BINARY);
        threshold(after_process,mask,low2,255,THRESH_BINARY);
    }
    else if(color == 1)
    {
        after_process = imgchannels[2] - imgchannels[0];
        threshold(imgchannels[2],imgGray,low1,255,THRESH_BINARY);
        threshold(after_process,mask,low2,255,THRESH_BINARY);
    }
//    imshow("mask1",mask);
//    imshow("imgGray",imgGray);
    mask = imgGray & mask;//与操作

    Mat kernal = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(mask, mask, kernal);
    imshow("mask", mask);
}
void light::myfindContours()
{
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    //drawContours(img, contours, -1, Scalar(0, 255, 0),1);
}
void light::selectLights(vector<RotatedRect> boundRect)
{
//    for(int i=0;i<boundRect.size();i++)
//    {
//        vector<Point2f> box(4);
//        boundRect[i].points(box.data());
//        vector<Point> boxInt(4);
//        for(int j = 0;j<4;j++)
//        {
//            boxInt[j] = Point(static_cast<int>(box[j].x),static_cast<int>(box[j].y));
//        }
//        polylines(img, boxInt, true, Scalar(0, 255, 0),1);
//    }
    for(int i=0;i<boundRect.size();i++)
    {
        double area = boundRect[i].size.area();
        if (area>min_area)
        {
            boundRect_after_select.push_back(boundRect[i]);
        }
    }
//    for(int i=0;i<boundRect_after_select.size();i++)
//    {
//        vector<Point2f> box(4);
//        boundRect_after_select[i].points(box.data());
//        vector<Point> boxInt(4);
//        for(int j = 0;j<4;j++)
//        {
//            boxInt[j] = Point(static_cast<int>(box[j].x),static_cast<int>(box[j].y));
//        }
//        polylines(img, boxInt, true, Scalar(0, 255, 0),1);
//    }
}
void light::recordLights(vector<vector<Point>> contours)
{
    boundRect.resize(contours.size());
    for(int i=0;i<contours.size();i++)
    {
        boundRect[i] = minAreaRect(contours[i]);
    }
}
void light::sortLights(vector<RotatedRect> boundRect)
{
    for(int i=0;i<boundRect.size()-1;i++)
    {
        for(int j = 0;j<boundRect.size()-i-1;j++)
        {
            if(boundRect[j].center.x>boundRect[j+1].center.x)
            {
                RotatedRect temp = boundRect[j];
                boundRect[j] = boundRect[j+1];
                boundRect[j+1] = temp;
            }
        }
    }
    boundRect_after_sort = boundRect;
//    for(int i = 0;i<boundRect_after_sort.size();i++)
//    {
//        putText(img, to_string(i), boundRect_after_sort[i].tl(), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
//        rectangle(img, boundRect_after_sort[i], Scalar(0, 255, 0),1);
//    }
}
void light::matchLights(vector<RotatedRect> Rect_after_sort)
{
//    for(int i=0;i<Rect_after_sort.size();i++)
//    {
//        vector<Point2f> box(4);
//        Rect_after_sort[i].points(box.data());
//        vector<Point> boxInt(4);
//        for(int j = 0;j<4;j++)
//        {
//            boxInt[j] = Point(static_cast<int>(box[j].x),static_cast<int>(box[j].y));
//        }
//        polylines(img, boxInt, true, Scalar(0, 255, 0),1);
//    }
    for(int i =0;i<Rect_after_sort.size()-1;i++)
    {
        Point center1 = Rect_after_sort[i].center;
        for(int j = i+1;j<Rect_after_sort.size();j++)
        {
            Point center2 = Rect_after_sort[j].center;
            double center_angle = fabs(atan2(center2.y-center1.y,center2.x-center1.x)*180/CV_PI);
            double angle_left = atan2(light_points[i][0].y-light_points[i][1].y,light_points[i][0].x-light_points[i][1].x)*180/CV_PI;
            double angle_right = atan2(light_points[j][0].y-light_points[j][1].y,light_points[j][0].x-light_points[j][1].x)*180/CV_PI;
            double sub_angle = fabs(angle_left-angle_right);
            double extend_left = norm(light_points[i][0]-light_points[i][1])/3;
            double extend_right = norm(light_points[j][2]-light_points[j][3])/3;
            if(light_points[i][2].y>(light_points[j][0].y+extend_right)||(light_points[i][0].y+extend_left)<light_points[j][2].y)continue;
            if(sub_angle>max_sub_angle)continue;
            if(center_angle>max_center_angle)continue;
            light_left.push_back(Rect_after_sort[i]);
            light_right.push_back(Rect_after_sort[j]);
            break;
        }
    }
//    for(int i=0;i<light_left.size();i++)
//    {
//        vector<Point2f> box1(4);
//        light_left[i].points(box1.data());
//        vector<Point> boxInt1(4);
//        for(int j = 0;j<4;j++)
//        {
//            boxInt1[j] = Point(static_cast<int>(box1[j].x),static_cast<int>(box1[j].y));
//        }
//        polylines(img, boxInt1, true, Scalar(0, 255, 0),1);
//        vector<Point2f> box2(4);
//        light_right[i].points(box2.data());
//        vector<Point> boxInt2(4);
//        for(int j = 0;j<4;j++)
//        {
//            boxInt2[j] = Point(static_cast<int>(box2[j].x),static_cast<int>(box2[j].y));
//        }
//        polylines(img, boxInt2, true, Scalar(255, 0, 255),1);
//    }
}
void light::LightsPointsAfterSort()
{
    vector<vector<Point2f>> current(boundRect_after_sort.size());
    light_points.resize(boundRect_after_sort.size());
    //记录角点
    for(int i = 0;i<boundRect_after_sort.size();i++)
    {
        vector<Point2f> temp(4);
        boundRect_after_sort[i].points(temp.data());
        current[i] = temp;
    }
    //角点排序逆时针,从右下角开始
    for(int i = 0;i<boundRect_after_sort.size();i++)
    {
        vector<double> sum(4);
        vector<double> sub(4);
        for(int j =0;j<4;j++)
        {
            sum[j]= current[i][j].x+current[i][j].y;
            sub[j]= current[i][j].x-current[i][j].y;
        }
        light_points[i].push_back(current[i][max_element(sum.begin(),sum.end())-sum.begin()]);//右下角为第一个点
        light_points[i].push_back(current[i][max_element(sub.begin(),sub.end())-sub.begin()]);//右上角为第二个点
        light_points[i].push_back(current[i][min_element(sum.begin(),sum.end())-sum.begin()]);//左上角为第三个点
        light_points[i].push_back(current[i][min_element(sub.begin(),sub.end())-sub.begin()]);//左下角为第四个点
        double height = norm(light_points[i][0]-light_points[i][1]);
        double width = norm(light_points[i][1]-light_points[i][2]);
        if(height<width)
        {
            Point2f temp0 = light_points[i][0];
            Point2f temp1 = light_points[i][1];
            Point2f temp2 = light_points[i][2];
            Point2f temp3 = light_points[i][3];
            light_points[i][3] = temp0;
            light_points[i][2] = temp3;
            light_points[i][1] = temp2;
            light_points[i][0] = temp1;
        }
    }
    //debug显示部分
//    for(int i=0;i<light_points.size();i++)
//    {
//        vector<Point2f> temp(4);
//        temp = light_points[i];
//        for(int j = 0;j<4;j++)
//        {
//            circle(img, temp[j], 3, Scalar(0, 255, 0), -1);
//            putText(img, to_string(j), temp[j], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
//        }
//    }
}
void light::LightsPointsAfterMatch()
{
    vector<vector<Point2f>> currentleft(light_left.size());
    vector<vector<Point2f>> currentright(light_right.size());
    //记录角点
    for(int i = 0;i<light_left.size();i++)
    {
        vector<Point2f> temp(4);
        light_left[i].points(temp.data());
        currentleft[i] = temp;
    }
    for(int i = 0;i<light_right.size();i++)
    {
        vector<Point2f> temp(4);
        light_right[i].points(temp.data());
        currentright[i] = temp;
    }
    light_points_left.resize(light_left.size());
    light_points_right.resize(light_right.size());
    //角点排序逆时针,从右下角开始
    for(int i = 0;i<light_points_left.size();i++)
    {
       vector<double> sum(4);
       vector<double> sub(4);
       for(int j =0;j<4;j++)
       {
           sum[j]= currentleft[i][j].x+currentleft[i][j].y;
           sub[j]= currentleft[i][j].x-currentleft[i][j].y;
       }
       light_points_left[i].push_back(currentleft[i][max_element(sum.begin(),sum.end())-sum.begin()]);//右下角为第一个点
       light_points_left[i].push_back(currentleft[i][max_element(sub.begin(),sub.end())-sub.begin()]);//右上角为第二个点
       light_points_left[i].push_back(currentleft[i][min_element(sum.begin(),sum.end())-sum.begin()]);//左上角为第三个点
       light_points_left[i].push_back(currentleft[i][min_element(sub.begin(),sub.end())-sub.begin()]);//左下角为第四个点
       double height = norm(light_points_left[i][0]-light_points_left[i][1]);
       double width = norm(light_points_left[i][1]-light_points_left[i][2]);
       if(height<width)
       {
           Point2f temp0 = light_points_left[i][0];
           Point2f temp1 = light_points_left[i][1];
           Point2f temp2 = light_points_left[i][2];
           Point2f temp3 = light_points_left[i][3];
           light_points_left[i][3] = temp0;
           light_points_left[i][2] = temp3;
           light_points_left[i][1] = temp2;
           light_points_left[i][0] = temp1;
       }
    }
    for(int i = 0;i<light_points_right.size();i++)
    {
        vector<double> sum(4);
        vector<double> sub(4);
        for(int j =0;j<4;j++)
        {
            sum[j]= currentright[i][j].x+currentright[i][j].y;
            sub[j]= currentright[i][j].x-currentright[i][j].y;
        }
        light_points_right[i].push_back(currentright[i][max_element(sum.begin(),sum.end())-sum.begin()]);//右下角为第一个点
        light_points_right[i].push_back(currentright[i][max_element(sub.begin(),sub.end())-sub.begin()]);//右上角为第二个点
        light_points_right[i].push_back(currentright[i][min_element(sum.begin(),sum.end())-sum.begin()]);//左上角为第三个点
        light_points_right[i].push_back(currentright[i][min_element(sub.begin(),sub.end())-sub.begin()]);//左下角为第四个点
        double height = norm(light_points_right[i][0]-light_points_right[i][1]);
        double width = norm(light_points_right[i][1]-light_points_right[i][2]);
        if(height<width)
        {
            Point2f temp0 = light_points_right[i][0];
            Point2f temp1 = light_points_right[i][1];
            Point2f temp2 = light_points_right[i][2];
            Point2f temp3 = light_points_right[i][3];
            light_points_right[i][3] = temp0;
            light_points_right[i][2] = temp3;
            light_points_right[i][1] = temp2;
            light_points_right[i][0] = temp1;
        }
    }
    //debug显示部分
//    for(int i=0;i<light_points_left.size();i++)
//    {
//        vector<Point2f> temp(4);
//        temp = light_points_left[i];
//        for(int j = 0;j<4;j++)
//        {
//            circle(img, temp[j], 3, Scalar(0, 255, 0), -1);
//            putText(img, to_string(j), temp[j], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
//        }
//        temp = light_points_right[i];
//        for(int j = 0;j<4;j++)
//       {
//            circle(img, temp[j], 3, Scalar(255, 0, 255), -1);
//           putText(img, to_string(j), temp[j], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
//       }
//    }
}
void light::myclear()
{
    light_left.clear();
    light_right.clear();
    light_points_left.clear();
    light_points_right.clear();
    light_points.clear();
    contours.clear();
    hierarchy.clear();
    boundRect.clear();
    boundRect_after_select.clear();
    boundRect_after_sort.clear();
}
void armor::recordPoints(Mat &img,vector<vector<Point2f>> light_points_left,vector<vector<Point2f>> light_points_right) {
    if(!light_points_left.empty()&&!light_points_right.empty())
    {
        points.resize(light_points_left.size());
        for(int i = 0;i<light_points_left.size();i++)
        {
            Point2f tl;
            tl.x=(light_points_left[i][1].x+light_points_left[i][2].x)/2;
            tl.y=(light_points_left[i][1].y+light_points_left[i][2].y)/2;
            Point2f tr;
            tr.x=(light_points_right[i][1].x+light_points_right[i][2].x)/2;
            tr.y=(light_points_right[i][1].y+light_points_right[i][2].y)/2;
            Point2f bl;
            bl.x=(light_points_left[i][0].x+light_points_left[i][3].x)/2;
            bl.y=(light_points_left[i][0].y+light_points_left[i][3].y)/2;
            Point2f br;
            br.x=(light_points_right[i][0].x+light_points_right[i][3].x)/2;
            br.y=(light_points_right[i][0].y+light_points_right[i][3].y)/2;
            points[i].push_back(br);
            points[i].push_back(tr);
            points[i].push_back(tl);
            points[i].push_back(bl);
        }
    }
}
void armor::myDraw(Mat &img)
{
    if(!points.empty())
    {
        for(int i = 0;i<points.size();i++)
        {
            if(!points[i].empty())
            {
                line(img, points[i][0], points[i][1], Scalar(0, 0, 255), 1);
                line(img, points[i][1], points[i][2], Scalar(0, 0, 255), 1);
                line(img, points[i][2], points[i][3], Scalar(0, 0, 255), 1);
                line(img, points[i][3], points[i][0], Scalar(0, 0, 255), 1);
//                circle(img, points[i][0], 3, Scalar(0, 255, 0), -1);
//                putText(img, to_string(0), points[i][0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255), 1);
//                circle(img, points[i][1], 3, Scalar(0, 255, 0), -1);
//                putText(img, to_string(1), points[i][1], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255), 1);
//                circle(img, points[i][2], 3, Scalar(0, 255, 0), -1);
//                putText(img, to_string(2), points[i][2], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255), 1);
//                circle(img, points[i][3], 3, Scalar(0, 255, 0), -1);
//                putText(img, to_string(3), points[i][3], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255), 1);
                Point2f center((points[i][0].x+points[i][1].x+points[i][2].x+points[i][3].x)/4,(points[i][0].y+points[i][1].y+points[i][2].y+points[i][3].y)/4);
                circle(img,center,3,Scalar(255,0,255),-1);
            }
        }
    }
}
void armor::selectArmor()
{
    vector<vector<Point2f>> current;
    for(int i =0;i<points.size();i++)
    {
        double angle_top = atan2(points[i][1].y-points[i][2].y,points[i][1].x-points[i][2].x)*180/CV_PI;
        double angle_bottom = atan2(points[i][0].y-points[i][3].y,points[i][0].x-points[i][3].x)*180/CV_PI;
        if(fabs(angle_bottom-angle_top)>max_sub_top_bottom)continue;
        double height1 = norm(points[i][0] - points[i][1]);
        double height2 = norm(points[i][2]-points[i][3]);
        double width = norm(points[i][1]-points[i][2]);
        if(width/height1>max_height_width_ratio||width/height2>max_height_width_ratio)
        {
            continue;
        }
        current.push_back(points[i]);
    }
    points=current;
    if(points.size()<2)return;
    for(int i = 0;i<points.size()-1;i++)
    {
        for(int j = i+1;j<points.size();j++)
        {
            if(points[i][0]==points[j][0]||points[i][3]==points[j][3])
            {
                double angle1 = fabs(atan2(points[i][1].y-points[i][2].y,points[i][1].x-points[i][2].x)*180/CV_PI);
                double angle2 = fabs(atan2(points[j][1].y-points[j][2].y,points[j][1].x-points[j][2].x)*180/CV_PI);
                if(angle1>angle2){
                    points[i].clear();
                }
                else points[j].clear();
            }
        }
    }
}
void armor::PnP(Mat &img)
{
    vector<double> camera_params={2.393262145695329e+03,0,7.554313585773901e+02,
                                  0,2.394045294762181e+03,6.196921100215907e+02,
                                  0,0,1};
    Mat cameraMatrix = Mat(3, 3, CV_64F, camera_params.data());

    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);//相机畸变参数，默认为0

    vector<Point3f> objectPoints;//世界坐标系
    objectPoints.push_back(cv::Point3f(64.0f, 25.0f, 0.0f)); // 右下角点
    objectPoints.push_back(cv::Point3f(64.0f, -25.0f, 0.0f)); // 右上角点
    objectPoints.push_back(cv::Point3f(-64.0f, -25.0f, 0.0f)); // 左上角点
    objectPoints.push_back(cv::Point3f(-64.0f, 25.0f, 0.0f)); // 左下角点
    for(int i = 0;i<points.size();i++)
    {
        if(!points[i].empty())
        {
            //图像坐标
            vector<Point2f> imagePoints;
            imagePoints.push_back(points[i][0]);//右下
            imagePoints.push_back(points[i][1]);//右上
            imagePoints.push_back(points[i][2]);//左上
            imagePoints.push_back(points[i][3]);//左下

            Mat rvec, tvec;
            solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
            //测距
            double distance = norm(tvec);
            putText(img, "distance : "+to_string(distance),Point(50,250), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 255), 1);
            //测欧拉角(XYZ顺序)
            Mat rotationMatrix;
            Rodrigues(rvec, rotationMatrix);
            double yaw,pitch,roll;
            double sy = sqrt(rotationMatrix.at<double>(0, 2) * rotationMatrix.at<double>(0, 2) + rotationMatrix.at<double>(1, 2) * rotationMatrix.at<double>(1, 2));
            bool singular = sy < 1e-6; // 如果接近奇异点

            if (!singular) {
                yaw = atan2(rotationMatrix.at<double>(1, 2), rotationMatrix.at<double>(0, 2));
                pitch = atan2(-rotationMatrix.at<double>(2, 2), sy);
                roll = atan2(rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 0));
            } else {
                yaw = atan2(-rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(1, 1));
                pitch = atan2(-rotationMatrix.at<double>(2, 2), sy);
                roll = 0;
            }

            putText(img, "yaw : "+to_string(yaw*180/CV_PI),Point(50,100), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 255), 1);
            putText(img, "pitch : "+to_string(pitch*180/CV_PI),Point(50,150), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 255), 1);
            putText(img, "roll : "+to_string(roll*180/CV_PI),Point(50,200), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 255), 1);

            //重投影坐标到图像上
            // 定义坐标轴端点
            vector<cv::Point3f> axisPoints;
            axisPoints.push_back(cv::Point3f(0.0f, 0.0f, 0.0f)); // 原点
            axisPoints.push_back(cv::Point3f(100.0f, 0.0f, 0.0f)); // X轴方向上的点
            axisPoints.push_back(cv::Point3f(0.0f, 100.0f, 0.0f)); // Y轴方向上的点
            axisPoints.push_back(cv::Point3f(0.0f, 0.0f, 100.0f)); // Z轴方向上的点

            // 投影坐标轴端点到图像平面上
            vector<cv::Point2f> imgPoints;
            projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imgPoints);
            // 绘制原点
            circle(img, imgPoints[0], 5, cv::Scalar(0, 0, 255), -1);

            // 绘制X轴红色线
            line(img, imgPoints[0], imgPoints[1], cv::Scalar(0, 0, 255), 3);

            // 绘制Y轴绿色线
            line(img, imgPoints[0], imgPoints[2], cv::Scalar(0, 255, 0), 3);

            // 绘制Z轴蓝色线
            line(img, imgPoints[0], imgPoints[3], cv::Scalar(255, 0, 0), 3);
        }
    }
}
void Kalman::initKalman(int max_armor_num)
{
    points_num = 1;
    each_state_dim = 4;//每个状态点四维(x,y,vx,vy)
    each_measure_dim = 2;//每个测量点两维(x,y)
    stateDim = points_num*each_state_dim;
    measureDim = points_num*each_measure_dim;
    kalman.resize(max_armor_num);
    float dt = 10000;
    //设置状态转移矩阵
    Mat transitionMatrix = Mat::eye(stateDim,stateDim,CV_32F);
    for(int j = 0;j<points_num;j++)
    {
        transitionMatrix.at<float>(j*each_state_dim,j*each_state_dim+2)=dt;
        transitionMatrix.at<float>(j*each_state_dim+1,j*each_state_dim+3)=dt;
    }
    //设置测量矩阵
    Mat measureMatrix = (Mat_<float>(measureDim,stateDim)<<1,0,0,0,
            0,1,0,0);
    //设置过程噪声协方差矩阵
    Mat processNoiseCov = Mat::eye(stateDim,stateDim,CV_32F)*10000;
    //设置测量噪声协方差矩阵
    Mat measurementNoiseCov = Mat::eye(measureDim,measureDim,CV_32F)*1;
    //设置状态误差协方差矩阵
    Mat estimatedErrorCov = Mat::eye(stateDim,stateDim,CV_32F)*10;
    //初始化状态矩阵
    Mat state = Mat::zeros(stateDim,1,CV_32F);
    for(int i = 0;i<kalman.size();i++)
    {
        kalman[i].init(stateDim,measureDim);
        kalman[i].transitionMatrix = transitionMatrix;
        kalman[i].measurementMatrix = measureMatrix;
        kalman[i].processNoiseCov = processNoiseCov;
        kalman[i].measurementNoiseCov = measurementNoiseCov;
        kalman[i].errorCovPre = estimatedErrorCov;
        kalman[i].statePre = state;
    }
}
void Kalman::useKalman(cv::Mat &img,vector<vector<Point2f>> points)
{
    if(points.empty())return;
    for(int i = 0;i<points.size();i++)
    {
        if(points[i].empty())continue;
        //创建观测向量
        vector<float> measurevector(measureDim);
        measurevector[0] = (points[i][0].x+points[i][1].x+points[i][2].x+points[i][3].x)/4;
        measurevector[1] = (points[i][0].y+points[i][1].y+points[i][2].y+points[i][3].y)/4;
        Mat measurement(measureDim,1,CV_32F,measurevector.data());
        //进行预测
        Mat prediction = kalman[i].predict();
        circle(img,Point2f(prediction.at<float>(0),prediction.at<float>(1)),3,Scalar(0,255,0),-1);
        Mat correct = kalman[i].correct(measurement);
        //circle(img,Point2f(correct.at<float>(0),correct.at<float>(1)),2,Scalar(0,0,255),-1);
    }
}

void armor::myclear()
{
    points.clear();
}