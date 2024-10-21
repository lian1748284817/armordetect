#include"MvCameraControl.h"
#include<iostream>
#include <opencv2/opencv.hpp>
#include<cstring>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <unistd.h> // For usleep()
#include"include/armor_detect.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
using namespace std;
using namespace cv;

queue<Mat> q;
mutex mu;
condition_variable cond;
light l;
armor a;
Kalman k;
void producer(void *handle,int nRet)
{
    std::chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    unsigned int nDataSize = stParam.nCurValue;
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    Mat M;
    int i = 0;
    while(1)
    {
        t1 = std::chrono::steady_clock::now();
        pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue * 3);

        nRet = MV_CC_GetImageForBGR(handle, pData, nDataSize, &stImageInfo, 1000);
        if (nRet != MV_OK) {
            printf("MV_CC_GetOneFrameTimeout failed! nRet [%x]\n", nRet);
            break;
        }
        M = Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData).clone();

        {unique_lock<mutex> lock(mu);
        q.push(M);
        cond.notify_one();
        free(pData);}

        int c = waitKey(1);
        if(c==27)
        {
            imwrite("/home/lian/CLionProjects/armorDetect/img/img"+to_string(i)+".jpg",M);
            i++;
        }
        t2 = std::chrono::steady_clock::now();
        //cout<<"FPS1:"<<std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()<<endl;
    }
}
void consumer()
{
    std::chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    Mat M;
    k.initKalman(20);
    while(1)
    {
        t1 = std::chrono::steady_clock::now();

        {
            unique_lock<mutex> lock(mu);
            while(q.empty())
                cond.wait(lock);
            M = q.front();
            q.pop();
        }

        l.img = M.clone();
        l.setColor();//设置颜色，0为蓝色，1为红色
        l.myimgprocess(l.low1,l.low2);//图像预处理得到二值化图像
        l.myfindContours();//寻找轮廓
        l.recordLights(l.contours);//记录轮廓矩形
        l.selectLights(l.boundRect);//筛选竖向轮廓为灯条
        if(!l.boundRect_after_select.empty())
        {
            l.sortLights(l.boundRect_after_select);//灯条排序
            l.LightsPointsAfterSort();
            l.matchLights(l.boundRect_after_sort);//灯条配对
            l.LightsPointsAfterMatch();
        }
        a.recordPoints(l.img,l.light_points_left,l.light_points_right);//记录装甲板四个角点
        a.selectArmor();
        a.myDraw(l.img);//绘制装甲板
        //a.PnP(l.img);//测距和测欧拉角并显示
        k.useKalman(l.img,a.points);
        imshow("img", l.img);
        l.myclear();
        a.myclear();

        t2 = std::chrono::steady_clock::now();
        //cout<<"FPS2:"<<std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()<<endl;
    }

}
int main()
{
    int nRet = -1;
    void *handle = nullptr;

    //枚举设备
    unsigned int type = MV_GIGE_DEVICE | MV_USB_DEVICE;
    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    nRet = MV_CC_EnumDevices(type,&m_stDevList);
    if(nRet!=MV_OK)
    {
        cout<<"枚举设备失败"<<endl;
    }
    if(m_stDevList.nDeviceNum==0)
    {
        cout<<"没有设备"<<endl;
    }
    //选择查找到的第一台在线设备创建句柄
    int nDeviceIndex = 0;
    MV_CC_DEVICE_INFO m_stDevInfo={0};
    memcpy(&m_stDevInfo,m_stDevList.pDeviceInfo[nDeviceIndex],sizeof(MV_CC_DEVICE_INFO));
    nRet = MV_CC_CreateHandle(&handle,m_stDevList.pDeviceInfo[nDeviceIndex]);
    if(nRet!=MV_OK)
    {
        cout<<"创建句柄失败"<<endl;
    }


    //连接设备

    nRet = MV_CC_OpenDevice(handle);
    if(nRet!=MV_OK)
    {
        cout<<"连接设备失败"<<endl;
    }
    //初始化像素格式
    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PixelType_Gvsp_BGR8_Packed);
    if(nRet!=MV_OK)
    {
        cout<<"初始化像素格式失败"<<endl;
    }

    //初始化曝光时间
    nRet = MV_CC_SetFloatValue(handle,"ExposureTime",100000);
    if (MV_OK != nRet)
    {
        cout << "MV_CC_SetEnumValue fail! nRet [0x" << hex << nRet << "]" << endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return -1;
    }

    //grabing image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
    }

    thread pro(producer,handle,nRet);
    thread con(consumer);
    con.join();
    pro.join();

    //停止取流
    nRet = MV_CC_StopGrabbing(handle);
    if(nRet!=MV_OK)
    {
        cout<<"停止取流失败"<<endl;
    }

    //关闭设备
    nRet = MV_CC_CloseDevice(handle);
    if(nRet!=MV_OK)
    {
        cout<<"关闭设备失败"<<endl;
    }

    //销毁句柄
    nRet = MV_CC_DestroyHandle(handle);
    if(nRet!=MV_OK)
    {
        cout<<"销毁句柄失败"<<endl;
    }
    cv::Mat img;
    return 0;
}