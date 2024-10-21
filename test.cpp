//#include<iostream>
//#include<opencv2/opencv.hpp>
//#include"serialport.h"
//#include"CRC_Check.h"
//using namespace std;
//using namespace cv;
//
//void myprint(vision_rx_info_t& visiondata){
//        cout<<"SOF : "<<visiondata.SOF<<' '<<"mode : "<<visiondata.mode<<' '<<"CRC8 : "<<visiondata.CRC8<<' '<<"pitch_angle : "<<visiondata.pitch_angle.f<<endl;
//    cout<<"yaw_angle : "<<visiondata.yaw_angle.f<<' '<<"is_find_target : "<<visiondata.is_find_target<<' '<<"armor_num : "<<visiondata.armor_num<<' '<<"is_find_buff : "<<visiondata.is_find_buff<<endl;
//    cout<<"is_hit_enable : "<<visiondata.is_hit_enable<<' '<<"shoot_type : "<<visiondata.shoot_type<<"is_spin : "<<visiondata.is_spin<<"CRC16 : "<<visiondata.CRC16<<endl;
//}
//int main()
//{
//    SerialPort sp("/dev/ttyUSB0");
//    bool a = sp.initSerialPort();
//    if(!a){cout<<"error"<<endl;}
//    vision_tx_info_t cardata;
//    vision_rx_info_t visiondata;
//    visiondata.reset();
//    //cout<<sizeof(visiondata)<<"**************"<<endl;
//    while(1)
//    {
//        sp.my_get_Mode(cardata);
//
//        cout<<(int)cardata.mode<<' '<<cardata.yaw_angle<<' '<<cardata.pitch_angle<<' '<<(int)cardata.my_color<<endl;
//
//        sp.my_transform_data(visiondata);
//        sp.send();
//        waitKey(1000);
//    }
//
//    //myprint(visiondata);
//
//    sp.closePort();
//    return 0;
//}
