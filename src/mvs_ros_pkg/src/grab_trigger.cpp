#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
using namespace std;

unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
bool exit_flag = false;
image_transport::Publisher pub;
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
std::string CameraName;

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  int ExposureAuto = Params["ExposureAuto"];
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int GainAuto = Params["GainAuto"];
  int GammaSelector = Params["GammaSelector"];
  int nRet;
  nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", ExposureAuto);
  if (MV_OK == nRet) {
    std::string msg = "Set Exposure Auto: " + ExposureAutoStr[ExposureAuto];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Exposure auto mode");
  }
  nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
  if (MV_OK == nRet) {
    std::string msg =
        "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) + "ms";
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Exposure Time Lower");
  }
  nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
  if (MV_OK == nRet) {
    std::string msg =
        "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) + "ms";
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Exposure Time Upper");
  }
  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);
  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Gain auto mode");
  }
}

void PressEnterToExit(void) {
  int c;
  while ((c = getchar()) != '\n' && c != EOF)
    ;
  fprintf(stderr, "\nPress enter to exit.\n");
  while (getchar() != '\n')
    ;
  exit_flag = true;
  sleep(1);
}

static void *WorkThread(void *pUser) {
  int nRet = MV_OK;

  // ch:获取数据包大小 | en:Get payload size
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return NULL;
  }

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  unsigned char *pData =
      (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  if (NULL == pData)
    return NULL;

  unsigned int nDataSize = stParam.nCurValue;

  while (ros::ok()) {

    nRet =
        MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      ros::Time rcv_time = ros::Time::now();
      std::string debug_msg;
      debug_msg = CameraName + " GetOneFrame,nFrameNum[" +
                  std::to_string(stImageInfo.nFrameNum) + "], FrameTime:" +
                  std::to_string(rcv_time.toSec());
      ROS_INFO_STREAM(debug_msg.c_str());
      cv::Mat srcImage;
      srcImage =
          cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
      msg->header.stamp = rcv_time;
      pub.publish(msg);
    }

    if (exit_flag)
      break;
  }

  if (pData) {
    free(pData);
    pData = NULL;
  }

  return 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mvs_trigger");
  std::string params_file = std::string(argv[1]);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  int nRet = MV_OK;
  void *handle = NULL;
  ros::Rate loop_rate(10);
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];
  std::string camera_name = Params["CameraName"];
  CameraName = camera_name;
  pub = it.advertise(pub_topic, 1);

  while (ros::ok()) {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // 枚举检测到的相机数量
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
      printf("Enum Devices fail!");
      break;
    }

    bool find_expect_camera = false;
    int expect_camera_index = 0;
    if (stDeviceList.nDeviceNum == 0) {
      ROS_ERROR_STREAM("No Camera.\n");
      break;
    } else {
      // 根据serial number启动指定相机
      for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        std::string serial_number =
            std::string((char *)stDeviceList.pDeviceInfo[i]
                            ->SpecialInfo.stUsb3VInfo.chSerialNumber);
        if (expect_serial_number == serial_number) {
          find_expect_camera = true;
          expect_camera_index = i;
          break;
        }
      }
    }
    if (!find_expect_camera) {
      std::string msg =
          "Can not find the camera with serial number " + expect_serial_number;
      ROS_ERROR_STREAM(msg.c_str());
      break;
    }

    nRet = MV_CC_CreateHandle(&handle,
                              stDeviceList.pDeviceInfo[expect_camera_index]);
    if (MV_OK != nRet) {
      ROS_ERROR_STREAM("Create Handle fail");
      break;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
      printf("Open Device fail\n");
      break;
    }

    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
    if (MV_OK != nRet) {
      printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
      break;
    }

    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet) {
      printf("Get PayloadSize fail\n");
      break;
    }
    g_nPayloadSize = stParam.nCurValue;

    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
    if (nRet != MV_OK) {
      printf("Pixel setting can't work.");
      break;
    }

    setParams(handle, params_file);

    // 设置触发模式为on
    // set trigger mode as on
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
      break;
    }

    // 设置触发源
    // set trigger source
    nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
      break;
    }

    ROS_INFO("Finish all params set! Start grabbing...");
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
      printf("Start Grabbing fail.\n");
      break;
    }

    pthread_t nThreadID;
    nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
    if (nRet != 0) {
      printf("thread create failed.ret = %d\n", nRet);
      break;
    }

    PressEnterToExit();

    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
      break;
    }

    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
      break;
    }

    break;
  }
  return 0;
}