#include "camera_capture.h"
#include <stdio.h>
#include <stdlib.h>

// class CameraCapture {
// private:
//     char *pbuf;
//     bool initialized;
    
// public:
//     CameraCapture() : pbuf(nullptr), initialized(false) {
//         // 在构造函数中进行相机初始化
//         int ret = ircamera_init(CAMERA_WIDTH, CAMERA_HEIGHT, 270);
//         if (ret != 0) {
//             fprintf(stderr, "相机初始化失败\n");
//             return;
//         }
        
//         pbuf = (char *)malloc(IMAGE_SIZE);
//         if (!pbuf) {
//             fprintf(stderr, "内存分配失败\n");
//             return;
//         }
        
//         // 跳过前10帧
//         int skip = 10;
//         while(skip--) {
//             ret = rgbcamera_getframe(pbuf);
//             if (ret) {
//                 printf("跳帧错误: %s, %d\n", __func__, __LINE__);
//             }
//         }
        
//         initialized = true;
//     }
    
//     ~CameraCapture() {
//         if (pbuf) {
//             free(pbuf);
//             pbuf = nullptr;
//         }
//         // 这里可以添加相机关闭的代码
//     }
    
//     cv::Mat getFrame() {
//         if (!initialized || !pbuf) {
//             return cv::Mat();
//         }
        
//         int ret = rgbcamera_getframe(pbuf);
//         if (ret) {
//             printf("获取帧错误: %s, %d\n", __func__, __LINE__);
//             return cv::Mat();
//         }
        
//         // 转换为Mat格式（BGR）
//         cv::Mat color_image(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);
//         memcpy(color_image.data, pbuf, IMAGE_SIZE);
        
//         // 转换为灰度图
//         cv::Mat gray_image;
//         cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
        
//         return gray_image;
//     }
    
//     bool isInitialized() const {
//         return initialized;
//     }
// };
