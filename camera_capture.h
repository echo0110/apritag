#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <opencv2/opencv.hpp>  // 如果需要用到OpenCV类
#include <cstdio>  // 可能需要stdio.h
#include <camera.h>
#define CAMERA_WIDTH  1920
#define CAMERA_HEIGHT 1080
#define IMGRATIO     3
#define IMAGE_SIZE   (CAMERA_WIDTH*CAMERA_HEIGHT*IMGRATIO)


class CameraCapture {
private:
    char *pbuf;
    bool initialized;

public:
     CameraCapture() : pbuf(nullptr), initialized(false) {
        int retry_count = 3;
        while (retry_count-- && !initialized) {
            int ret = rgbcamera_init(CAMERA_WIDTH, CAMERA_HEIGHT, 270);
            if (ret == 0) {
                pbuf = (char *)malloc(IMAGE_SIZE);
                if (pbuf) {
                    // 跳过前几帧
                    int skip = 10;
                    while(skip--) {
                        ret = rgbcamera_getframe(pbuf);
                        usleep(10000);
                    }
                    initialized = true;
                    break;
                }
            }
            usleep(500000); // 失败后等待0.5秒再重试
        }
    }
    
    ~CameraCapture() {
        if (pbuf) {
            free(pbuf);
            pbuf = nullptr;
        }
        // 这里可以添加相机关闭的代码
    }
    
    inline  cv::Mat getFrame() {
        if (!initialized || !pbuf) {
            return cv::Mat();
        }
        
        int ret = rgbcamera_getframe(pbuf);
        if (ret) {
            printf("获取帧错误: %s, %d\n", __func__, __LINE__);
            return cv::Mat();
        }
        
        // 转换为Mat格式（BGR）
        cv::Mat color_image(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);
        memcpy(color_image.data, pbuf, IMAGE_SIZE);
        
        // 转换为灰度图
        cv::Mat gray_image;
        cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
        
        return gray_image;
    }
    
    bool isInitialized() const {
        return initialized;
    }
};

#endif  // CAMERA_CAPTURE_H