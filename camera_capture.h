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
    bool initialized;

public:
    CameraCapture() : initialized(false) {
        // 不再进行摄像头初始化，因为已经在编码器进程中完成
        initialized = true;
    }
    
    ~CameraCapture() {
        // 不再需要释放摄像头，由编码器进程管理
    }
    
    inline cv::Mat getFrame() {
        if (!initialized) {
            return cv::Mat();
        }
        
        // 从共享内存或其他IPC机制获取图像数据
        // 这里直接返回空Mat，因为实际图像数据已经在编码器进程中处理并推流
        return cv::Mat();
    }
    
    bool isInitialized() const {
        return initialized;
    }
};
#endif  // CAMERA_CAPTURE_H