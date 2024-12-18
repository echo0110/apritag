#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <opencv2/opencv.hpp>
#include <camera.h>

class CameraCapture {
private:
    char *pbuf;
    bool initialized;
    
public:
    CameraCapture();
    ~CameraCapture();
    cv::Mat getFrame();
    bool isInitialized() const;
};

#endif // CAMERA_CAPTURE_H