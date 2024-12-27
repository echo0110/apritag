#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <camera.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#define CAMERA_WIDTH  1920
#define CAMERA_HEIGHT 1080
#define IMGRATIO     3
#define IMAGE_SIZE   (CAMERA_WIDTH*CAMERA_HEIGHT*IMGRATIO)
#define SHM_NAME "/apriltag_camera_shm"

class CameraCapture {
private:
    bool initialized;
    void *shm_addr;        // 共享内存地址
    int shm_fd;            // 共享内存文件描述符
    const size_t FRAME_SIZE = CAMERA_WIDTH * CAMERA_HEIGHT * IMGRATIO;

public:
    CameraCapture() : initialized(false), shm_addr(nullptr), shm_fd(-1) {
        // 打开或创建共享内存对象
        shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            fprintf(stderr, "shm_open failed: %s\n", strerror(errno));
            return;
        }

        // 设置共享内存大小
        if (ftruncate(shm_fd, FRAME_SIZE) == -1) {
            fprintf(stderr, "ftruncate failed: %s\n", strerror(errno));
            close(shm_fd);
            return;
        }

        // 映射共享内存
        shm_addr = mmap(nullptr, FRAME_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shm_addr == MAP_FAILED) {
            fprintf(stderr, "mmap failed: %s\n", strerror(errno));
            close(shm_fd);
            return;
        }

        initialized = true;
    }
    
    ~CameraCapture() {
        if (shm_addr != nullptr && shm_addr != MAP_FAILED) {
            munmap(shm_addr, FRAME_SIZE);
        }
        if (shm_fd != -1) {
            close(shm_fd);
            shm_unlink(SHM_NAME);
        }
    }
    
    inline cv::Mat getFrame() {
        if (!initialized || !shm_addr) {
            return cv::Mat();
        }
        
        cv::Mat frame(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);
        memcpy(frame.data, shm_addr, FRAME_SIZE);
        
        return frame;
    }

    bool isInitialized() const {
        return initialized;
    }
};
#endif