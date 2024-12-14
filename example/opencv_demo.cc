/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <iostream>
#include <iomanip>

#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
#include "tag36h10.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
}
#include "apriltag_pose.h"
/*rv1126 begin*/
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include <camera.h>
#include <string>
#define CAMERA_WIDTH	1920
#define CAMERA_HEIGHT	1080
#define	IMGRATIO		3
#define	IMAGE_SIZE		(CAMERA_WIDTH*CAMERA_HEIGHT*IMGRATIO)

//=====================  RTSP  =====================
#include "rtspServer/rtspServer.h"
#include "enCoder/enCoder.h"

// Global RTSP configuration
const char* DEFAULT_RTSP_URL = "rtsp://admin:admin@192.168.1.108:554/stream1";
bool use_rtsp = false;
std::string rtsp_url;

// RTSP streaming configuration
#define RTSP_PORT "8554"
#define RTSP_STREAM_NAME "apriltag"
static bool rtsp_initialized = false;

// Function to initialize RTSP server
bool init_rtsp_server() {
    if (rtsp_initialized) return true;
    
    // Initialize encoder
    if (enCoderInit("apriltag") != 0) {
        std::cerr << "Failed to initialize encoder" << std::endl;
        return false;
    }
    
    // Initialize RTSP server
    if (rtspServerInit(RTSP_PORT) != 0) {
        std::cerr << "Failed to initialize RTSP server" <<std::endl;
        return false;
    }
    
    rtsp_initialized = true;
    std::cout << "RTSP server initialized on port " << RTSP_PORT << std::endl;
    std::cout << "Stream URL: rtsp://[your-ip]:" << RTSP_PORT << "/" << RTSP_STREAM_NAME << std::endl;
    return true;
}

// Function to stream frame via RTSP
void stream_frame(const cv::Mat& frame) {
    if (!rtsp_initialized) {
        if (!init_rtsp_server()) {
            return;
        }
    }
    
    // Convert frame to required format if necessary
    cv::Mat stream_frame;
    if (frame.channels() == 1) {
        cv::cvtColor(frame, stream_frame, cv::COLOR_GRAY2BGR);
    } else {
        stream_frame = frame;
    }
    
    // Send frame to RTSP server
    // Note: You might need to adjust the format and parameters based on your RTSP server implementation
    //rtspServerPushFrame((char*)stream_frame.data, stream_frame.total() * stream_frame.elemSize());
}

/*end*/
using namespace std;
using namespace cv;

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;
/**
 * 定义了角度归一化函数，角度范围统一输出范围是[-pi,pi].
 **/
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}


cv::Mat acquire_image()
{
    static cv::VideoCapture cap;
    static bool initialized = false;
    cv::Mat frame;

    if (!initialized) {
        if (use_rtsp) {
            // Open RTSP stream
            cap.open(rtsp_url);
            if (!cap.isOpened()) {
                cerr << "Failed to open RTSP stream: " << rtsp_url << endl;
                return cv::Mat();
            }
        } else {
            // Use original camera initialization
            int ret = ircamera_init(CAMERA_WIDTH, CAMERA_HEIGHT, 270);
            if (ret != 0) {
                cerr << "Failed to initialize camera" << endl;
                return cv::Mat();
            }
        }
        initialized = true;
    }

    if (use_rtsp) {
        // Get frame from RTSP stream
        if (!cap.read(frame)) {
            cerr << "Failed to read frame from RTSP stream" << endl;
            return cv::Mat();
        }
        return frame;
    } else {
        // Original camera code
        char *pbuf = (char *)malloc(IMAGE_SIZE);
        if (!pbuf) {
            fprintf(stderr, "error: memory allocation failed: %s, %d\n", __func__, __LINE__);
            return cv::Mat();
        }

        int ret = rgbcamera_getframe(pbuf);
        if (ret) {
            free(pbuf);
            return cv::Mat();
        }

        cv::Mat color_image(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3, pbuf);
        cv::Mat result = color_image.clone();
        free(pbuf);
        return result;
    }
}


int main(int argc, char *argv[])
{
    printf("**********************************99999999999999999999999999");

    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_int(getopt, 'c', "camera", "0", "camera ID");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, 'r', "rtsp", 0, "Use RTSP stream instead of local camera");
    getopt_add_string(getopt, 'u', "url", DEFAULT_RTSP_URL, "RTSP URL (used only with -r)");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    cout << "Enabling video capture" << endl;

    TickMeter meter;
    meter.start();

    // Configure RTSP if requested
    use_rtsp = getopt_get_bool(getopt, "rtsp");
    if (use_rtsp) {
        rtsp_url = getopt_get_string(getopt, "url");
        cout << "Using RTSP stream from: " << rtsp_url << endl;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h10")) {
        tf = tag36h10_create();
    } else if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    printf("func is %s,%d,%s\n",__func__,__LINE__,"##############");
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    if (errno == ENOMEM) {
        printf("Unable to add family to detector due to insufficient memory to allocate the tag-family decoder with the default maximum hamming value of 2. Try choosing an alternative tag family.\n");
        exit(-1);
    }

    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

    meter.stop();
    cout << "Detector " << famname << " initialized in "
        << std::fixed << std::setprecision(3) << meter.getTimeSec() << " seconds" << endl;
#if CV_MAJOR_VERSION > 3
    // cout << "  " << cap.get(CAP_PROP_FRAME_WIDTH ) << "x" <<
    //                 cap.get(CAP_PROP_FRAME_HEIGHT ) << " @" <<
    //                 cap.get(CAP_PROP_FPS) << "FPS" << endl;
#else
    cout << "  " << cap.get(CV_CAP_PROP_FRAME_WIDTH ) << "x" <<
                    cap.get(CV_CAP_PROP_FRAME_HEIGHT ) << " @" <<
                    cap.get(CV_CAP_PROP_FPS) << "FPS" << endl;
#endif
    meter.reset();

    /***********输入标定的相机参数*************/
    apriltag_detection_info_t info;     // parameters of the camera calibrations 在这里把标定得到的四元参数输入到程序里
    info.tagsize = 0.056; //标识的实际尺寸
    info.fx = 652.894;
    info.fy = 651.487;
    info.cx = 301.857;
    info.cy = 237.548;

    cv::Mat gray_image,gray;
    gray = acquire_image();  
   
    printf("func is %s,%d,%s\n",__func__,__LINE__,"##############");
    while (1) {
        errno = 0;
        // gray = cv::Mat(rgb_image.rows, rgb_image.cols, CV_8UC1);
        // printf("func is %s,%d,%s\n",__func__,__LINE__,"##############");
        // cvtColor(rgb_image, gray, COLOR_BGR2GRAY);
        // printf("func is %s,%d,%s\n",__func__,__LINE__,"##############");
        // Make an image_u8_t header for the Mat data
        image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

        zarray_t *detections = apriltag_detector_detect(td, &im);

        if (errno == EAGAIN) {
            printf("Unable to create the %d threads requested.\n",td->nthreads);
            exit(-1);
        }

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            /* 加入位姿估计函数 */
            info.det = det;  // det --> info
            apriltag_pose_t pose;
            estimate_tag_pose(&info, &pose);
            // double err = estimate_tag_pose(&info, &pose);

            /* 将欧拉角转换成度为坐标，并归一化到[-pi,pi]范围内 */
            double yaw = 180 * standardRad(atan2(pose.R->data[3], pose.R->data[0]));
            double pitch = 180 * standardRad(asin(pose.R->data[6]));
            double roll = 180 * standardRad(atan2(pose.R->data[7], pose.R->data[8]));

            yaw = yaw / PI;
            pitch = pitch / PI;
            roll = roll / PI;

            /* 输出三维位置坐标信息 */
            cout << "THE 3D POSE: "
                << "x = " << pose.t->data[0] << ", "
                << "y = " << pose.t->data[1] << ", "
                << "z = " << pose.t->data[2] << endl;

            /* 输出3个欧拉角数据 */
            cout << "yaw: " << yaw << "'" << endl;
            cout << "pitch: " << pitch << "'" << endl;
            cout << "roll: " << roll << "'" << endl;


            line(gray, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(gray, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(gray, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(gray, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(gray, text, Point(det->c[0]-textsize.width/2,
                                           det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        }
        apriltag_detections_destroy(detections);
        printf("**********************************2222");
        // Instead of imshow and imwrite, stream the frame via RTSP
        stream_frame(gray);
    }

    apriltag_detector_destroy(td);
    if (!strcmp(famname, "tag36h10")) {
        tag36h10_destroy(tf);
    } else if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tf);
    } else if (!strcmp(famname, "tag16h5")) {
        tag16h5_destroy(tf);
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tagCircle21h7_destroy(tf);
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tagCircle49h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tagStandard41h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tagStandard52h13_destroy(tf);
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tagCustom48h12_destroy(tf);
    }


    getopt_destroy(getopt);

    return 0;
}
