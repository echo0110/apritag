/**
*
* Copyright 2022 by Guangzhou Easy EAI Technologny Co.,Ltd.
* website: www.easy-eai.com
*
* Author: Jiehao.Zhong <zhongjiehao@easy-eai.com>
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* License file for more details.
* 
*/

#include "system.h"

#include "rtspServer.h"
#include "enCoder/enCoder.h"
#define PROCESS_ENCODER_NAME   "enCoder"
#define PROCESS_RTSPSERVER_NAME "rtspServer"

void VideoStreamConnect(void *pCustomData)
{    printf("New RTSP connection request received\n");
    int *queueChnId = (int *)pCustomData;

    flush_video_channel(*queueChnId);
    
}
int32_t VideoStreamDataIn(void *pCustomData, RTSPVideoDesc_t *pDesc, uint8_t *pData)
{
    int ret = -1;
    
    int *queueChnId = (int *)pCustomData;
    
    VideoNodeDesc node;
    memset(&node, 0, sizeof(node));
    ret = get_node_from_video_channel(*queueChnId, &node, pData);
    if(0 != ret){
        return ret;
    }
    pDesc->frameType  = node.bySubType;
    pDesc->frameIndex = node.dwFrameIndex;
    pDesc->dataLen    = node.dwDataLen;
    pDesc->timeStamp  = node.ddwTimeStamp;
    printf("Sending video frame, size: %d\n", pDesc->dataLen);
    
    return 0;
}

//FILE *fp_audio_output = NULL;
void AudioStreamConnect(void *pCustomData)
{
    int *queueChnId = (int *)pCustomData;
    
    flush_audio_channel(*queueChnId);
    
//    fp_audio_output = fopen("./cameraAudio.aac", "w+b");
}
int32_t AudioStreamDataIn(void *pCustomData, RTSPAudioDesc_t *pDesc, uint8_t *pData)
{   
    int *queueChnId = (int *)pCustomData;

    AudioNodeDesc node;
    memset(&node, 0, sizeof(node));
    
    static uint8_t *pTempBuff = NULL;
    if(NULL == pTempBuff){
        pTempBuff = (uint8_t *)malloc(MEM_BLOCK_SIZE_64K);
    }
    int ret = get_node_from_audio_channel(*queueChnId, &node, pTempBuff);
    if(0 != ret){
        return ret;
    }
    
    // 写入ADTS头
    memcpy(pData, node.byAACHeader, node.wAACHeaderLen);
    // 写入AAC ES数据
    memcpy(pData+node.wAACHeaderLen, pTempBuff, node.dwDataLen);
    pDesc->dataLen    = node.wAACHeaderLen + node.dwDataLen;
    pDesc->timeStamp  = node.ddwTimeStamp;
    
    //if(fp_audio_output){
    //    fwrite(pData, 1, pDesc->dataLen, fp_audio_output);
    //}
    
    return 0;
}

int rtspServerInit(const char *moduleName)
{
    int queueChnId;
    
    log_manager_init(LOGCONFIG_PATH, moduleName);
    
    RtspServer_t srv;
    memset(&srv, 0, sizeof(RtspServer_t));
    srv.port = 554;
    queueChnId = CHANNEL_0;
    srv.stream[0].bEnable = true;
    srv.stream[0].videoHooks.eDateFmt = VDEC_FORMAT_H265;
    srv.stream[0].videoHooks.pConnectHook = VideoStreamConnect;
    srv.stream[0].videoHooks.pDataInHook = VideoStreamDataIn;
    srv.stream[0].videoHooks.pCustomData = &queueChnId;
    srv.stream[0].audioHooks.eDateFmt = AUDIO_TYPE_AAC_ADTS;
    srv.stream[0].audioHooks.pConnectHook = AudioStreamConnect;
    srv.stream[0].audioHooks.pDataInHook = AudioStreamDataIn;
    srv.stream[0].audioHooks.pCustomData = &queueChnId;
    strcpy(srv.stream[0].strName, "aabb");

    create_video_frame_queue_pool(CHANNEL_Max);
    create_audio_frame_queue_pool(CHANNEL_Max);

    create_rtsp_Server(srv);

    // 正常情况下不会走到这里
    PRINT_ERROR("Fatal Error! RtspServer loop exited!");

    return -1;
}


int init_rtsp_main_process() {
    // 1. 先执行Main进程的初始化
    // struct st_SysTask st_TaskInfo;
    // memset(&st_TaskInfo, 0, sizeof(st_TaskInfo));
    
    // 2. 创建RTSP服务器进程
    pid_t rtsp_pid = fork();
    if (rtsp_pid == 0) {
        // 子进程 - RTSP服务器
        printf("Starting RTSP server process...\n");
        return rtspServerInit(PROCESS_RTSPSERVER_NAME);
    } else if (rtsp_pid < 0) {
        printf("Failed to create RTSP server process\n");
        return -1;
    }
    
    // 等待RTSP服务器启动
    sleep(5);
    
    // 3. 创建编码器进程
    pid_t encoder_pid = fork();
    if (encoder_pid == 0) {
        // 子进程 - 编码器
        printf("Starting encoder process...\n");
        return enCoderInit(PROCESS_ENCODER_NAME);
    } else if (encoder_pid < 0) {
        printf("Failed to create encoder process\n");
        return -1;
    }
    
    // 4. 主进程继续运行
    printf("All processes created successfully\n");
    return 0;
}
