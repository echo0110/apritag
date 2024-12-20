#include "camera_capture.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define TASK_INFO_NAME_LEN 256
#define TASK_RESPAWN_RETRY_MAX_TIMES 4 // 子进程崩溃后重启重试次数
struct st_Task_Info
{
	char byTaskName[TASK_INFO_NAME_LEN];
	char byTaskPara[TASK_INFO_NAME_LEN];
	pid_t dwTaskID;
	time_t dwExitBootTime;
	time_t dwExitRunTime;
	pid_t dwExitTaskID;
	uint32_t dwExitTimes;
	int32_t dwExitStatus;
};

#define PROGRAM_NAME_LEN 64 //主进程名
#define TASK_MAX_NUMBER 20  //子进程总数
struct st_SysTask
{
    char progName[PROGRAM_NAME_LEN];
	uint32_t dwTaskNum;
	struct st_Task_Info st_task[TASK_MAX_NUMBER];
};


#define PROCESS_ENCODER_NAME   "enCoder"
#define PROCESS_RTSPSERVER_NAME "rtspServer"

void ShowStatus( pid_t pid, int32_t status )
{
  int flag = 1;

  printf( "[show_status]: pid = %u => ", pid );

  if ( WIFEXITED( status ) ) {
    flag = 0;
    printf( "true if the child terminated normally, that is, "
    "by calling exit() or _exit(), or "
    "by returning from main().\n" );
	printf("子进程正常结束!\n");
  }
  if ( WEXITSTATUS( status ) ) {
    flag = 0;
    printf( "evaluates to the least significant eight bits of the "
    "return code of  the  child  which terminated, which may "
    "have been set as the"
    "argument to a call to exit() or _exit() or as the argument for a"
    "return  statement  in  the main program.  This macro can only be"
    " evaluated if WIFEXITED returned true. \n" );
	printf("子进程exit()返回的结束代码:[%d]\n",WEXITSTATUS( status ));
  }
  if ( WIFSIGNALED( status ) ) {
      flag = 0;
      printf( "\033[33m true if the child process terminated because of a signal"
	   	" which was not caught.\033[0m\n" );
	  printf("子进程是因为信号而结束!\n");
  }
  if ( WTERMSIG( status ) ) {
      flag = 0;
      printf( "  the  number of the signal that caused the child process"
	    "to terminate. This macro can only be  evaluated  if  WIFSIGNALED"
	    "returned non-zero.\n");
	  printf("子进程因信号而中止的信号代码:[%d] - [%d]\n", WIFSIGNALED( status ),WTERMSIG( status ));
  }
  if ( WIFSTOPPED( status ) ) {
      flag = 0;
      printf( "  true  if  the  child process which caused the return is"
	    "currently stopped; this is only possible if the  call  was  done"
	    "using   WUNTRACED  or  when  the  child  is  being  traced  (see"
	    "ptrace(2)).\n" );
	  printf("子进程处于暂停执行情况!\n");
	  
  }
  if ( WSTOPSIG( status ) ) {
    flag = 0;
    printf( " the number of the signal which caused the child to stop."
	  "This   macro  can  only  be  evaluated  if  WIFSTOPPED  returned"
    "non-zero.\n" );
	printf("引发子进程暂停的信号代码:[%d]\n", WIFSTOPPED( status ));
	
  }
  if ( flag ) {
    printf( "Unknown status = 0x%X\n", status );
  }
}

static int32_t CreateProcess(const char *pcPara, struct st_SysTask *st_TaskInfo)
{
	pid_t pid;
	uint8_t byNum = 0;
	uint8_t byFlg = 1;

    if(TASK_MAX_NUMBER <= st_TaskInfo->dwTaskNum){
        printf("[error]-The number of processes exceeds the limit, Max Task number is : %d\n", TASK_MAX_NUMBER);
        return -1;
    }

	pid = fork();						//创建子进程
	if(pid == -1)						//创建失败
	{
		printf("fork error, error: %d => %s", errno, strerror(errno));
		return -1;
	}
	else if(pid != 0)					//如果是父进程,则返回
	{
		for(byNum = 0; byNum < st_TaskInfo->dwTaskNum; byNum++)
		{
			if(0 == strcmp(pcPara, st_TaskInfo->st_task[byNum].byTaskName))
			{
				st_TaskInfo->st_task[byNum].dwTaskID = pid;
				st_TaskInfo->st_task[byNum].dwExitBootTime = time(NULL);
				byFlg = 0;
			}
		}
		
		if(byFlg)
		{
			strcpy(st_TaskInfo->st_task[st_TaskInfo->dwTaskNum].byTaskName, pcPara);
			strcpy(st_TaskInfo->st_task[st_TaskInfo->dwTaskNum].byTaskPara, pcPara);
			st_TaskInfo->st_task[st_TaskInfo->dwTaskNum].dwTaskID = pid;
			st_TaskInfo->dwTaskNum++;
			st_TaskInfo->st_task[st_TaskInfo->dwTaskNum].dwExitBootTime = time(NULL);
		}
		
		osTask_msDelay(100);
		return pid; 
	}

    char cmd[PROGRAM_NAME_LEN+2] = {0};
    sprintf(cmd,"./%s",st_TaskInfo->progName);
	//执行程序文件
	if(execlp(cmd, st_TaskInfo->progName, pcPara, (char *)0) < 0)
	{
		perror("execlp error");
		printf("execlp error: %d => %s", errno, strerror(errno));
		exit(1);
	}
    
    return 0;
}

// Proposed changes to camera_capture.cpp
// Define the init_rtsp_server function

bool init_rtsp_server(const char* progName) {
    struct st_SysTask st_TaskInfo;
    memset(&st_TaskInfo, 0, sizeof(st_TaskInfo));
    strcpy(st_TaskInfo.progName, progName);

    // Initialize logging
    log_manager_init(LOGCONFIG_PATH, progName);

    // Create RTSP server process
    if (CreateProcess(PROCESS_RTSPSERVER_NAME, &st_TaskInfo) == -1) {
        std::cerr << "Failed to create RTSP server process" << std::endl;
        return false;
    }
    
    // Create encoder process
    if (CreateProcess(PROCESS_ENCODER_NAME, &st_TaskInfo) == -1) {
        std::cerr << "Failed to create encoder process" << std::endl;
        return false;
    }

    // Wait for processes to initialize
    sleep(2);  // Give processes time to start up

    std::cout << "RTSP server initialized on port 554" << std::endl;
    std::cout << "Stream URL: rtsp://[your-ip]:554/aabb" << std::endl;
    return true;
}

int init_rtsp_main_process() {
    // 1. 先执行Main进程的初始化
    struct st_SysTask st_TaskInfo;
    memset(&st_TaskInfo, 0, sizeof(st_TaskInfo));
    
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
    sleep(1);
    
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


