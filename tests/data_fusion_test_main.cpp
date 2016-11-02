#include <dirent.h>
#include <getopt.h> 

#include "gflags/gflags.h"
#include "common/hal/halio.h"

#include "data_fusion.h"
#include "datafusion_math.h"

// global variables
static const char* program = "data_fudion_test";
static const char* version = "0.1.0";

static const char* usage = "Usage: %s [options]\n"
"\n"
"Options:\n"
"  -h, --help                   Print this help message and exit\n"
"  -v, --version                Print version message and exit\n"
"  -i, --imu                    Print the imu data\n"
"  -s, --speed                  Print the speed data\n"

"\n";

void speed_callback(struct timeval *tv, int type, float speed)
{
//    printf("callback timestamp %ld %ld: type %d speed %f\n",
//            tv->tv_sec, tv->tv_usec, type, speed);
}


int main(int argc, char *argv[])
{   
    int show_help = 0;
    int show_version = 0;
    int is_print_imu = 0;
    int is_print_speed = 0;

    static struct option long_options[] = {
        {"help", no_argument, &show_help, 'h'},
        {"version", no_argument, &show_version, 'v'},
        {"imu", no_argument, &is_print_imu, 'i'},
        {"speed", no_argument, &is_print_speed, 's'},
        {0, 0, 0, 0}
    };

    while (true) 
    {
        // "hvi:"  :表示i后面要跟参数
        int opt = getopt_long(argc, argv, "hvis", long_options, NULL);
        if (opt == -1) 
        {
            break;
        }else if (opt == 'h') 
        {
            printf(usage, program);
            exit(EXIT_SUCCESS);
        }else if (opt == 'v') 
        {
            printf("%s version %s\n", program, version);
            exit(EXIT_SUCCESS);
        } else if (opt == 'i') 
        {
            is_print_imu = 1;
        } else if (opt == 's') 
        {            
            is_print_speed = 1;
        } else 
        {  // 'h'
            fprintf(stderr, usage, program);
            exit(EXIT_FAILURE);
        }
    }


    //读取车速
    HalIO &halio = HalIO::Instance();
    bool res = halio.Init(speed_callback, MOBILEEYE);
    if (!res) {
        std::cerr << "HALIO init fail" << std::endl;
        return -1;
    } 
    
    #if defined(USE_GLOG)
        #if (!defined(ANDROID))
            FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
            google::InitGoogleLogging(argv[0]);
        #endif
    #endif  

   
    // 进行数据融合的类
    DataFusion data_fusion;
    data_fusion.StartDataFusionTask();  

    if(is_print_imu)
    {
        printf("set is_print_imu = %d\n", is_print_imu);
        data_fusion.PrintImuData(is_print_imu);
    }

    if(is_print_speed)
    {
        printf("set is_print_speed = %d\n", is_print_speed);
        data_fusion.PrintSpeedData(is_print_speed);
    }


    while(1)
    {
      sleep(1);  
    }

  
    return 0;
}


