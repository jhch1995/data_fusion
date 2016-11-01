#include <dirent.h>
#include <getopt.h> 

#include "gflags/gflags.h"

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
"\n";

int main(int argc, char *argv[])
{   
    int show_help = 0;
    int show_version = 0;
    int is_print_imu = 0;

    static struct option long_options[] = {
        {"help", no_argument, &show_help, 'h'},
        {"version", no_argument, &show_version, 'v'},
        {"imu", no_argument, &is_print_imu, 'i'},
        {0, 0, 0, 0}
    };

    while (true) 
    {
        // "hvi:"  :表示i后面要跟参数
        int opt = getopt_long(argc, argv, "hvi", long_options, NULL);
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
            printf("set11 is_print_imu = %d\n", is_print_imu);
        } else 
        {  // 'h'
            fprintf(stderr, usage, program);
            exit(EXIT_FAILURE);
        }
    }


    #if defined(USE_GLOG)
        #if (!defined(ANDROID))
            FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
            google::InitGoogleLogging(argv[0]);
        #endif
    #endif  

   

// 初始化融合函数
    // 进行数据融合的类
    DataFusion data_fusion;
    data_fusion.StartDataFusionTask();  

    if(is_print_imu)
    {
        printf("set is_print_imu = %d\n", is_print_imu);
        data_fusion.print_imu_data(is_print_imu);
    }
    while(1)
    {
      sleep(1);  
    }

  
    return 0;
}


