#include <dirent.h>
#include <getopt.h> 
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <sys/types.h>
#include <unistd.h>

#include "gflags/gflags.h"
#include "common/hal/halio.h"

#include "data_fusion.h"
#include "datafusion_math.h"


int main(int argc, char *argv[])
{
    // 进行数据融合的类
    DataFusion data_fusion;    

    #if defined(ANDROID)
    {
        //读取车速
        HalIO &halio = HalIO::Instance();
        bool res = halio.Init(NULL, MOBILEEYE);
        if (!res) {
            std::cerr << "HALIO init fail" << std::endl;
            return -1;
        } 
    #endif


    while(1){
        printf(" loop\n");
        sleep(1); 
    }
    return 0;
}

