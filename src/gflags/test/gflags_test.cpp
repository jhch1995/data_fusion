#include <iostream>
#include "gflags/gflags.h"

DEFINE_string(dir, "test dir", "comment string");
DEFINE_bool(is_true, true, "comment bool");
DEFINE_double(double_val, 0.2, "comment double");
DEFINE_int32(int32_val, 1, "comment int");

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::cout << "dir : " << FLAGS_dir << std::endl;
    std::cout << "bool : " << FLAGS_is_true << std::endl;
    std::cout << "double : " << FLAGS_double_val << std::endl;
    std::cout << "int32 : " << FLAGS_int32_val << std::endl;
    return 0;
}
