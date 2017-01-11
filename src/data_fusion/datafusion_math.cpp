#include "datafusion_math.h"
DEFINE_double(gyro_bias_x, 0, "imu gyro bias x ");
DEFINE_double(gyro_bias_y, 0, "imu gyro bias y ");
DEFINE_double(gyro_bias_z, 0, "imu gyro bias z ");

#if defined DATA_FROM_LOG
    DEFINE_string(imu_init_addr, "./imu.ini", "imu calibrate result base address");
    DEFINE_string(imu_parameter_log_addr, "./imu_paramer_log.txt", "save all the imu calibrate results");
    DEFINE_string(log_data_addr, "data/doing/log.txt", "the log data address (which save imu, speed and son on)");
    DEFINE_string(jpg_data_addr, "data/doing/frame/image", "the kpg data address ");
#else
    DEFINE_string(imu_init_addr, "/storage/sdcard0/imu.ini", "imu calibrate result base address");
    DEFINE_string(imu_parameter_log_addr, "/storage/sdcard0/imu_paramer_log.txt", "save all the imu calibrate results");

#endif


