# data_fusion说明
## 文件说明：
 build：编译的路径
 common：软连接到库
 lib：本地利用标注数据进行测试依赖的库，外部使用不需要
src：预测相关的代码
thirdparty：软连接到库

### src相关文件：
data_fusion：车道线预测的调用和接口函数
imu_attitude_estimate：姿态解算
can_vehicle_estimate：车辆运动信息解算
datafusion_math：数学库

lan_tracking_main：进行本地标注数据测试的主函数（外部使用无需调用）

### 数据文件
data_fusion：离线跑的时候会调用log数据，会读取build/bin/data/doing/log.txt文件


##备注
#### 本地测试时的数据
an_tracking_main：进行本地利用标注数据测试的时候，会调用
1). 车道线标注结果：build/bin/data/doing/lane_data.txt
2). 图像文件：build/bin/data/doing/frame/***