# 位姿、尺寸使用 DetectedObjectArray.header 的坐标系和采集时间。
uint8 RED=1
uint8 GREEN=2
uint8 BLUE=3

string object_id
uint8 color
geometry_msgs/Pose pose             # 物体几何中心
geometry_msgs/Vector3 dimensions    # 物体自身坐标系尺寸，m
