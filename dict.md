3.26 今天分了组 探索了可能可以实现的方法
可实现的方法为如果用单目摄像头+雷达 
步骤	输入	输出	方法
1	RGB图像	数字识别结果 (bbox位置, 数字)	OCR模型 (如PaddleOCR)
2	点云数据、标定外参	投影到图像上每个点的(u,v)坐标与深度	相机内外参矩阵转换
3	bbox区域内点云聚类	每个box精确位置（点云质心）	DBSCAN聚类
4	重复计数判别	box中心位置、数字	位置距离阈值判断

或许也许可以用深度相机的信息

工作 ： 部署了yolov11 但是官方的模型并不是很适配
roslaunch me5413_world world.launch
roslaunch me5413_world navigation.launch
rosrun detection yolov11.py 