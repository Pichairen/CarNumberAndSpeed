# 车辆统计和车速预测
这里提供一个简单的基于Opencv3的车辆统计和车速测量方法
效果图：
![Image text](https://github.com/Pichairen/CarNumberAndSpeed/blob/master/%E6%95%88%E6%9E%9C%E5%9B%BE.png)
# 车辆统计
![Image text](https://github.com/Pichairen/CarNumberAndSpeed/blob/master/%E8%BD%A6%E8%BE%86%E7%BB%9F%E8%AE%A1.PNG)
# 车速估计
![Image text](https://github.com/Pichairen/CarNumberAndSpeed/blob/master/%E8%BD%A6%E9%80%9F%E4%BC%B0%E8%AE%A1.PNG)
# 优化方向
1、代码中RoI是固定的，可以设置鼠标响应事件设置RoI  

2、背景建模后还有阴影影响车辆检测的准确度，可以设置算法进行背景消除  

3、本来使用质心进行测速的，但是发现车辆在进出RoI的时候车辆质心随外接矩形进行变换，修改为车辆进入窄带的第一个点(这里使用右下角)替代质心提高准确度  

4、没有进行标定，速度估计单位为pixels/s
