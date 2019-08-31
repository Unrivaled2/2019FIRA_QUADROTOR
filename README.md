# Project Title

2019 FIRA 无人机竞速钻框项目代码

## Getting Started

任务说明：起飞之后钻过红框，无人机自动调整速度，距离

本代码包括了通道控制代码，图像识别处理代码，图像质量检测代码，图像采集代码，图像识别效果浏览代码，感兴趣像素HSV阈值采集代码
串口通信代码。并拥有一整套完整的任务作业流程，可以帮助你快速了解无人机自动控制，并完成钻款任务

### Prerequisites

使用的无人机是我们自己设计完成

##### 核心硬件

匿名光流

匿名拓空者飞控

在板控制电脑：Up Board

遥控器：乐迪遥控器

Up Board系统: ubilinux或者ubuntu

Opencv版本：4.1.1

python版本：3.6

![Up Board](https://github.com/Unrivaled2/quadrotor/blob/master/Up%20Board.jpg) 

![光流](https://github.com/Unrivaled2/quadrotor/blob/master/%E5%85%89%E6%B5%81.jpg)

![摄像头](https://github.com/Unrivaled2/quadrotor/blob/master/%E6%91%84%E5%83%8F%E5%A4%B4.jpg)

![比赛框](https://github.com/Unrivaled2/quadrotor/blob/master/%E6%AF%94%E8%B5%9B%E6%A1%86.jpg)

![飞控](https://github.com/Unrivaled2/quadrotor/blob/master/%E9%A3%9E%E6%8E%A7.jpg)


### How to use

1. 飞机组装校正后，Up Board编译安装opencv，将代码上传到up Board一份

2. 相同目录下创建pic文件夹和pic1文件夹用来保存图片

  ```
  python capture_read.py
  ```
  
3. 可以遥控起飞后或者手持飞机，运行采集图片代码(每秒30帧)，图片将保存到pic1文件夹，将pic1文件夹下载到本地

4. 依次选取几张照片进行确定感兴趣颜色阈值，确定方法为

    修改rect_sample.py文件73行路径为进行取样的图片路径
    使用IDE运行rect_sample.py,img窗口显示原图，并且使用鼠标在其上勾画矩形取样，多次取样，将对取样范围求交，mask窗口显示在取样范围内的原图的二值图
    控制台会将取样范围打印
    
    ![运行结果示例](https://github.com/Unrivaled2/rect_sample_show/blob/master/rect_sample_show.png)
    ![控制台打印的范围示例](https://github.com/Unrivaled2/rect_sample_show/blob/master/rect_sample_result.png)
    
5. 根据第四步得到的范围更改edge_detection_canny_multi_rects.py378，379行颜色范围的上下界

6. 检查取样效果，运行detect_rectangle_center.py,显示两个窗口，origin为原图，process为处理后的二值图，键盘的上下键进行pic1目录所有图片效果浏览，
取样结果满意，继续下一步，否则返回第四步

    ![浏览效果示例](https://github.com/Unrivaled2/rect_sample_show/blob/master/detect_rectangle_center.png)
 
7. 将edge_detection_canny_multi_rects.py上传到飞机的Up Board上，即可运行钻款

   ```
   python cug_drone.py
   ```
   运行结果中，会将识别到的图和处理后的图保存到pic文件夹中，可下载到本地查看
 
   
### Principle

根据取样后的处理结果将原图转化成二值图，之后对白色像素点求最小外包矩形，以此确定矩形中心点
起飞之后，根据中心点的位置调整飞机相对于框的水平位置，竖直位置，当飞机相对于矩形中心在一个合理范围
后，立马加速冲过去（速度不够会因为激光定高悬挂在框的顶部）

### some possible issues 

1. issue:飞机飞行不稳，一直飘

    解决：确定遥控器是否将飞行模式设置成定点（排除飞行模式不正确），重新拔插电源（排除串口缓存指令影响），换一个地面试一下（排除因为光流采集地面
  图片无差别造成的影响），如果依然不行，重新进行加速度计（一定在水平地面上），罗盘（在水平地面上），光流校准（在水平地面上，光流安装在飞机上校准）
  
2. 个别螺旋桨转速不稳

   解决：用扎带将连接对应电调和飞控的信号线绑死，排除因为信号不稳定的原因
  
3. 飞机起飞即飘忽不定
  
    解决：飞机起飞时的姿态就是不稳的，调整起飞姿态
  

## Authors

* **李静涛** - *Initial work* - [PurpleBooth](https://github.com/Unrivaled2)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

