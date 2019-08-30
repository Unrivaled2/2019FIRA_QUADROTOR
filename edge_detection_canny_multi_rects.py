# coding:utf-8
import cv2 as cv
import numpy as np
from crop_rectangle import *


"""
本代码中对真边的判断是针对两种情况
1 某条边根据矩形法则运算出来而非真正对应一条边
2 某边右一些零星的点干扰形成

识别出来后的作用是
1 根据是否是真边判断是否是矩形的角
2 判断出角后便可以准确得到下一步准确移动位置，否则存在两种可能性

重要参数
根据hsv颜色阈值筛选后效果决定
485行，根据宽度和高度差值确定是否有效
460行 rangeList 代表进行探索的几个深度值 目前[5,10,15,20]
463行 thresh 代表确定假边的阈值 目前 [0.5]
400行 k_thresh 代表确定斜率无穷大的阈值 目前180
400行 min_thresh max_thresh k_select对于该斜率范围直线，保持原本斜率会过于大，直接加减又不合适,更新为k_select


目前存在的问题是：
1 颜色严格，选取所有的点
颜色不严格，进行筛选
"""

"""
用于识别方框中心
"""
GcenterX = 0 #方框中心X坐标
GcenterY = 0 #方框中心Y坐标
def bi_demo(image):   #双边滤波
    dst = cv.bilateralFilter(image, 0, 100, 15)
    return dst


#根据最小外包矩形的四个顶点和周围像素情况评估识别出来的中心点
def evaluation_rect(box,image,range,cur_state,thresh,k_thresh,min_thresh,max_thresh,k_select):
    """
    :param box: 外包矩形四个顶点
    :param image: 图片
    :param min_thresh max_thresh 对于该斜率范围直线，保持原本斜率会过于大，直接加减又不合适,更新为k_select
    :param  k_thresh
    :param k_thresh:代表确定斜率无穷大的阈值
    :param thresh: 白色像素百分比阈值，大于此阈值，为1
    :param range: 向内拓展像素范围
    :param cur_state: 判断当前是否确定该边为真边
    :return: [top,bottom,left,right] 此次判断结果，1代表确定为真边，0代表还没有
    """
    """
    有些图最小外包矩形返回的box是左下 左上 右上 右下
    有些是右下 左下 左上 右上
    如果是第二种，统一成第一种
    """
    if((box[0][0] - box[1][0]) > 30):
        #print("swap value")
        a = box[0]
        box = np.array(box)
        box = np.delete(box,0,0) #arr pos axis
        box = np.insert(box,3,a,0) #arr index obj axis

    PointBLX = box[0][0]    #左下角点
    PointBLY = box[0][1]
    PointTLX = box[1][0]    #左上角点
    PointTLY = box[1][1]
    PointTRX = box[2][0]    #右上角点
    PointTRY = box[2][1]
    PointBRX = box[3][0]    #右下角点
    PointBRY = box[3][1]
    #print("最小外包矩形顶点信息-----------")
    #print(box)
    if(PointBRX == PointBLX):
        return [0,0,0,0]
    if(PointTRX == PointTLX):
        return [0,0,0,0]
    top = 0  #记录是否是合格的边
    bottom = 0
    left = 0
    right = 0
    """
    存储要裁剪矩形的另外两个点
    """
    if(cur_state[1] == 0):
        #print("评估底边")
        tempX1 = PointBLX
        tempX2 = PointBRX
        if(PointTLX == PointBLX):
            k = 100
        else:
            k = (PointTLY - PointBLY)/(PointTLX - PointBLX)
        if(abs(k) >= min_thresh) & (abs(k) <= max_thresh):
            k = k_select*(k/abs(k))
        #print("斜率")
        #print(k)
        if (abs(k) > k_thresh):
            tempY1 = PointBLY - range
            tempY2 = PointBRY - range
        else:
            tempY1 = PointBLY - abs(k) * range
            tempY2 = PointBRY - abs(k) * range
            if (k >= 0):
                tempX1 = PointBLX + range
                tempX2 = PointBRX + range
            else:
                tempX1 = PointBLX - range
                tempX2 = PointBRX - range

        """
        print("extended rectange info----------------------")
        print(PointBLX)
        print(PointBLY)
        print(tempX1)
        print(tempY1)
        print(tempX2)
        print(tempY2)
        print(PointBRX)
        print(PointBRY)
        print("extended rectange info----------------------")
        """

        cnt = np.array([
            [[int(PointBLX), int(PointBLY)]],
            [[int(tempX1), int(tempY1)]],
            [[int(tempX2), int(tempY2)]],
            [[int(PointBRX), int(PointBRY)]]
        ])
        test_ans = crop_rotated_rectangle(image, cnt)
        test = test_ans[0]
        if (test is not None):
            test = cv.cvtColor(test, cv.COLOR_BGR2GRAY)
            height, width = test.shape
            percent = cv.countNonZero(test) / (width * height)
            if (percent > thresh):
                cv.drawContours(image, test_ans[1], 0, (0, 0, 255), 2)
                bottom = 1
                print("底边非黑色像素点数量百分比" + str(cv.countNonZero(test) / (width * height)))

        #print(' 底边部分白色像素占比:', cv.countNonZero(test))
        """
        print("======================")
        print(cnt)
        cv.imshow("after crop",test)
        cv.waitKey(0)
        """
    if(cur_state[2] == 0):

        #print("评估左边")
        tempY1 = PointTLY
        tempY2 = PointBLY
        if (PointTLY == PointTRY):
            k = 100
        else:
            k = (PointTRX - PointTLX) / (PointTRY - PointTLY)
        if (abs(k) >= min_thresh) & (abs(k) <= max_thresh):
            k = k_select * (k / abs(k))
        #print("斜率")
        #print(k)
        if (abs(k) > k_thresh):
            tempX1 = PointTLX + range
            tempX2 = PointBLX + range
        else:
            tempX1 = PointTLX + abs(k) * range
            tempX2 = PointBLX + abs(k) * range
            if (k >= 0):
                tempY1 = PointTLY + range
                tempY2 = PointBLY + range
            else:
                tempY1 = PointTLY - range
                tempY2 = PointBLY - range

        """
        print("左边----------------------")
        print(PointBLX)
        print(PointBLY)
        print(PointTLX)
        print(PointTLY)
        print(tempX1)
        print(tempY1)
        print(tempX2)
        print(tempY2)
        print("左边----------------------")
        """
        cnt = np.array([
            [[int(PointBLX), int(PointBLY)]],
            [[int(PointTLX), int(PointTLY)]],
            [[int(tempX1), int(tempY1)]],
            [[int(tempX2), int(tempY2)]],
        ])
        test_ans = crop_rotated_rectangle(image, cnt)
        test = test_ans[0]
        if (test is not None):
            test = cv.cvtColor(test, cv.COLOR_BGR2GRAY)
            height, width = test.shape
            percent = cv.countNonZero(test) / (width * height)
            if (percent > thresh):
                left = 1
                cv.drawContours(image, test_ans[1], 0, (0, 0, 255), 2)
                print("左边非黑色像素点数量百分比" + str(cv.countNonZero(test) / (width * height)))

    if(cur_state[3] == 0):
        #print("评估右边")
        tempY1 = PointTRY
        tempY2 = PointBRY
        if(PointTLY == PointTRY):
            k = 100
        else:
            k = (PointTRX - PointTLX)/(PointTRY - PointTLY)
        if (abs(k) >= min_thresh) & (abs(k) <= max_thresh):
            k = k_select * (k / abs(k))
        #print("右边斜率")
        #print(k)
        if (abs(k) > k_thresh ):
            tempX1 = PointTRX - range
            tempX2 = PointBRX - range
        else:
            tempX1 = PointTRX - abs(k) * range
            tempX2 = PointBRX - abs(k) * range
            if (k >= 0):
                tempY1 = PointTRY - range
                tempY2 = PointBRY - range
            else:
                tempY1 = PointTRY + range
                tempY2 = PointBRY + range
        """
        print("----------------------")
        print(PointBLX)
        print(PointBLY)
        print(PointTLX)
        print(PointTLY)
        print(tempX1)
        print(tempY1)
        print(tempX2)
        print(tempY2)
        print("----------------------")
        """
        cnt = np.array([
            [[int(tempX2), int(tempY2)]],
            [[int(tempX1), int(tempY1)]],
            [[int(PointTRX), int(PointTRY)]],
            [[int(PointBRX), int(PointBRY)]],
        ])
        test_ans = crop_rotated_rectangle(image, cnt)
        test = test_ans[0]
        if (test is not None):
            test = cv2.cvtColor(test, cv2.COLOR_BGR2GRAY)
            height, width = test.shape
            percent = cv2.countNonZero(test) / (width * height)
            if (percent > thresh):
                right = 1
                """
                临时调试信息
                """

                """
                print("临时调试信息")
                print(k)
                print(PointBLX)
                print(PointBLY)
                print(PointTLX)
                print(PointTLY)
                print(tempX1)
                print(tempY1)
                print(tempX2)
                print(tempY2)
                """
                cv2.drawContours(image, test_ans[1], 0, (0, 0, 255), 2)
                height, width = test.shape
                print("右边非黑色像素点数量百分比" + str(cv2.countNonZero(test) / (width * height)))

    if(cur_state[0] == 0):
        #print("评估上边")
        tempX1 = PointTLX
        tempX2 = PointTRX
        if (PointTLX == PointBLX):
            k = 100
        else:
            k = (PointTLY - PointBLY) / (PointTLX - PointBLX)
        if (abs(k) >= min_thresh) & (abs(k) <= max_thresh):
            k = k_select * (k / abs(k))
        #print("斜率")
        #print(k)
        if (abs(k) > k_thresh):
            tempY1 = PointTLY + range
            tempY2 = PointTRY + range
        else:
            tempY1 = PointTLY + abs(k) * range
            tempY2 = PointTRY + abs(k) * range
            if (k >= 0):
                tempX1 = PointTLX - range
                tempX2 = PointTRX - range
            else:
                tempX1 = PointTLX + range
                tempX2 = PointTRX + range
        """
        print("----------------------")
        print(range)
        print(tempX1)
        print(tempY1)
        print(PointTLX)
        print(PointTLY)
        print(tempX2)
        print(tempY2)
        print(PointTRX)
        print(PointTRY)
        print("----------------------")
        """

        cnt = np.array([
            [[int(tempX1), int(tempY1)]],
            [[int(PointTLX), int(PointTLY)]],
            [[int(tempX2), int(tempY2)]],
            [[int(PointTRX), int(PointTRY)]]
        ])
        test_ans = crop_rotated_rectangle(image, cnt)
        test = test_ans[0]
        if (test is not None):
            test = cv.cvtColor(test, cv.COLOR_BGR2GRAY)
            height, width = test.shape
            percent = cv.countNonZero(test) / (width * height)
            if (percent > thresh):
                top = 1
                cv.drawContours(image, test_ans[1], 0, (0, 0, 255), 2)
            height, width = test.shape
            print("上边非黑色像素点数量百分比" + str(cv.countNonZero(test) / (width * height)))
    return [top,bottom,left,right]
def process_pictue(image):
    """
    :param image: 要处理的图片
    :return: [result_code,image]
    result_code
    [0,image]   #图片无效
    [1,image,centerX,centerY]    #四个真角，四个边均为真边，可直接冲过去或向前移动
    [2, image,centerX,centerY]   #只有左上角为真角
    [3, image,centerX,centerY]   #只有右上角为真角
    [4, image,centerX,centerY]   # 只有右下角为真角
    [5, image,centerX,centerY]   # 只有左下角为真角
    [6,image,centerX,centerY] #上下两边为真边，零个真角，中心y值为真值
    [7, image, centerX, centerY]  # 上下两边为真边，零个真角，中心y值为真值
    [8, image, centerX, centerY]  # 左右底部三边为真边，两个真角
    [9, image, centerX, centerY]  # 左右顶部三边为真边，两个真角
    [10, image, centerX, centerY]  # 上下右部三边为真边，两个真角
    [11, image, centerX, centerY]  # 上下左部三边为真边，两个真角

    """
    # 读取图片
    # print("read image")
    #image = cv.imread(path)
    #cv.imshow("after remove specularity", image)
    #image = RectifyInit(image)

    """
    imread函数：
    cv2.IMREAD_COLOR : Loads a color image. Any transparency of image will be neglected. It is the default flag.  flag = 1
        format:
            BGR order if you used cv2.imread()
            RGB order if you used mpimg.imread() (assuming import matplotlib.image as mpimg)
    cv2.IMREAD_GRAYSCALE : Loads image in grayscale mode flag = 0
    cv2.IMREAD_UNCHANGED : Loads image as such including alpha channel flag = -1
    """

    """
    #灰度图转二值图代码
    (thresh, im_bw) = cv.threshold(image, 128, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    cv.imshow("after binary change",im_bw)
    """

    """
    图像rgb选取品红范围之内，将颜色空间BGR转换为HSV空间（利于颜色过滤）
    color selection
    """
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    """
    颜色范围通过取样决定
    """
    lower_red = np.array([0, 198, 149])
    higher_red = np.array([4, 249, 218])
    mask2 = cv.inRange(hsv, lower_red, higher_red)
    image = mask2
    #cv.imshow("after filter", image)
    # print(gray)

    """
    # 二值图腐蚀处理
    #cv.imshow("before erosion",image)
    kernel = np.ones((3, 3), np.uint8)
    image_erosion = cv.erode(image, kernel, iterations=1)
    #cv.imshow("after erosion",image_erosion)
    vix2 = np.concatenate((image,image_erosion),axis=1)
    cv.imshow("erosion",vix2)
    image = image_erosion
    """

    # 用腐蚀和膨胀,对二值图像进行预处理 ，技巧：先腐蚀，后膨胀
    kernel = np.ones((3, 3), np.uint8)
    image_erosion = cv.erode(image, kernel, iterations=1)
    #cv.imshow("after erosion",image_erosion)
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
    image = cv.dilate(image_erosion, kernel)  # 膨胀
    #cv.imshow("after dilate",image)
    # 识别物体轮廓
    contours = cv.findContours(image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    #print(contours)
    #print("len(contours)" + str(len(contours)))
    image = cv.cvtColor(image,cv.COLOR_GRAY2BGR)

    """
    参数记录最大矩形框点集
    """
    max_rec_points = [
        [[ ]],
    ]
    max_num = 0
    """
    该代码选取点数最多轮廓
    mark = 0 #标记是否找到轮廓点
    for cnt in contours[0]:
    #contours[0]包含许多外包矩形，coutours[1]只包含最大的外包矩形
    #print("len(cnt)" + str(len(cnt)))
        if len(cnt) > 180:
            if(len(cnt) > max_num):
                mark = 1
                max_rec_points = cnt
                max_num = len(max_rec_points)
    """

    """
    该代码合并所有轮廓点到max_rec_points
    """
    mark = 0 #标记是否找到轮廓点
    #print(np.array(contours[0]).shape)
    if(np.array(contours[0]).shape[0] == 1):
        max_rec_points = contours[0][0]
        mark = 1
    elif(np.array(contours[0]).shape[0] > 1):
        i = 1
        max_rec_points = contours[0][0]
        for cnt in contours[0]:
            if(i == 1):
                i += 1
                continue
            max_rec_points = np.vstack((max_rec_points,cnt))
            mark = 1

    """
    print("max_rec_points---------------")
    print(max_rec_points)
    """



    """
    识别出来每个多边形轮廓，再分别求解最小外包矩形
    """
    if(mark == 1):
        rect = cv.minAreaRect(max_rec_points)  #（最小外接矩形的中心（x，y），（宽度，高度），旋转角度）
        box = cv.boxPoints(rect) #外包矩形四个顶点
        box = np.int0(box)
        rec_width = rect[1][0]
        rec_height = rect[1][1]
        """
        画出矩形框和中心点
        """
        centerX = rect[0][0]
        centerY = rect[0][1]
        #print(centerX)
        #print(centerY)
        #print(box)
        cv.drawContours(image, [box], 0, (0, 0, 225), 2)
        """
        opencv坐标系，左上角（0，0）
        向下y
        向右x
        """
        image[int(centerY), int(centerX)] = [0, 0, 255]  # 注意传进去的是y,x参数
        GcenterX = centerX
        GcenterY = centerY
        """
        这里筛选根据是如果长度和宽度差值小于20则返回有效值,否则无效值
        """
        print("width: " + str(rec_width))
        print("height: " + str(rec_height))
        print("abs(width- height): " + str(abs(rec_height - rec_width)))
        if(abs(rec_width - rec_height) <= 120 ):
            percent = (rec_width * rec_height)/(480*640)
            return [1,image,centerX,centerY,percent]
        else:
            return [12, image]
        """
        cv.imshow("find center",image)
        cv.waitKey(0)
        """

        """
        进行四个边是否为真边判断
        """
        rangeList = [5,10,20]
        cur_state = [0,0,0,0] #代表当前判断出对应边合适
        for ran in rangeList:
            ans = evaluation_rect(box,image,ran,cur_state,0.5,9,2,9,2)
            cur_state[0]  = cur_state[0] | ans[0]
            cur_state[1] = cur_state[1] | ans[1]
            cur_state[2] = cur_state[2] | ans[2]
            cur_state[3] = cur_state[3] | ans[3]
        print("四个边判断结果--------------")
        print(cur_state)
        if((not cur_state[0]) & (not cur_state[1]) & (not cur_state[2]) & (not cur_state[3])):
            return [0,image,centerX,centerY]   #图片无效
        if(cur_state.count(1) is 1):
            return [0, image,centerX,centerY]  # 图片无效
        if(cur_state[0] & cur_state[1] & cur_state[2] & cur_state[3]):
            return [1,image,centerX,centerY]    #四个真角，四个边均为真边，可直接冲过去或向前移动
        if (cur_state[0] & (not cur_state[1]) & cur_state[2] & (not cur_state[3])):
            return [2, image,centerX,centerY]   #只有左上角为真角，两个真边
        if (cur_state[0] & (not cur_state[1]) & (not cur_state[2]) & ( cur_state[3])):
            return [3, image,centerX,centerY]   #只有右上角为真角，两个真边
        if ((not cur_state[0]) & ( cur_state[1]) & (not cur_state[2]) & (cur_state[3])):
            return [4, image,centerX,centerY]   # 只有右下角为真角，两个真边
        if ((not cur_state[0]) & ( cur_state[1]) & ( cur_state[2]) & (not cur_state[3])):
            return [5, image,centerX,centerY]   # 只有左下角为真角，两个真边
        if (( cur_state[0]) & ( cur_state[1]) & (not cur_state[2]) & (not cur_state[3])):
            return [6,image,centerX,centerY] #上下两边为真边，零个真角，中心y值为真值
        if ((not cur_state[0]) & (not cur_state[1]) & (cur_state[2]) & (cur_state[3])):
            return [7, image, centerX, centerY]  # 左右两边为真边，零个真角，中心x值为真值
        if(not cur_state[0]):
            return [8, image, centerX, centerY]  # 左右底部三边为真边，两个真角
        if (not cur_state[1]):
            return [9, image, centerX, centerY]  # 左右顶部三边为真边，两个真角
        if (not cur_state[2]):
            return [10, image, centerX, centerY]  # 上下右部三边为真边，两个真角
        if (not cur_state[3]):
            return [11, image, centerX, centerY]  # 上下左部三边为真边，两个真角
    return [12,image] #代表图片无效，且无中心点返回

    """
    另一种方法
    x, y, w, h = cv.boundingRect(cnt)
    cv.rectangle(image, (x, y), (x + w, y + h), (0, 255,), 3) #画出来的矩形是方方正正的，不具有倾斜角度
    """

    # cv.circle(image, (int(x + w / 2), int(y + h / 2)), 2, (0, 0, 255), 2)

    """
    判断是否返回结果
    """

    if(len(max_rec_points) != 0):
        rect = cv.minAreaRect(max_rec_points)
        """
        返回值：元组
        元组（（最小外接矩形的中心坐标），（宽，高），旋转角度）----->    ((y, x), (w, h), θ )
        """
        width = rect[1][0]
        height = rect[1][1]
        """
        进行筛选，根据宽高比以及中心点位置进行筛选
        """
        if(0.87 < width/height) & (width/height < 1.08 ) & (0 < rect[0][1] < 640) & (0 < rect[0][0] < 480):
            return rect
    return [0,image] #代表未成功找出中心点


"""
origin = cv.imread("./180.jpg",1)
rect = process_pictue("./180.jpg")
if(np.shape(rect) != (1,1)):
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(origin, [box], 0, (0, 0, 225), 2)
    center = rect[0]
    origin[int(center[0]), int(center[1])] = [255, 255, 255]
    cv.imshow("ans", origin)
    cv.waitKey(0)

"""

def process_pictue2(image):
    """
    :param image: 要处理的图片
    :return: [result_code,image]
    result_code
    [0,image]   #图片无效
    [1,image,centerX,centerY]    #四个真角，四个边均为真边，可直接冲过去或向前移动
    [2, image,centerX,centerY]   #只有左上角为真角
    [3, image,centerX,centerY]   #只有右上角为真角
    [4, image,centerX,centerY]   # 只有右下角为真角
    [5, image,centerX,centerY]   # 只有左下角为真角
    [6,image,centerX,centerY] #上下两边为真边，零个真角，中心y值为真值
    [7, image, centerX, centerY]  # 上下两边为真边，零个真角，中心y值为真值
    [8, image, centerX, centerY]  # 左右底部三边为真边，两个真角
    [9, image, centerX, centerY]  # 左右顶部三边为真边，两个真角
    [10, image, centerX, centerY]  # 上下右部三边为真边，两个真角
    [11, image, centerX, centerY]  # 上下左部三边为真边，两个真角

    """
    # 读取图片
    # print("read image")
    #image = cv.imread(path)
    #cv.imshow("after remove specularity", image)
    #image = RectifyInit(image)

    """
    imread函数：
    cv2.IMREAD_COLOR : Loads a color image. Any transparency of image will be neglected. It is the default flag.  flag = 1
        format:
            BGR order if you used cv2.imread()
            RGB order if you used mpimg.imread() (assuming import matplotlib.image as mpimg)
    cv2.IMREAD_GRAYSCALE : Loads image in grayscale mode flag = 0
    cv2.IMREAD_UNCHANGED : Loads image as such including alpha channel flag = -1
    """

    """
    #灰度图转二值图代码
    (thresh, im_bw) = cv.threshold(image, 128, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    cv.imshow("after binary change",im_bw)
    """

    """
    图像rgb选取品红范围之内，将颜色空间BGR转换为HSV空间（利于颜色过滤）
    color selection
    """
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    """
    颜色范围通过取样决定
    """
    lower_red = np.array([99, 181, 215])
    higher_red = np.array([106, 243, 252])
    mask2 = cv.inRange(hsv, lower_red, higher_red)
    image = mask2
    #cv.imshow("after filter", image)
    # print(gray)

    """
    # 二值图腐蚀处理
    #cv.imshow("before erosion",image)
    kernel = np.ones((3, 3), np.uint8)
    image_erosion = cv.erode(image, kernel, iterations=1)
    #cv.imshow("after erosion",image_erosion)
    vix2 = np.concatenate((image,image_erosion),axis=1)
    cv.imshow("erosion",vix2)
    image = image_erosion
    """

    # 用腐蚀和膨胀,对二值图像进行预处理 ，技巧：先腐蚀，后膨胀
    kernel = np.ones((3, 3), np.uint8)
    image_erosion = cv.erode(image, kernel, iterations=1)
    #cv.imshow("after erosion",image_erosion)
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
    image = cv.dilate(image_erosion, kernel)  # 膨胀
    #cv.imshow("after dilate",image)
    # 识别物体轮廓
    contours = cv.findContours(image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    #print(contours)
    #print("len(contours)" + str(len(contours)))
    image = cv.cvtColor(image,cv.COLOR_GRAY2BGR)

    """
    参数记录最大矩形框点集
    """
    max_rec_points = [
        [[ ]],
    ]
    max_num = 0
    mark = 0
    """
    该代码选取点数最多轮廓
    mark = 0 #标记是否找到轮廓点
    """
    for cnt in contours[0]:
    #contours[0]包含许多外包矩形，coutours[1]只包含最大的外包矩形
    #print("len(cnt)" + str(len(cnt)))
        if len(cnt) > 0:
            if(len(cnt) > max_num):
                mark = 1
                max_rec_points = cnt
                max_num = len(max_rec_points)


    """
    该代码合并所有轮廓点到max_rec_points
    
    mark = 0 #标记是否找到轮廓点
    #print(np.array(contours[0]).shape)
    if(np.array(contours[0]).shape[0] == 1):
        max_rec_points = contours[0][0]
        mark = 1
    elif(np.array(contours[0]).shape[0] > 1):
        i = 1
        max_rec_points = contours[0][0]
        for cnt in contours[0]:
            if(i == 1):
                i += 1
                continue
            max_rec_points = np.vstack((max_rec_points,cnt))
            mark = 1

    """
    print("max_rec_points---------------")
    print(max_rec_points)
    """



    """
    #识别出来每个多边形轮廓，再分别求解最小外包矩形

    if(mark == 1):
        rect = cv.minAreaRect(max_rec_points)  #（最小外接矩形的中心（x，y），（宽度，高度），旋转角度）
        box = cv.boxPoints(rect) #外包矩形四个顶点
        box = np.int0(box)
        rec_width = rect[1][0]
        rec_height = rect[1][1]

        #画出矩形框和中心点

        centerX = rect[0][0]
        centerY = rect[0][1]
        #print(centerX)
        #print(centerY)
        #print(box)
        cv.drawContours(image, [box], 0, (255, 255, 255), 2)
        if ((box[0][0] - box[1][0]) > 30):
            # print("swap value")
            a = box[0]
            box = np.array(box)
            box = np.delete(box, 0, 0)  # arr pos axis
            box = np.insert(box, 3, a, 0)  # arr index obj axis
        print("四个顶点：  ")
        print(box)
        PointBLX = box[0][0]  # 左下角点
        PointBLY = box[0][1]
        PointTLX = box[1][0]  # 左上角点
        PointTLY = box[1][1]
        PointTRX = box[2][0]  # 右上角点
        PointTRY = box[2][1]
        PointBRX = box[3][0]  # 右下角点
        PointBRY = box[3][1]
        Point1X = (PointBLX + PointBRX)/2
        Point1Y = (PointBLY + PointBRY)/2
        Point2Y = (PointTRY + PointTLY)/2
        Point2X = (PointTRX + PointTLX)/2
        Point1 = [Point1X,Point1Y]
        Point2 = [Point2X,Point2Y]
        image[int(centerY), int(centerX)] = [0, 0, 255]  # 注意传进去的是y,x参数
        image[int(Point1[1]), int(Point1[0])] = [0, 255, 0]
        image[int(Point2[1]), int(Point1[0])] = [0, 255, 0]
        GcenterX = centerX
        GcenterY = centerY
        print("width: " + str(rec_width))
        print("height: " + str(rec_height))
        print("Point1: " + str(Point1))
        print("Point2: " + str(Point2))
        print("abs(width- height): " + str(abs(rec_height - rec_width)))
        if(abs(rec_width - rec_height) <= 120 ):
            percent = (rec_width * rec_height)/(480*640)
            return [1,image,centerX,centerY,percent,Point1,Point2]
        else:
            return [12, image]
    return [12, image]

