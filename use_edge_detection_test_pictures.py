import os
import edge_detection_canny_multi_rects as cug
import cv2 as cv
import numpy as np

dir = 'C:\\Users\\18056\\PycharmProjects\\untitled\\pic1'
outdir =  'C:\\Users\\18056\\PycharmProjects\\untitled\\test_output_images\\'

if dir is None or not os.path.exists(dir):
    raise Exception("input_dir does not exist")
for dirpath, dirnames, filenames in os.walk(dir):
    for filename in filenames:
        name, _ = os.path.splitext(filename)
        path = os.path.join(dirpath, filename)
        origin = cv.imread(path)
        print("start***********************************************************")
        res = cug.process_pictue(origin)
        out_file_name = outdir + filename
        result_code = res[0]
        #  存储结果
        processed_image = np.concatenate((origin, res[1]), axis=1)
        print(out_file_name)
        print("result_code:" + str(res[0]))
        if(result_code != 12):
            print("center_point" + "(" + str(res[2]) + "," + str(res[3]) + ")")
        print("end***********************************************************")
        cv.imwrite(out_file_name,processed_image)
        """
                    返回所有效果，在原图上画矩形框
                    box = cv.boxPoints(rect)
                    box = np.int0(box)
                    cv.drawContours(origin, [box], 0, (0, 0, 225), 2)
                    center = rect[0]
                    if((0 < rect[0][1] < 640) & (0 < rect[0][0] < 480)):
                        for i in range(10):
                            origin[int(center[1] + i), int(center[0])] = [255, 0, 0]
                            origin[int(center[1] - i), int(center[0])] = [255, 0, 0]
                            origin[int(center[1] ), int(center[0]) + i] = [255, 0, 0]
                            origin[int(center[1] ), int(center[0]) - i] = [255, 0, 0]
                            cv.imwrite(out_file_name,origin)
        """

