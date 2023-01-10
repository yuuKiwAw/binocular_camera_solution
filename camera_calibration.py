import cv2
import numpy as np


# 阈值
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#棋盘格模板规格
w = 9
h = 6

# 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵
objp = np.zeros((w*h,3), np.float32)
objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)

objpoints = [] # 在世界坐标系中的三维点
imgpoints = [] # 在图像平面的二维点
def main():
    img = cv2.imread("D:\\source\\python_project\\dualcamera\\save_img\\20230110-094220\\right\\right_10.jpg")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 找到棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, (w,h),None)
    if ret == True:
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        # 将角点在图像上显示
        cv2.drawChessboardCorners(img, (w,h), corners, ret)
        cv2.imshow('findCorners',img)
        cv2.waitKey(0)

    cv2.destroyAllWindows()

    # 标定
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


if __name__ == '__main__':
    main()