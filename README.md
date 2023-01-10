# HBVCAM-4M2214HD-2 V11

## python脚本说明
- dual_camera_opencv.py (双目相机测试使用)
- dual_camera_shutter.py (双目相机照片拍摄使用)
- gen_pattern.py (生成一个宽9 高6 的棋盘, A4打印每格边长约为27mm)
- camera_calibration.py (opencv棋盘点标定)
- point_cloud_display.py (点云图读取显示)

## c++程序说明
- ./src/stereo_calib.cpp (官方案例使用opencv双目相机标定)

## Opencv stereo_calib.cpp 双目相机标定
### 内外参数据说明
#### 内参 intrinsic.yml

- M1: cameraMatrix1 First camera matrix. （相机1矩阵）
- D1: distCoeffs1 First camera distortion parameters.（distortion coefficient，相机1畸变系数）
- M2: cameraMatrix2 Second camera matrix.（相机2矩阵）
- D2: distCoeffs2 Second camera distortion parameters.（相机2畸变系数)

#### 外参 extrinsic.yml

- R:  @param R Rotation matrix between the coordinate systems of the first and the second cameras.（两相机坐标系的旋转矩阵）
- T:  @param T Translation vector between coordinate systems of the cameras.（两相机坐标系的平移向量）
- R1: @param R1 Output 3x3 rectification transform (rotation matrix) for the first camera.（相机1的3x3整流变换（旋转矩阵））
- R2: @param R2 Output 3x3 rectification transform (rotation matrix) for the second camera.（相机2的3x3整流变换（旋转矩阵））
- P1: @param P1 Output 3x4 projection matrix in the new (rectified) coordinate systems for the first camera.（相机1对于新坐标系的（调整后的）投影矩阵）
- P2: @param P2 Output 3x4 projection matrix in the new (rectified) coordinate systems for the second camera.（相机2对于新坐标系的（调整后的）投影矩阵）
- Q:  @param Q Output 4x4 disparity-to-depth mapping matrix (see reprojectImageTo3D ).（4X4视差 到 深度映射矩阵）