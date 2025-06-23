import cv2
import numpy as np
import glob

# 设置棋盘格参数
chessboard_size = (12, 8)  # 棋盘格的内角点个数
square_size = 1.45  # cm

# 准备棋盘格的世界坐标系坐标（假设z=0）
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# 用于存储所有图像的世界坐标系点和图像坐标系点
objpoints = []  # 世界坐标系中的点
imgpoints = []  # 图像坐标系中的点

# 获取所有棋盘格图像的路径
images = glob.glob('../chessboard/photo1.jpg')

for image_path in images:
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 检测棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # 绘制角点以进行可视化
        img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard Corners', img)
        cv2.waitKey(0)

cv2.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# 输出相机的内参和畸变系数
print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)

# 保存内参和畸变系数到文件
np.savez('camera_calibration.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
for image_path in images:
    img = cv2.imread(image_path)
    h, w = img.shape[:2]

    # 计算新的相机矩阵和有效像素区域
    # newcameramtx: 新的相机矩阵
    # roi: 有效像素区域（区域内的像素可以保证无畸变）
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # 使用新的相机矩阵 undistort 方法校正畸变图像
    # mtx: 原始相机矩阵
    # dist: 畸变系数
    # newcameramtx: 新的相机矩阵
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    #  # 获取有效像素区域
    x, y, w, h = roi
    # 裁剪图像，只保留有效像素区域
    dst = dst[y:y + h, x:x + w]

    cv2.imshow('Undistorted Image', dst)
    cv2.waitKey(0)
cv2.destroyAllWindows()

# 加载 .npz 文件
'''data = np.load('camera_calibration.npz')
print('测试')
print(data['mtx'])
print(data['dist'])'''
