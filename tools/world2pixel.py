import numpy as np
import cv2
import pyads


def transform_to_pixel(projection_matrix, camera_matrix, dist_coeffs, robot_pose):
    """
    将机械臂末端坐标转换为像素坐标（考虑畸变）

    参数:
    projection_matrix: 3x4标定矩阵
    camera_matrix: 3x3相机内参矩阵
    dist_coeffs: 1x5畸变系数矩阵
    robot_pose: 4x4机械臂末端位姿矩阵

    返回:
    (u, v): 畸变校正后的像素坐标
    """
    position = robot_pose[:3, 3]

    # 齐次坐标向量 [x, y, z, 1]
    world_point = np.append(position, 1)

    projected = projection_matrix @ world_point

    u_norm = projected[0] / projected[2]
    v_norm = projected[1] / projected[2]

    norm_point = np.array([[[u_norm, v_norm]]], dtype=np.float32)

    dist_point = cv2.undistortPoints(norm_point, camera_matrix, dist_coeffs, P=camera_matrix)

    u, v = dist_point[0, 0]

    return int(round(u)), int(round(v))


def draw_bounding_box(image, center_u, center_v, box_size=224):
    """
    在图像上绘制以指定点为中心的边界框

    参数:
    image: 输入图像
    center_u, center_v: 中心点像素坐标
    box_size: 边界框大小（正方形边长）

    返回:
    绘制了边界框的图像
    """
    half_size = box_size // 2
    x1 = center_u - half_size
    y1 = center_v - half_size
    x2 = center_u + half_size
    y2 = center_v + half_size

    height, width = image.shape[:2]
    x1 = max(0, min(x1, width - 1))
    y1 = max(0, min(y1, height - 1))
    x2 = max(0, min(x2, width - 1))
    y2 = max(0, min(y2, height - 1))

    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(image, (center_u, center_v), 4, (0, 0, 255), -1)

    return image




def MDH(alpha, a, theta, d):
    #x轴旋转变化矩阵
    rx = np.array([
        [1, 0, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha), 0],
        [0, np.sin(alpha), np.cos(alpha), 0],
        [0, 0, 0, 1]
        ])
    #沿着x轴平移
    tx = np.array([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    #沿着z轴旋转
    rz = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    #沿着z轴平移，有方向
    tz = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])
    return rx@tx@rz@tz


def Aiv(alpha, a, d, theta, Tnum):
    for i in range(Tnum):
        Ti = MDH(alpha[i], a[i], theta[i], d[i])
        if i == 0:
            Ai = Ti
        else:
            Ai = Ai@Ti
    return Ai


if __name__ == "__main__":
    # 相机外参矩阵
    # projection_matrix= np.array(
    #     [[5.26707588e-01, -2.60123578e-01, -2.97883055e-01,  1.32539219e-01],
    #      [1.91903505e-02,  2.72253847e-01, -6.88317690e-01,  1.84549223e-02],
    #     [3.07424692e-05, -8.45717663e-04, -9.55467050e-04, 2.25523181e-05]]
    # )
    projection_matrix = np.array(
        [[5.77684915e-01, -3.32057757e-01, -3.23397164e-01,  1.64017036e-01],
         [9.63920367e-03,  2.16525868e-01, -6.13768421e-01,  2.90201833e-02],
        [1.20259315e-05, -3.58454817e-04, -3.61512970e-04, 3.04328323e-05]]
        )


    # 相机内参矩阵
    camera_matrix = np.array([
    [1.02374302e+03, 0, 3.28987281e+02],
    [0, 1.37430887e+03, 2.12861817e+02],
    [0, 0, 1]],
    dtype=np.float32)

    # 畸变系数
    dist_coeffs = np.array([1.35715707e+00, -2.41678706e+01, -1.26054713e-02, -8.59585295e-04, 1.95623551e+02], dtype=np.float32)

    # q = []
    # plc = pyads.Connection('10.1.233.139.1.1', 855)
    # plc.open()
    # for i in range(14):
    #     joint = plc.read_by_name(f"HumanInterface.TestHmiDataOut.masterJoint[{i}]", pyads.PLCTYPE_REAL)
    #     q.append(joint)
    #
    # plc.close()
    q = [0.19896475970745087, -0.182090625166893, 0.13544048368930817, -0.05935335531830788, -1.229324221611023, -0.050729282200336456, -0.4657789468765259, 0.14895488321781158, 0.0004690513014793396, -0.39636003971099854, -0.10277658700942993, -0.5083662271499634, -0.0017330986447632313, -1.4875304698944092]

    # print(q)
    alpha = [np.pi / 2, 0, -np.pi / 2, 0, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2]
    a = [0, 0, 0, 0.3, 0.35, 0, 0, 0]
    d = [0, -0.166, 0, 0, 0.145, 0, 0, 0]
    theta_left = [np.pi / 2, q[0], np.pi / 2 + q[1], -np.pi / 2 + q[2] - q[1], q[3], q[4], np.pi / 2 + q[5], np.pi + q[6]]
    theta_right = [np.pi / 2, q[7], np.pi / 2 + q[8], -np.pi / 2 + q[9] - q[8], q[10], q[11], np.pi / 2 + q[12], np.pi + q[13]]

    left_location = Aiv(alpha, a, d, theta_left, 7)
    right_location = Aiv(alpha, a, d, theta_right, 7)

    #print(left_location)

    robot_pose = left_location
    #robot_pose = right_location
    #机械臂坐标
    # T_end = np.array([[-0.980788721261899, 0.195073009677934, 7.16982724662051e-5, -0.0691156699763307],
    #                      [5.29894132665595e-7, 0.000370210014501179, -0.999999931472130, 0.321186715135642],
    #                      [-0.195073022853414, -0.980788654012544, -0.000363201174764230, 0.343134701098648],
    #                      [0, 0, 0, 1]])
    image = cv2.imread(r'D:\python\loose_hands_detection\250618\left-hand\image\on_hands\0142.jpg')
    u, v = transform_to_pixel(projection_matrix, camera_matrix, dist_coeffs, robot_pose)

    print(f"像素坐标: ({u}, {v})")


    # 在图像上绘制边界框
    result_image = draw_bounding_box(image, u, v, box_size=224)

    cv2.imwrite("result_with_box1.jpg", result_image)
    cv2.imshow("Result", result_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
