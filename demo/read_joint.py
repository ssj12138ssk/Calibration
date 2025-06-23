import pyads
import numpy as np
import sympy as sy

class MDH:
    def MDH(self, alpha, a, theta, d):
        #x轴旋转变化矩阵
        rx = np.array([
            [1, 0, 0, 0],
            [0, sy.cos(alpha), -sy.sin(alpha), 0],
            [0, sy.sin(alpha), sy.cos(alpha), 0],
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


    def Aiv(self, alpha, a, d, theta, Tnum):
        for i in range(Tnum):
            Ti = self.MDH(alpha[i], a[i], theta[i], d[i])
            if i == 0:
                Ai = Ti
            else:
                Ai = Ai@Ti
        return Ai

if __name__ == "__main__":
    # q = []
    # plc = pyads.Connection('10.1.233.139.1.1', 855)
    # plc.open()
    # for i in range(14):
    #     joint = plc.read_by_name(f"HumanInterface.TestHmiDataOut.masterJoint[{i}]", pyads.PLCTYPE_REAL)
    #     q.append(joint)
    #
    # plc.close()
    q = [0.8122738599777222, -0.02604612335562706, 0.48214635252952576, 0.6865506172180176, -1.0420238971710205, -0.006002732552587986, -0.00809883326292038, 0.20299173891544342, -0.2848658263683319, -0.2658134400844574, -0.1541880965232849, -0.6508072018623352, -0.0018563289195299149, -1.4888428449630737]

    #print(q)
    alpha = [np.pi / 2, 0, -np.pi / 2, 0, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2]
    a = [0, 0, 0, 0.3, 0.35, 0, 0, 0]
    d = [0, -0.166, 0, 0, 0.145, 0, 0, 0]
    theta_left = [np.pi / 2, q[0], np.pi / 2 + q[1], -np.pi / 2 + q[2] - q[1], q[3], q[4], np.pi / 2 + q[5], np.pi + q[6]]
    theta_right = [np.pi / 2, q[7], np.pi / 2 + q[8], -np.pi / 2 + q[9] - q[8], q[10], q[11], np.pi / 2 + q[12], np.pi + q[13]]
    mdh = MDH()

    left_location = mdh.Aiv(alpha, a, d, theta_left, 7)
    right_location = mdh.Aiv(alpha, a, d, theta_right, 7)
    left_xyz = []
    right_xyz = []
    #print(left_location)
    # for i in range(len(right_location)-1):
    #     right_xyz.append(right_location[i][3])
    # print(right_xyz)
    for i in range(len(left_location)-1):
        left_xyz.append(left_location[i][3])
    #print(left_xyz)

    print(right_location)
