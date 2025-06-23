import numpy as np
import sympy as sy



class MDH:
    def __init__(self):
        self.q = []*7
        self.alpha = [np.pi / 2, 0, -np.pi / 2, 0, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2]
        self.a = [0, 0, 0, 0.3, 0.35, 0, 0, 0]
        self.d = [0, -0.166, 0, 0, 0.145, 0, 0, 0]
        self.theta = [np.pi / 2, q[0], np.pi / 2 + q[1], -np.pi / 2 + q[2], q[3], q[4], np.pi / 2 + q[5], np.pi + q[6]]



    def MDH(self, alpha, a, theta, d):
        #x轴旋转变化矩阵
        rx = sy.Matrix([
            [1, 0, 0, 0],
            [0, sy.cos(alpha), -sy.sin(alpha), 0],
            [0, sy.sin(alpha), sy.cos(alpha), 0],
            [0, 0, 0, 1]
            ])
        #沿着x轴平移
        tx = sy.Matrix([
            [1, 0, 0, a],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        #沿着z轴旋转
        rz = sy.Matrix([
            [sy.cos(theta), -sy.sin(theta), 0, 0],
            [sy.sin(theta), sy.cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        #沿着z轴平移，有方向
        tz = sy.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]
        ])
        return rx@tx@rz@tz


    def Aiv(self, alpha, a, d, theta, Tnum):
        for i in range(Tnum):
            Ti = MDH(alpha[i], a[i], theta[i], d[i])
            if i == 0:
                Ai = Ti
            else:
                Ai = Ai@Ti
        return Ai


if __name__ == "__main__":
    # z轴旋转角度
    alpha1 = sy.Symbol("alpha1")
    # z轴平移
    a1 = sy.Symbol("a1")
    # x轴旋转角度
    theta1 = sy.Symbol("theta1")
    # x轴平移
    d1 = sy.Symbol("d1")

    test = MDH(alpha1, a1, theta1, d1)
    print(test)
    Tnum8robot = MDH.Aiv(alpha1, a1, d1, theta1, 7)
    print(Tnum8robot)

