import numpy as np
from numpy.linalg import inv
from numba import float64, int32
from numba.experimental import jitclass
import numpy as np
# from numba import njit




# %%
NUM = 2000
GNE= 180
n_offsprings = 500



DTT = 0.01
G_EXP_POS = np.array([0.2,0.2,0.2])
exp_yaw = 0

# %%

# # %% 通用函数
# @njit
def euler_to_quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return np.array([qw, qx, qy, qz])

# @njit
def quaternion_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

# @njit
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

# @njit
def rotate_vector_by_quaternion(v, q):
    v_quat = np.zeros(4)  # 创建一个长度为4的零数组
    v_quat[0] = 0
    v_quat[1] = v[0]
    v_quat[2] = v[1]
    v_quat[3] = v[2]
    q_conj = quaternion_conjugate(q)
    return quaternion_multiply(quaternion_multiply(q, v_quat), q_conj)[1:]

# @njit
def global_to_body_vector(v_global, roll, pitch, yaw):
    q = euler_to_quaternion(roll, pitch, yaw)
    return rotate_vector_by_quaternion(v_global, quaternion_conjugate(q))

# @njit
def body_to_global_vector(v_body, roll, pitch, yaw):
    q = euler_to_quaternion(roll, pitch, yaw)
    return rotate_vector_by_quaternion(v_body, q)


# @njit
def NP_sigmoid_M(x):
    if x<0:
        return 0
    elif x>1:
        return 1
    else:
        return x




class PID_C:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0

        self.ORIGN_P = kp
        self.ORIGN_I = ki
        self.ORIGN_F = kd

    def update_ACT(self, error):
        derivative = (error - self.previous_error) / self.dt
        self.integral += error * self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        self.derivative = derivative
        return output



class FLY_CONTROL_PID():
    def __init__(self,
                 kp_1, ki_1, kd_1,
                 kp_2, ki_2, kd_2,
                 kp_3, ki_3, kd_3,
                 kp_4, ki_4, kd_4,
                 kp_5, ki_5, kd_5,
                 kp_6, ki_6, kd_6,dt,IS_REPORT = False):
        self.dt = dt

        # 创建PID控制器实例
        self.pid_c1 = PID_C(kp=kp_1, ki=ki_1, kd=kd_1, dt=dt)  # 用于A_P1[0]和A_P2[0]的误差 -> EXP_roll
        self.pid_c2 = PID_C(kp=kp_2, ki=ki_2, kd=kd_2, dt=dt)  # 用于A_P1[1]和A_P2[1]的误差 -> EXP_pitch
        self.pid_c3 = PID_C(kp=kp_3, ki=ki_3, kd=kd_3, dt=dt)  # 用于EXP_YAW和当前yaw的误差 -> CMD_YAW
        self.pid_c4 = PID_C(kp=kp_4, ki=ki_4, kd=kd_4, dt=dt)  # 用于g_p1_z和g_p2_z的误差 -> CMD_Thrust
        self.pid_c5 = PID_C(kp=kp_5, ki=ki_5, kd=kd_5, dt=dt)  # 用于EXP_roll和当前roll的误差 -> CMD_ROLL
        self.pid_c6 = PID_C(kp=kp_6, ki=ki_6, kd=kd_6, dt=dt)  # 用于EXP_pitch和当前pitch的误差 -> CMD_PITCH

        self.IS_REPORT = IS_REPORT

    def GENE_control_commands(self, G_REAL_POS, G_REAL_ANG, G_EXP_POS, exp_yaw):
        """
        :param a_p1: 当前位置
        :param current_attitude: 当前姿态
        :param a_p2: 期望位置
        :param exp_yaw: 期望偏航角
        :return:
        """

        # 从姿态角信息中提取 roll, pitch, yaw
        roll = G_REAL_ANG[0]
        pitch = G_REAL_ANG[1]
        yaw = G_REAL_ANG[2]
        real_height = G_REAL_POS[2]
        exp_height = G_EXP_POS[2]

        # 坐标系分析(将地面坐标系的点转换到机体坐标系,进而实现表面上的解耦)
        B_REAL_POS = global_to_body_vector(G_REAL_POS, roll, pitch,yaw)
        B_EXP_POS  = global_to_body_vector(G_EXP_POS, roll,pitch,yaw)

        # 计算期望滚转角 EXP_roll 和期望俯仰角 EXP_pitch
        exp_roll = - self.pid_c1.update_ACT(B_EXP_POS[1] - B_REAL_POS[1])   # 这里要添加负号，因为坐标系定义问题
        exp_pitch = self.pid_c2.update_ACT(B_EXP_POS[0] - B_REAL_POS[0])

        # 计算期望偏航命令 CMD_YAW
        cmd_yaw = self.pid_c5.update_ACT(exp_yaw - yaw)

        # 计算推力命令 CMD_Thrust
        cmd_thrust = self.pid_c6.update_ACT(exp_height - real_height)

        # 计算滚转控制命令 CMD_ROLL 和俯仰控制命令 CMD_PITCH
        cmd_roll = self.pid_c3.update_ACT(exp_roll - roll)
        cmd_pitch = self.pid_c4.update_ACT(exp_pitch - pitch)

        if abs(cmd_thrust)>1 or abs(cmd_roll)>1 or abs(cmd_pitch)>1 or abs(cmd_yaw)>1:
            FLAG = True
        else:
            FLAG = False

        if self.IS_REPORT:
            print("---",FLAG,"原始指令",cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw,)


        cmd_roll   = np.tanh(cmd_roll)         # 缩放到[-1,+1]
        cmd_pitch  = np.tanh(cmd_pitch)        # 缩放到[-1,+1]
        cmd_yaw    = np.tanh(cmd_yaw)          # 缩放到[-1,+1]
        cmd_thrust = np.tanh(cmd_thrust)  # 缩放到[0,+1]


        return cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw,FLAG

# %% 验证过程

G_EXP_POS = np.array([0, 0, 1])  # 目标位置
exp_yaw = 0.0  # 目标偏航角

# 示例数据
G_REAL_POS = np.array([0, 0, 0])  # 全局位置1
G_REAL_ANG = np.array([0, 0, 0])  # 当前姿态角 [roll, pitch, yaw]


FC = FLY_CONTROL_PID(kp_1 = 1.0, ki_1 = 1.0, kd_1 = 1.0,
                     kp_2 = 1.0, ki_2 = 1.0, kd_2 = 1.0,
                     kp_3 = 1.0, ki_3 = 1.0, kd_3 = 1.0,
                     kp_4 = 1.0, ki_4 = 1.0, kd_4 = 1.0,
                     kp_5 = 1.0, ki_5 = 1.0, kd_5 = 1.0,
                     kp_6 = 1.0, ki_6 = 1.0, kd_6 = 1.0,
                     dt = 0.01)

# 计算控制命令
cmd_thrust, cmd_roll, cmd_pitch, cmd_yaw,FLAG = FC.GENE_control_commands(G_REAL_POS, G_REAL_ANG, G_EXP_POS, exp_yaw)

print("CMD_THRUST:", cmd_thrust)
print("CMD_ROLL:", cmd_roll)
print("CMD_PITCH:", cmd_pitch)
print("CMD_YAW:", cmd_yaw)