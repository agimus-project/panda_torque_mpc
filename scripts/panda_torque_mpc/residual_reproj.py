import numpy as np
import pinocchio as pin
from example_robot_data import load



EPS = 1e-7

def compute_num_jac(f, x, rplus=np.add, eps=EPS):
    """
    Compute numerical jacobian wrt. to each element of x.
    """
    y = f(x)
    nx, ny = len(x), len(y)
    J = np.zeros((ny, nx))

    eps_mat = eps*np.eye(nx)

    for i in range(nx):
        x_pert = rplus(x, eps_mat[i,:])
        J[:,i] = (f(x_pert) - f(x))/eps
    
    return J


def skew(v):
    return np.array([
        [ 0   ,-v[2], v[1]],
        [ v[2],    0,-v[0]],
        [-v[1], v[0],    0],
    ])

def calc_proj(t, fx, fy, cx, cy):
    return np.array([
        cx + fx*t[0]/t[2],
        cy + fy*t[1]/t[2]
    ])

class ResidualModelReproj:

    def __init__(self, robot, T_e_c, t_o_t_ref, uv_ref, intrinsics) -> None:
        self.r = robot
        self.ee_id = self.r.model.getFrameId('panda_hand')
        self.T_e_c = T_e_c
        self.t_o_t_ref = t_o_t_ref
        self.fx, self.fy, self.cx, self.cy = intrinsics
        self.uv_ref = uv_ref

    def calc(self, q):
        pin.forwardKinematics(self.r.model, self.r.data, q)
        pin.updateFramePlacement(self.r.model, self.r.data, self.ee_id)
        self.T_o_e = self.r.data.oMf[self.ee_id]
        self.T_o_c = self.T_o_e * self.T_e_c
        self.T_c_o = self.T_o_c.inverse()
        self.t_c_t = self.T_c_o * self.t_o_t_ref

        self.proj = calc_proj(self.t_c_t, self.fx, self.fy, self.cx, self.cy)

        return self.proj - self.uv_ref

    def calcDiff(self, q):
        self.calc(q)

        # MAGIC
        J_proj_tct = calc_J_proj_tct(self.t_c_t, self.fx, self.fy)
        J_tct_Tco = calc_J_tct_Tco(self.T_c_o, self.t_o_t_ref) 
        J_Tco_Toc = calc_J_Tco_Toc(self.T_o_c)
        J_Toc_Toe = calc_J_Toc_Toe(self.T_o_e, self.T_e_c)
        J_Toe_q = calc_J_Toe_q(q, self.r, self.ee_id)

        J_proj_q = J_proj_tct @ J_tct_Tco @ J_Tco_Toc @ J_Toc_Toe @ J_Toe_q

        return J_proj_q


def calc_J_proj_tct(t, fx, fy):
    return np.array([
        [fx/t[2], 0, - fx*t[0]/t[2]**2],
        [0, fy/t[2], - fy*t[1]/t[2]**2],
    ])

def calc_J_tct_Tco(Tco, tot):
    return np.hstack([
        Tco.rotation, -Tco.rotation @ skew(tot)
    ])

def calc_J_Tco_Toc(Toc):
    return - Toc.toActionMatrix()

def calc_J_Toc_Toe(Toe, Tec):
    return Tec.toActionMatrixInverse()

def calc_J_Toe_q(q, r, ee_id):
    pin.computeJointJacobians(r.model,r.data,q)
    return pin.getFrameJacobian(r.model,r.data, ee_id, pin.LOCAL)
    # return pin.getFrameJacobian(r.model,r.data, ee_id, pin.WORLD)
    # return pin.getFrameJacobian(r.model,r.data, ee_id, pin.LOCAL_WORLD_ALIGNED)


if __name__ == '__main__':
    T_e_c = pin.SE3.Identity()
    
    # Simulating real panda setup
    pose_e_c = np.array([0.137, -0.01, -0.059,  0.0869, -0.0866, -0.70365, 0.69986])
    pose_e_c[3:] = pose_e_c[3:]/np.linalg.norm(pose_e_c[3:])
    T_e_c =  pin.XYZQUATToSE3(pose_e_c)
    t_o_t_ref = np.array([0.5, 0, 0])
    uv_ref = np.zeros(2)
    intrinsics = np.array([637, 637, 643, 376]) # TODO

    robot = load('panda')
    q = robot.q0

    res = ResidualModelReproj(robot, T_e_c, t_o_t_ref, uv_ref, intrinsics)

    J_q = res.calcDiff(q)
    calc = lambda q: res.calc(q)
    J_q_num = compute_num_jac(calc, q, eps=1e-6)

    print(J_q - J_q_num)
    # print(J_q)
    # print(J_q_num)

