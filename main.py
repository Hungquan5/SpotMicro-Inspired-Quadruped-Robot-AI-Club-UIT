import pybullet as p
import pybullet_data
import time
import numpy as np
import math
from math import *

# ==========================================================
# 1. KINEMATICS (Core Calculation)
# ==========================================================
class Kinematics(object):
    def __init__(self):
        # Leg segment lengths in millimeters
        self.l1 = 50   # Shoulder
        self.l2 = 20   # Leg top
        self.l3 = 100  # Thigh
        self.l4 = 100  # Shin
        self.L = 140   # Body Length
        self.W = 75    # Body Width

    def bodyIK(self, omega, phi, psi, xm, ym, zm):
        Rx = np.array([[1, 0, 0, 0],
                       [0, np.cos(omega), -np.sin(omega), 0],
                       [0, np.sin(omega), np.cos(omega), 0], [0, 0, 0, 1]])
        Ry = np.array([[np.cos(phi), 0, np.sin(phi), 0],
                       [0, 1, 0, 0],
                       [-np.sin(phi), 0, np.cos(phi), 0], [0, 0, 0, 1]])
        Rz = np.array([[np.cos(psi), -np.sin(psi), 0, 0],
                       [np.sin(psi), np.cos(psi), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Rxyz = Rx.dot(Ry.dot(Rz))

        T = np.array([[0, 0, 0, xm], [0, 0, 0, ym], [0, 0, 0, zm], [0, 0, 0, 0]])
        Tm = T + Rxyz

        sHp = np.sin(pi / 2)
        cHp = np.cos(pi / 2)
        (L, W) = (self.L, self.W)

        # Calculate Corner Transformation Matrices
        return ([Tm.dot(np.array([[cHp, 0, sHp, L / 2], [0, 1, 0, 0], [-sHp, 0, cHp, W / 2], [0, 0, 0, 1]])),
                 Tm.dot(np.array([[cHp, 0, sHp, L / 2], [0, 1, 0, 0], [-sHp, 0, cHp, -W / 2], [0, 0, 0, 1]])),
                 Tm.dot(np.array([[cHp, 0, sHp, -L / 2], [0, 1, 0, 0], [-sHp, 0, cHp, W / 2], [0, 0, 0, 1]])),
                 Tm.dot(np.array([[cHp, 0, sHp, -L / 2], [0, 1, 0, 0], [-sHp, 0, cHp, -W / 2], [0, 0, 0, 1]]))])

    def legIK(self, point):
        (x, y, z) = (point[0], point[1], point[2])
        (l1, l2, l3, l4) = (self.l1, self.l2, self.l3, self.l4)
        
        try:
            F = sqrt(x ** 2 + y ** 2 - l1 ** 2)
        except ValueError:
            F = l1
            
        G = F - l2
        H = sqrt(G ** 2 + z ** 2)
        
        theta1 = -atan2(y, x) - atan2(F, -l1)

        D = (H ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4)
        
        if D > 1.0: D = 1.0
        if D < -1.0: D = -1.0
            
        try:
            theta3 = acos(D)
        except ValueError:
            theta3 = 0
            
        theta2 = atan2(z, G) - atan2(l4 * sin(theta3), l3 + l4 * cos(theta3))

        return (theta1, theta2, theta3)

    def calcIK(self, Lp, angles, center):
        (omega, phi, psi) = angles
        (xm, ym, zm) = center

        (Tlf, Trf, Tlb, Trb) = self.bodyIK(omega, phi, psi, xm, ym, zm)

        Ix = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        # Order: FL, FR, BL, BR
        return np.array([
            self.legIK(np.linalg.inv(Tlf).dot(Lp[0])),
            self.legIK(Ix.dot(np.linalg.inv(Trf).dot(Lp[1]))),
            self.legIK(np.linalg.inv(Tlb).dot(Lp[2])),
            self.legIK(Ix.dot(np.linalg.inv(Trb).dot(Lp[3])))
        ])

# ==========================================================
# 2. GAIT GENERATOR
# ==========================================================
class TrottingGait:
    def __init__(self):
        # Timing
        self.t0 = 0
        self.t1 = 510
        self.t2 = 0
        self.t3 = 185
        
        # Parameters
        self.Sl = 0
        self.Sw = 0
        self.Sh = 60
        self.Sa = 0
        self.Spf = 87
        self.Spr = 77
        self.Fx = 120
        self.Rx = 50 

    def calcLeg(self, t, x, y, z):
        startLp = np.array([x - self.Sl / 2.0, y, z - self.Sw, 1])
        endLp = np.array([x + self.Sl / 2, y, z + self.Sw, 1])

        if t < self.t0:
            return startLp
        elif t < self.t0 + self.t1:
            td = t - self.t0
            tp = td / self.t1 if self.t1 > 0 else 0
            diffLp = endLp - startLp
            curLp = startLp + diffLp * tp
            
            psi = -((math.pi / 180 * self.Sa) / 2) + (math.pi / 180 * self.Sa) * tp
            Ry = np.array([[np.cos(psi), 0, np.sin(psi), 0],
                           [0, 1, 0, 0],
                           [-np.sin(psi), 0, np.cos(psi), 0], [0, 0, 0, 1]])
            curLp = Ry.dot(curLp)
            return curLp
        elif t < self.t0 + self.t1 + self.t2:
            return endLp
        elif t < self.t0 + self.t1 + self.t2 + self.t3:
            td = t - (self.t0 + self.t1 + self.t2)
            tp = td / self.t3 if self.t3 > 0 else 0
            diffLp = startLp - endLp
            curLp = endLp + diffLp * tp
            curLp[1] += self.Sh * math.sin(math.pi * tp)
            return curLp
        else:
            return startLp

    def positions(self, t, current_y):
        Tt = (self.t0 + self.t1 + self.t2 + self.t3)
        Tt2 = Tt / 2
        
        td = (t * 1000) % Tt
        t2 = (t * 1000 - Tt2) % Tt
        rtd = (t * 1000) % Tt
        rt2 = (t * 1000 - Tt2) % Tt
        
        Fx = self.Fx
        Rx = -self.Rx # Rear is negative
        
        # Y is Height here (-100 etc)
        Fy = current_y
        Ry = current_y

        return np.array([
            self.calcLeg(td,  Fx, Fy,  self.Spf), # FL
            self.calcLeg(t2,  Fx, Fy, -self.Spf), # FR (Negative width)
            self.calcLeg(rt2, Rx, Ry,  self.Spr), # BL
            self.calcLeg(rtd, Rx, Ry, -self.Spr)  # BR (Negative width)
        ])

# ==========================================================
# 3. HELPER FUNCTION
# ==========================================================
def lerp(start, end, alpha):
    return start + (end - start) * alpha

# ==========================================================
# 4. MAIN SIMULATION
# ==========================================================
def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    
    # Physics Parameters
    p.setPhysicsEngineParameter(numSolverIterations=200)

    # Load Floor and Robot
    planeId = p.loadURDF("plane.urdf")
    
    # *** IMPORTANT: UPDATE PATH ***
    urdf_path = '/home/quannh/SpotMicro-Inspired-Quadruped-Robot-AI-Club-UIT/simulation/Basic simulation by user Florian Wilk/urdf/spotmicroai_gen_ros.urdf'
    robotId = p.loadURDF(urdf_path, [0, 0, 0.5], p.getQuaternionFromEuler([0, 0, 0]))

    # --- SETUP JOINTS AND DIRECTIONS ---
    # NOTE: The order here must be [FL, FR, BL, BR]
    # Each leg has [Shoulder, Leg, Foot]
    joint_names_ordered = [
        'front_left_shoulder', 'front_left_leg', 'front_left_foot',
        'front_right_shoulder', 'front_right_leg', 'front_right_foot',
        'rear_left_shoulder', 'rear_left_leg', 'rear_left_foot',
        'rear_right_shoulder', 'rear_right_leg', 'rear_right_foot'
    ]
    
    joint_indices = []
    num_joints = p.getNumJoints(robotId)
    joint_dict = {}
    for i in range(num_joints):
        info = p.getJointInfo(robotId, i)
        joint_name = info[1].decode('utf-8')
        joint_dict[joint_name] = i

    for name in joint_names_ordered:
        if name in joint_dict:
            joint_indices.append(joint_dict[name])
        else:
            print(f"Warning: Joint {name} not found!")

    # --- DIRECTION MATRIX FROM REFERENCE CODE ---
    # Row 0: FL, Row 1: FR, Row 2: BL, Row 3: BR
    # Cols: Shoulder, Leg, Foot
    # This inverts the left shoulders/legs to match right side logic or vice versa
    dirs = [
        [-1, 1, 1], # FL
        [ 1, 1, 1], # FR
        [-1, 1, 1], # RL
        [ 1, 1, 1]  # RR
    ]

    # Dynamics
    kp = 0.045
    kd = 0.4
    maxForce = 12.5
    for i in range(num_joints):
        p.changeDynamics(robotId, i, localInertiaDiagonal=[1e-6, 1e-6, 1e-6])

    # Logic
    kin = Kinematics()
    gait = TrottingGait()

    # Camera
    p.resetDebugVisualizerCamera(1.0, 50, -35, [0,0,0])

    print("=========================================")
    print(" CONTROLS:")
    print(" [Q] STAND UP")
    print(" [E] LAY DOWN")
    print(" ARROW KEYS: Move")
    print("=========================================")

    # Heights (Y in Kinematics frame)
    HEIGHT_REST = -100.0
    HEIGHT_STAND = -170.0
    
    target_y = HEIGHT_REST
    current_y = HEIGHT_REST
    t_gait = 0
    
    while p.isConnected():
        keys = p.getKeyboardEvents()
        
        # Inputs
        if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
            target_y = HEIGHT_STAND
        if ord('e') in keys and keys[ord('e')] & p.KEY_WAS_TRIGGERED:
            target_y = HEIGHT_REST

        vx = 0.0
        wz = 0.0
        
        # Only move if standing
        if current_y < -100:
            if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
                vx = -1.0
            if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
                vx = 1.0
            if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
                wz = 1.0
            if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                wz = -1.0

        # Smooth Height
        current_y = lerp(current_y, target_y, 0.05)

        # Gait Param Update
        gait.Sl = vx * 124 
        gait.Sa = wz * 20
        
        if abs(vx) > 0.1 or abs(wz) > 0.1:
            t_gait += 0.015

        # 1. Get Positions
        feet_pos = gait.positions(t_gait, current_y)
        
        # 2. Get Angles (Raw)
        raw_angles = kin.calcIK(feet_pos, (0,0,0), (0,0,0))
        
        # 3. Apply Direction Corrections (The Fix)
        final_angles = []
        
        # raw_angles is 4x3 (4 Legs, 3 Joints)
        # dirs is 4x3
        for i in range(4): # Loop Legs
            for j in range(3): # Loop Joints (Shoulder, Leg, Foot)
                # Multiply angle by direction
                corrected_angle = raw_angles[i][j] * dirs[i][j]
                final_angles.append(corrected_angle)

        # 4. Motor Control
        p.setJointMotorControlArray(
            bodyUniqueId=robotId,
            jointIndices=joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=final_angles,
            positionGains=[kp]*12  ,
            velocityGains=[kd]*12,
            forces=[maxForce]*12
        )

        p.stepSimulation()
        
        pos, _ = p.getBasePositionAndOrientation(robotId)
        p.resetDebugVisualizerCamera(1.0, 50, -35, pos)
        
        time.sleep(1./240.)

if __name__ == "__main__":
    main()