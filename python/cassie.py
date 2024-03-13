import pybullet as p
import time
import pinocchio as pin
from pathlib import Path

joint_names_to_actuators = {
    "hip_abduction_left": "hip_abduction_motor_left",
    "hip_rotation_left": "hip_rotation_motor_left",
    "hip_flexion_left": "hip_flexion_motor_left",
    "knee_joint_left": "knee_joint_motor_left",
    "toe_joint_left": "toe_joint_motor_left",
    "hip_abduction_right": "hip_abduction_motor_right",
    "hip_rotation_right": "hip_rotation_motor_right",
    "hip_flexion_right": "hip_flexion_motor_right",
    "knee_joint_right": "knee_joint_motor_right",
    "toe_joint_right": "toe_joint_motor_right",
}

joint_ids_to_actuators = {
    2: "hip_abduction_motor_left",
    3: "hip_rotation_motor_left",
    4: "hip_flexion_motor_left",
    5: "knee_joint_motor_left",
    8: "toe_joint_motor_left",
    10: "hip_abduction_motor_right",
    11: "hip_rotation_motor_right",
    12: "hip_flexion_motor_right",
    13: "knee_joint_motor_right",
    16: "toe_joint_motor_right",
}

models_path = Path().absolute() / "models"
plane_path = models_path / "plane"
cassie_path = models_path / "cassie"

cassie_urdf_filename = str(cassie_path / "urdf" / "cassie_collide.urdf")
plane_urdf_filename = str(plane_path / "plane.urdf")


model = pin.buildModelFromUrdf(cassie_urdf_filename)
print("model name: " + model.name)
data = model.createData()
q = pin.randomConfiguration(model)
print("q: %s" % q.T)
pin.forwardKinematics(model, data, q)
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat)))

p.connect(p.GUI)
p.loadURDF(plane_urdf_filename)
humanoid = p.loadURDF(cassie_urdf_filename, [0, 0, 0.8], useFixedBase=False)
gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=100)
p.changeDynamics(humanoid, -1, linearDamping=0, angularDamping=0)

jointAngles = [
    0,
    0,
    1.0204,
    -1.97,
    -0.084,
    2.06,
    -1.9,
    0,
    0,
    1.0204,
    -1.97,
    -0.084,
    2.06,
    -1.9,
    0,
]
activeJoint = 0
for j in range(p.getNumJoints(humanoid)):
    p.changeDynamics(humanoid, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(humanoid, j)
    jointName = info[1]
    jointType = info[2]
    if jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE:
        jointIds.append(j)
        paramIds.append(
            p.addUserDebugParameter(
                jointName.decode("utf-8"), -4, 4, jointAngles[activeJoint]
            )
        )
        p.resetJointState(humanoid, j, jointAngles[activeJoint])
        print(p.getJointInfo(humanoid, j), jointAngles[activeJoint])
        activeJoint += 1

print(p.getBasePositionAndOrientation(humanoid))
p.setRealTimeSimulation(1)
while 1:
    p.getCameraImage(320, 200)
    p.setGravity(0, 0, p.readUserDebugParameter(gravId))
    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        p.setJointMotorControl2(
            humanoid, jointIds[i], p.POSITION_CONTROL, targetPos, force=140.0
        )
    time.sleep(0.01)
    base_pos_ori = p.getBasePositionAndOrientation(humanoid)
    base_vel = p.getBaseVelocity(humanoid)
    print(base_pos_ori)
    for i, name in joint_ids_to_actuators.items():
        joint_state = p.getJointState(bodyUniqueId=humanoid, jointIndex=i)
        print(name, joint_state)
        
        
