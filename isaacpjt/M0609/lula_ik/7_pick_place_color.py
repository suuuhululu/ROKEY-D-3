"""
Pick & Place — 큐브 접근 후 대기 & ROS2 외부 명령 수신 대기 & 무한 자동 리셋 (최신 API 반영)

수정 사항:
 1. 로봇 작업이 DONE 상태에 도달하면 1초 대기 후 자동으로 씬을 리셋하는 로직 추가
 2. 사람이 수동으로 Play 버튼을 누르거나 정지시키지 않는 한 무한히 작업을 반복합니다.
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

from pathlib import Path
import time
import random
import numpy as np
import omni.usd

import rclpy
from std_msgs.msg import Int32

from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)
from isaacsim.core.api.objects import DynamicCuboid, VisualCylinder

# ══════════════════════════════════════════════════════════════
#  전역 변수 (외부 명령 수신용)
# ══════════════════════════════════════════════════════════════
global_color_id = None

def color_id_callback(msg):
    global global_color_id
    global_color_id = msg.data

# ══════════════════════════════════════════════════════════════
#  경로 및 기본 설정
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH         = str(M0609_DIR / "Collected_m0609_camera_cube/m0609_camera_cube.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

ROBOT_BASE_POS  = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])
READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 0.9
TCP_OFFSET = np.array([0.0, 0.0, 0.19671])

CUBE_SIZE = 0.04
WORKSPACE_X = (0.35, 0.55)
WORKSPACE_Y = (-0.20, 0.20)
PLACE_BLUE_XY  = np.array([0.45, -0.35])
PLACE_GREEN_XY = np.array([0.45,  0.35])

PICK_Z          = 0.025  
PLACE_Z         = 0.03
APPROACH_HEIGHT = 0.25
LIFT_HEIGHT     = 0.23

GRIPPER_WAIT = 150
TCP_SPEED  = 0.004
MIN_STEPS  = 60
MAX_STEPS  = 600

APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0

# ══════════════════════════════════════════════════════════════
#  수학 유틸
# ══════════════════════════════════════════════════════════════
def quat_from_axis(axis, deg):
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    return np.concatenate([[np.cos(half)], (a / np.linalg.norm(a)) * np.sin(half)])

def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])

def make_target_quat(roll, pitch, yaw):
    q = quat_mul(quat_from_axis([1,0,0], roll), quat_from_axis([0,1,0], pitch))
    return quat_mul(q, quat_from_axis([0,0,1], yaw))

def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def tcp_to_flange(tcp_pos, quat):
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET

def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET

def steps_for(start, goal):
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist

def lerp(start, goal, alpha):
    return start + alpha * (goal - start)

# ══════════════════════════════════════════════════════════════
#  상태 기계
# ══════════════════════════════════════════════════════════════
class PickPlaceFSM:
    NAMES = ["APPROACH", "DESCEND", "WAIT_CMD", "GRASP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "DONE"]
    GRIPPER_STATES = {3: "close", 7: "open"}
    DONE_STATE = 8
    WAIT_STATE = 2

    def __init__(self, robot):
        self._robot = robot
        self.reset(np.array([0.4, 0.0]))

    def reset(self, pick_xy):
        self.pick_xy = pick_xy
        self.place_xy = None
        self.state = 0
        self.step = 0
        self.start = None
        
        px, py = self.pick_xy
        self.waypoints = {
            0: np.array([px, py, APPROACH_HEIGHT]), # APPROACH
            1: np.array([px, py, PICK_Z]),          # DESCEND
            2: np.array([px, py, PICK_Z]),          # WAIT_CMD
            3: np.array([px, py, PICK_Z]),          # GRASP
            4: np.array([px, py, LIFT_HEIGHT]),     # LIFT
        }
        
        self.goal = self.waypoints[0]
        self.n_steps = MIN_STEPS
        self.gripper = "open"

    def set_place_target(self, place_xy):
        self.place_xy = place_xy
        gx, gy = self.place_xy
        self.waypoints[5] = np.array([gx, gy, LIFT_HEIGHT]) # MOVE
        self.waypoints[6] = np.array([gx, gy, PLACE_Z])     # LOWER
        self.waypoints[7] = np.array([gx, gy, PLACE_Z])     # RELEASE

    def current_target(self):
        if self.start is None:
            return self.goal
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self):
        if self.state >= self.DONE_STATE:
            return

        if self.state == self.WAIT_STATE:
            if self.step % 60 == 0:
                print("   [WAIT_CMD] Waiting for /color_id ... (1: Blue, 2: Green)")
            self.step += 1
            
            if global_color_id in [1, 2]:
                print(f"\n   [WAIT_CMD] Command '{global_color_id}' received! Executing Place.")
                if global_color_id == 1:
                    self.set_place_target(PLACE_BLUE_XY)
                elif global_color_id == 2:
                    self.set_place_target(PLACE_GREEN_XY)
                self._next()
            return

        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]
            self.gripper = self.GRIPPER_STATES.get(self.state, self.gripper)

            if self.state in self.GRIPPER_STATES:
                self.n_steps = GRIPPER_WAIT
            else:
                self.n_steps, _ = steps_for(self.start, self.goal)

        self.step += 1
        if self.step >= self.n_steps:
            self._next()

    def _next(self):
        self.state += 1
        self.step = 0
        self.start = None
        if self.state == self.DONE_STATE:
            print("   [DONE] Task Completed. Preparing for next cycle...")

# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
# ══════════════════════════════════════════════════════════════
def find_prim_path(root_path, name):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid(): return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name: return str(prim.GetPath())
    return None

class M0609Task(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._setup_arm_drives()
        self._register_robot(scene)
        self._add_objects(scene)
        print("   scene        ready")

    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid(): UsdGeom.Xform.Define(stage, "/World")
        stage.GetPrimAtPath("/World").GetReferences().AddReference(USD_PATH)
        for _ in range(15): simulation_app.update()

    def _setup_arm_drives(self):
        stage = omni.usd.get_context().get_stage()
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() not in ARM_JOINTS: continue
            for drive_type in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)

    def _register_robot(self, scene):
        ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
        gripper = ParallelGripper(
            end_effector_prim_path=ee_path, joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array([GRIPPER_OPEN_POS]*2),
            joint_closed_positions=np.array([GRIPPER_CLOSE_POS]*2), action_deltas=None
        )
        self._robot = scene.add(SingleManipulator(
            prim_path=ROBOT_PRIM_PATH, name="m0609_robot",
            end_effector_prim_path=ee_path, gripper=gripper
        ))

    def _add_objects(self, scene):
        self.green_marker = scene.add(VisualCylinder(
            prim_path="/World/GreenMarker", name="green_marker",
            position=np.array([PLACE_GREEN_XY[0], PLACE_GREEN_XY[1], 0.001]),
            radius=0.06, height=0.002, color=np.array([0.0, 1.0, 0.0])
        ))
        self.blue_marker = scene.add(VisualCylinder(
            prim_path="/World/BlueMarker", name="blue_marker",
            position=np.array([PLACE_BLUE_XY[0], PLACE_BLUE_XY[1], 0.001]),
            radius=0.06, height=0.002, color=np.array([0.0, 0.0, 1.0])
        ))
        self.green_cube = scene.add(DynamicCuboid(
            prim_path="/World/GreenCube", name="green_cube", position=np.array([10.0, 10.0, -10.0]),
            scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]), color=np.array([0.0, 1.0, 0.0]), mass=0.1
        ))
        self.blue_cube = scene.add(DynamicCuboid(
            prim_path="/World/BlueCube", name="blue_cube", position=np.array([10.0, 10.0, -10.0]),
            scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]), color=np.array([0.0, 0.0, 1.0]), mass=0.1
        ))

    @property
    def robot(self): return self._robot

def create_ik_solver(robot):
    lula = LulaKinematicsSolver(robot_description_path=DESCRIPTION_PATH, urdf_path=URDF_PATH)
    lula.set_robot_base_pose(robot_position=ROBOT_BASE_POS, robot_orientation=ROBOT_BASE_QUAT)
    return ArticulationKinematicsSolver(robot_articulation=robot, kinematics_solver=lula, end_effector_frame_name=EE_LINK_NAME)

# ══════════════════════════════════════════════════════════════
#  메인 루프
# ══════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = rclpy.create_node("isaac_pick_place_node")
    node.create_subscription(Int32, "/color_id", color_id_callback, 10)

    world = World(stage_units_in_meters=1.0)
    task = M0609Task(name="m0609_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view, articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions, set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )

    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)
    
    for _ in range(30): world.step(render=True)

    ik_solver = create_ik_solver(robot)
    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)

    fsm = PickPlaceFSM(robot)
    was_playing = False
    
    # [추가] 자동 리셋을 위한 타이머 변수
    auto_reset_step = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)
        
        rclpy.spin_once(node, timeout_sec=0.0)

        is_playing = world.is_playing()
        
        # [추가] 작업이 끝난 상태(DONE_STATE)인지 확인하고 대기 카운트를 올립니다.
        need_auto_reset = False
        if is_playing and fsm.state == fsm.DONE_STATE:
            auto_reset_step += 1
            if auto_reset_step > 150:  # 약 1초 정도 대기 후 리셋 (시뮬레이션 스텝 기준)
                need_auto_reset = True
                auto_reset_step = 0

        # 사람이 Play 버튼을 처음 누를 때 OR 자동 리셋 조건 충족 시
        if (is_playing and not was_playing) or need_auto_reset:
            global global_color_id
            global_color_id = None 

            world.reset()
            robot.initialize()
            
            robot.gripper.initialize(
                physics_sim_view=world.physics_sim_view, articulation_apply_action_func=robot.apply_action,
                get_joint_positions_func=robot.get_joint_positions, set_joint_positions_func=robot.set_joint_positions,
                dof_names=robot.dof_names,
            )
            robot.set_joint_positions(q)

            cube_x = np.random.uniform(*WORKSPACE_X)
            cube_y = np.random.uniform(*WORKSPACE_Y)
            random_pick_xy = np.array([cube_x, cube_y])

            is_green = random.choice([True, False])
            active_cube = task.green_cube if is_green else task.blue_cube
            hidden_cube = task.blue_cube if is_green else task.green_cube

            active_cube.set_world_pose(position=np.array([cube_x, cube_y, CUBE_SIZE / 2.0]))
            active_cube.set_linear_velocity(np.zeros(3))
            active_cube.set_angular_velocity(np.zeros(3))
            hidden_cube.set_world_pose(position=np.array([10.0, 10.0, -10.0]))
            hidden_cube.set_linear_velocity(np.zeros(3))
            hidden_cube.set_angular_velocity(np.zeros(3))

            fsm.reset(random_pick_xy)
            print(f"\n[NEW TASK] Moved to Cube at {random_pick_xy}. Waiting for /color_id ...")

        # [수정] 작업이 덜 끝났을 때만 로봇을 제어하여 불필요한 IK 연산 방지
        elif is_playing and fsm.state < fsm.DONE_STATE:
            target_tcp = fsm.current_target()
            flange_target = tcp_to_flange(target_tcp, target_quat)

            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target, target_orientation=target_quat,
            )
            if solved:
                robot.apply_action(action)

            robot.apply_action(robot.gripper.forward(action=fsm.gripper))
            fsm.advance()

        was_playing = is_playing

    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    main()