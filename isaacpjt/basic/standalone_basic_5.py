from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import time
import omni.usd

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid


world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()


cube_prim = DynamicCuboid(
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 0.15]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)


world.scene.add_default_ground_plane()
world.scene.add(cube_prim)

world.reset()


# -------------------------
# 루프 제어

step_count = 0
reset_needed = False


while simulation_app.is_running():

    world.step(render=True)
    time.sleep(0.01)


    # Stop 상태 감지
    if world.is_stopped():
        reset_needed = True


    # Play 상태일 때만 실행
    if world.is_playing():

        # Stop 후 다시 Play를 눌렀을 경우
        if reset_needed:

            world.reset()

            step_count = 0
            reset_needed = False

            print("[리셋] 처음부터 다시 시작")


        # Step 증가
        step_count += 1


        # 100 Step마다 확인
        if step_count % 100 == 0:
            print("step:", step_count)


        # 300 Step이 되면 순간이동
        if step_count == 300:

            cube_prim.set_world_pose(
                position=np.array([0.0, 0.0, 1.0])
            )

            print("[이동] 큐브를 Z=1m로 이동")


simulation_app.close()