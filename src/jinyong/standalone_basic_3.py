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
    prim_path="/World/BlueCube",
    name="blue_cube",
    position=np.array([0.0, 0.0, 0.5]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([0.0, 0.0, 1.0]),
)
world.scene.add_default_ground_plane()
world.scene.add(cube_prim)
world.reset()

teleport_step = 300

step_count = 0
was_playing = False

while simulation_app.is_running():
    world.step(render=True)
    time.sleep(0.01)

    is_playing = world.is_playing()

    # Stop → Play 전환된 순간에만 리셋
    if is_playing and not was_playing:
        step_count = 0
        cube_prim.set_world_pose(position=np.array([0.0, 0.0, 0.5]))
        print(f"[리셋] Play 시작 → step_count = {step_count}")

    if is_playing:
        step_count += 1

        # 100스텝마다 로그 출력 (화면 콘솔처럼)
        if step_count % 100 == 0:
            print(f"step: {step_count}")

        # 300스텝에 순간이동
        if step_count == teleport_step:
            cube_prim.set_world_pose(position=np.array([0.0, 0.0, 1.0]))
            print("[이동] 큐브 순간이동")

    was_playing = is_playing

simulation_app.close()