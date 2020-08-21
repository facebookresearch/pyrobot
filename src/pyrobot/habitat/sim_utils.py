# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# This file is derived from Habitat-Sim - https://github.com/facebookresearch/habitat-sim.git

import habitat_sim
import habitat_sim.agent
import habitat_sim.bindings as hsim


# build SimulatorConfiguration
def make_cfg(SIM):
    sim_cfg = hsim.SimulatorConfiguration()

    if SIM.SCENE_ID == "none":
        SIM.SCENE_ID = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    sim_cfg.scene.id = SIM.SCENE_ID

    sim_cfg.enable_physics = SIM.PHYSICS
    if SIM.PHYSICS:
        sim_cfg.physics_config_file = SIM.PHYSICS_CONFIG_FILE
    # sim_cfg.gpu_device_id = 0
    # sim_cfg.scene.id = settings["scene"]

    # define default sensor parameters (see src/esp/Sensor/Sensor.h)
    sensors = dict()
    for i in range(len(SIM.AGENT.SENSORS.NAMES)):
        sensors[SIM.AGENT.SENSORS.NAMES[i]] = {
            "sensor_type": getattr(hsim.SensorType, SIM.AGENT.SENSORS.TYPES[i]),
            "resolution": [
                SIM.AGENT.SENSORS.RESOLUTIONS[i][0],
                SIM.AGENT.SENSORS.RESOLUTIONS[i][1],
            ],
            "position": [
                SIM.AGENT.SENSORS.POSES[i][0],
                SIM.AGENT.SENSORS.POSES[i][1],
                SIM.AGENT.SENSORS.POSES[i][2],
            ],
            "orientation": [
                SIM.AGENT.SENSORS.POSES[i][3],
                SIM.AGENT.SENSORS.POSES[i][4],
                SIM.AGENT.SENSORS.POSES[i][5],
            ],
        }

    # create sensor specifications
    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        sensor_spec = hsim.SensorSpec()
        sensor_spec.uuid = sensor_uuid
        sensor_spec.sensor_type = sensor_params["sensor_type"]
        sensor_spec.resolution = sensor_params["resolution"]
        sensor_spec.position = sensor_params["position"]
        sensor_spec.gpu2gpu_transfer = False  # Todo: Move this to config

        print("==== Initialized Sensor Spec: =====")
        print("Sensor uuid: ", sensor_spec.uuid)
        print("Sensor type: ", sensor_spec.sensor_type)
        print("Sensor position: ", sensor_spec.position)
        print("===================================")

        sensor_specs.append(sensor_spec)

    # create agent specifications
    # TODO: Accomodate more agents
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs
    # TODO: Move agent actions to config
    agent_cfg.action_space = {
        "move_forward": habitat_sim.agent.ActionSpec(
            "move_forward", habitat_sim.agent.ActuationSpec(amount=1.0)
        ),
        "turn_left": habitat_sim.agent.ActionSpec(
            "turn_left", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
        "turn_right": habitat_sim.agent.ActionSpec(
            "turn_right", habitat_sim.agent.ActuationSpec(amount=10.0)
        ),
    }
    sim_cfg.default_agent_id = SIM.DEFAULT_AGENT_ID
    # # override action space to no-op to test physics
    # if sim_cfg.enable_physics:
    #     agent_cfg.action_space = {
    #         "move_forward": habitat_sim.agent.ActionSpec(
    #             "move_forward", habitat_sim.agent.ActuationSpec(amount=0.0)
    #         )
    #     }

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])
