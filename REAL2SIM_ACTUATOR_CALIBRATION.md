# Real2Sim Actuator Calibration

This workflow is separate from the pour-v1 mimic pipeline in `README.md`.

## Why HDF5 is 100 Hz in the mimic pipeline

The pour-v1 mimic HDF5 is a policy-training dataset, not the highest-fidelity actuator-identification log. OpenArm publishes joint states near 1000 Hz, while the Tesollo hand reference is 100 Hz. The dataset is synchronized to 100 Hz because:

- the slowest command stream for the hand is 100 Hz;
- the policy action/observation contract should see one coherent sample per control step;
- downsampling avoids repeating the same 100 Hz Tesollo command across many 1000 Hz OpenArm samples;
- DB3 remains the canonical raw source when higher-rate identification is needed.

For Real2Sim actuator identification, keep the raw DB3. Convert separate identification HDF5 files at the rate needed by the target subsystem:

- OpenArm arm response: prefer 500-1000 Hz if analyzing fast transients.
- Tesollo hand response: 100 Hz is usually sufficient because the command reference is 100 Hz.
- Combined policy validation: 100 Hz is appropriate.

## What to record

Record both arms and the Tesollo hand together, even if only one side is moving:

```bash
cd /home/user/rl_ws/teleopration_openarm_tesollo
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=126 ./src/openarm_teleop/script/record_real2sim_identification_bag.sh \
  ./bags/real2sim_identification/run_001 20
```

The script records:

- `/openarm/left/joint_states`
- `/openarm/right/joint_states`
- `/openarm/left/leader/gripper_state`
- `/openarm/right/leader/gripper_state`
- `/dg5f_right/rj_dg_pospid/reference`
- `/dg5f_right/joint_states`
- `/tesollo/right/joint_states`
- `/tesollo/right/sensor`
- `/tf`
- `/tf_static`

## Motion set

Do not rely only on free teleoperation task demos. Collect short identification sequences:

- joint/group small step commands;
- slow ramp open/close or flex/extend;
- hold at several positions;
- return-to-neutral;
- low, middle, and high workspace arm postures;
- unloaded and light-contact hand motions.

Use small amplitudes first. Avoid high-speed or large-range sweeps until safety limits are confirmed.

## Is teleoperation data enough?

Teleoperation task data is useful for validation because it contains real command, position, velocity, effort, and contact signals under the task distribution. It is not enough by itself to separate stiffness, damping, friction, and delay because the operator command is correlated with the robot response and the excitation is not controlled.

Use:

- task teleop data for final replay/validation;
- dedicated step/ramp/hold data for identifying actuator response;
- both arms recorded in all sessions so left/right coupling, base motion, and passive side behavior can be checked later.

## Calibration JSON generation

After converting the identification bag to HDF5, estimate one actuator group at a time:

```bash
python3 src/openarm_teleop/script/real2sim_actuator_calibration.py \
  --dataset datasets/real2sim_identification_100hz.hdf5 \
  --group-name tesollo_hand_curl \
  --command-dataset obs/right_hand_reference_joint_pos \
  --measured-dataset obs/right_hand_joint_pos \
  --joint-name-regex 'rj_dg_[1-5]_2' \
  --defaults 30.0,5.0,7.5,3.14159,0.0 \
  --output datasets/real2sim_actuator_calibration.json
```

Apply it to the HDGP v10 Isaac Lab task:

```bash
export OPENARM_REAL2SIM_ACTUATOR_CALIBRATION=/abs/path/to/real2sim_actuator_calibration.json
```
