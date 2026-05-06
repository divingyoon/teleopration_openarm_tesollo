.. _teleoperation-imitation-learning:

Isaac Lab Mimic을 이용한 텔레오퍼레이션과 모방 학습
====================================================


텔레오퍼레이션
~~~~~~~~~~~~~~

로봇 제어를 위해 SE(2) 및 SE(3) 공간에서 명령을 제공하는 인터페이스를 제공합니다.
SE(2) 텔레오퍼레이션의 경우 반환되는 명령은 선형 x-y 속도와 yaw rate이며,
SE(3)의 경우 반환되는 명령은 pose 변화량을 나타내는 6차원 벡터입니다.

.. note::

   현재 Isaac Lab Mimic은 Linux에서만 지원됩니다.

키보드 장치로 inverse kinematics(IK) 제어를 실행하려면 다음을 사용합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --num_envs 1 --teleop_device keyboard

더 부드러운 조작과 축에서 벗어난 조작에는 SpaceMouse를 입력 장치로 사용하는 것을 권장합니다.
더 부드러운 demonstration을 제공하면 policy가 해당 행동을 clone하기 쉬워집니다.
SpaceMouse를 사용하려면 teleop device만 다음처럼 변경하면 됩니다.

.. code:: bash

   ./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --num_envs 1 --teleop_device spacemouse

.. note::

   SpaceMouse가 감지되지 않으면 연결된 SpaceMouse의 device index에 해당하는 ``<#>``를 사용해
   ``sudo chmod 666 /dev/hidraw<#>``를 실행하여 추가 사용자 권한을 부여해야 할 수 있습니다.

   device index를 확인하려면 ``ls -l /dev/hidraw*``를 실행하여 모든 ``hidraw`` 장치를 나열합니다.
   이전 단계에서 나열된 각 장치에 대해 ``cat /sys/class/hidraw/hidraw<#>/device/uevent``를 실행하여
   SpaceMouse에 해당하는 장치를 식별합니다.

   SpaceMouse를 사용하려면 Isaac Lab의 local deployment를 사용하는 것을 권장합니다.
   container deployment(:ref:`deployment-docker`)를 사용하는 경우 ``docker-compose.yaml`` 파일에
   장치 경로를 포함하는 ``devices`` 속성을 추가하여 SpaceMouse를 ``isaac-lab-base`` 컨테이너에
   수동으로 mount해야 합니다.

   .. code:: yaml

      devices:
         - /dev/hidraw<#>:/dev/hidraw<#>

   여기서 ``<#>``는 연결된 SpaceMouse의 device index입니다.

   IsaacLab + CloudXR container deployment(:ref:`cloudxr-teleoperation`)를 사용하는 경우
   ``docker/docker-compose.cloudxr-runtime.patch.yaml`` 파일의 ``services -> isaac-lab-base`` 섹션 아래에
   ``devices`` 속성을 추가할 수 있습니다.

   Isaac Lab은 3Dconnexion의 SpaceMouse Wireless 및 SpaceMouse Compact 모델과만 호환됩니다.


hand tracking이 있는 extended reality(XR) 장치를 사용하면 이점이 있는 task의 경우,
Isaac Lab은 NVIDIA CloudXR을 사용해 장면을 호환 XR 장치로 몰입형 스트리밍하여 텔레오퍼레이션할 수 있도록 지원합니다.
hand tracking을 사용할 때는 absolute variant task(``Isaac-Stack-Cube-Franka-IK-Abs-v0``) 사용을 권장하며,
이 task에는 ``handtracking`` device가 필요합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/environments/teleoperation/teleop_se3_agent.py --task Isaac-Stack-Cube-Franka-IK-Abs-v0 --teleop_device handtracking --device cpu

.. note::

   CloudXR 사용 방법과 Isaac Lab 텔레오퍼레이션 체험 방법은 :ref:`cloudxr-teleoperation`을 참고하십시오.


스크립트는 설정된 teleoperation event를 출력합니다. 키보드의 경우 다음과 같습니다.

.. code:: text

   Keyboard Controller for SE(3): Se3Keyboard
      Reset all commands: R
      Toggle gripper (open/close): K
      Move arm along x-axis: W/S
      Move arm along y-axis: A/D
      Move arm along z-axis: Q/E
      Rotate arm along x-axis: Z/X
      Rotate arm along y-axis: T/G
      Rotate arm along z-axis: C/V

SpaceMouse의 경우 다음과 같습니다.

.. code:: text

   SpaceMouse Controller for SE(3): Se3SpaceMouse
      Reset all commands: Right click
      Toggle gripper (open/close): Click the left button on the SpaceMouse
      Move arm along x/y-axis: Tilt the SpaceMouse
      Move arm along z-axis: Push or pull the SpaceMouse
      Rotate arm: Twist the SpaceMouse

다음 섹션에서는 imitation learning을 위한 데이터 수집에 teleoperation device를 사용하는 방법을 설명합니다.


Isaac Lab Mimic을 이용한 모방 학습
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

teleoperation device를 사용하면 learning from demonstrations(LfD)를 위한 데이터를 수집할 수도 있습니다.
이를 위해 open HDF5 형식으로 데이터를 수집하는 스크립트를 제공합니다.

Demonstration 수집
^^^^^^^^^^^^^^^^^^

``Isaac-Stack-Cube-Franka-IK-Rel-v0`` 환경에서 텔레오퍼레이션으로 demonstration을 수집하려면 다음 명령을 사용합니다.

.. code:: bash

   # step a: create folder for datasets
   mkdir -p datasets
   # step b: collect data with a selected teleoperation device. Replace <teleop_device> with your preferred input device.
   # Available options: spacemouse, keyboard, handtracking
   ./isaaclab.sh -p scripts/tools/record_demos.py --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --device cpu --teleop_device <teleop_device> --dataset_file ./datasets/dataset.hdf5 --num_demos 10
   # step a: replay the collected dataset
   ./isaaclab.sh -p scripts/tools/replay_demos.py --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --device cpu --dataset_file ./datasets/dataset.hdf5


.. note::

   쌓인 cube의 순서는 파란색(아래), 빨간색(중간), 초록색(위)이어야 합니다.

.. tip::

   XR 장치를 사용할 때는 task의 ``Isaac-Stack-Cube-Frank-IK-Abs-v0`` 버전과
   ``--teleop_device handtracking``으로 demonstration을 수집하는 것을 권장합니다.
   이 방식은 손의 absolute position을 사용해 end effector를 제어합니다.

다음 단계가 성공하려면 약 10개의 성공적인 demonstration이 필요합니다.

성공적인 policy training으로 이어지는 demonstration을 수행하기 위한 팁은 다음과 같습니다.

* demonstration을 짧게 유지하십시오. demonstration이 짧을수록 policy가 내려야 할 결정이 줄어들어 학습이 쉬워집니다.
* 직접적인 경로를 선택하십시오. 임의의 축을 따라 움직이지 말고 목표를 향해 직선으로 이동하십시오.
* 멈추지 마십시오. 대신 부드럽고 연속적인 motion을 수행하십시오. policy 입장에서는 왜, 언제 멈춰야 하는지 명확하지 않으므로 연속 motion이 더 배우기 쉽습니다.

demonstration 수행 중 실수했거나 다른 이유로 현재 demonstration을 기록하면 안 되는 경우,
``R`` 키를 눌러 현재 demonstration을 폐기하고 새 시작 위치로 reset하십시오.

.. note::
   IsaacLab의 physics는 ``env.reset`` 사용 시 결정적으로 재현되지 않으므로 replay 중 non-determinism이 관찰될 수 있습니다.

사전 기록된 demonstration
^^^^^^^^^^^^^^^^^^^^^^^^^^

``Isaac-Stack-Cube-Franka-IK-Rel-v0``에 대한 10개의 human demonstration이 들어 있는 사전 기록 ``dataset.hdf5``를 제공합니다.
위치는 다음과 같습니다. `[Franka Dataset] <https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/IsaacLab/Mimic/franka_stack_datasets/dataset.hdf5>`__.
직접 demonstration을 수집하지 않으려면 이 dataset을 다운로드하여 이후 tutorial 단계에서 사용할 수 있습니다.

.. note::
   사전 기록 dataset 사용은 선택 사항입니다.

.. _generating-additional-demonstrations:

Isaac Lab Mimic으로 추가 demonstration 생성
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Isaac Lab Mimic을 사용해 추가 demonstration을 생성할 수 있습니다.

Isaac Lab Mimic은 Isaac Lab의 기능으로, 추가 demonstration을 자동 생성할 수 있게 해 줍니다.
이를 통해 수동 demonstration이 소수만 있어도 policy가 성공적으로 학습할 수 있습니다.

다음 예제에서는 Isaac Lab Mimic을 사용해 추가 demonstration을 생성하는 방법을 보여줍니다.
생성된 demonstration은 state-based policy(``Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0`` 환경 사용)
또는 visuomotor policy(``Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Mimic-v0`` 환경 사용)를 학습하는 데 사용할 수 있습니다.

.. note::
   다음 명령은 사용되는 env 수가 적고 compute-bound가 아니라 I/O-bound이므로 CPU mode로 실행됩니다.

.. important::

   다음 섹션의 모든 명령은 일관된 policy type을 유지해야 합니다.
   예를 들어 state-based policy를 선택했다면 사용하는 모든 명령은 "State-based policy" 탭의 명령이어야 합니다.

기록된 dataset과 함께 Isaac Lab Mimic을 사용하려면 먼저 recording의 subtask에 annotation을 추가합니다.

.. tab-set::
   :sync-group: policy_type

   .. tab-item:: State-based policy
      :sync: state

      .. code:: bash

         ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
         --device cpu --task Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0 --auto \
         --input_file ./datasets/dataset.hdf5 --output_file ./datasets/annotated_dataset.hdf5

   .. tab-item:: Visuomotor policy
      :sync: visuomotor

      .. code:: bash

         ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
         --device cpu --enable_cameras --task Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Mimic-v0 --auto \
         --input_file ./datasets/dataset.hdf5 --output_file ./datasets/annotated_dataset.hdf5


그런 다음 Isaac Lab Mimic을 사용해 추가 demonstration을 일부 생성합니다.

.. tab-set::
   :sync-group: policy_type

   .. tab-item:: State-based policy
      :sync: state

      .. code:: bash

         ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
         --device cpu --num_envs 10 --generation_num_trials 10 \
         --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset_small.hdf5

   .. tab-item:: Visuomotor policy
      :sync: visuomotor

      .. code:: bash

         ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
         --device cpu --enable_cameras --num_envs 10 --generation_num_trials 10 \
         --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset_small.hdf5

.. note::

   ``annotate_demos.py`` 스크립트의 output_file은 ``generate_dataset.py`` 스크립트의 input_file입니다.

생성된 데이터의 출력(``generated_dataset_small.hdf5`` 파일명)을 검사하고 만족스럽다면 전체 dataset을 생성합니다.

.. tab-set::
   :sync-group: policy_type

   .. tab-item:: State-based policy
      :sync: state

      .. code:: bash

         ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
         --device cpu --headless --num_envs 10 --generation_num_trials 1000 \
         --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset.hdf5

   .. tab-item:: Visuomotor policy
      :sync: visuomotor

      .. code:: bash

         ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
         --device cpu --enable_cameras --headless --num_envs 10 --generation_num_trials 1000 \
         --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset.hdf5


demonstration 수는 늘리거나 줄일 수 있으며, 이 task에서는 1000개의 demonstration이 좋은 학습 결과를 제공하는 것으로 나타났습니다.

또한 데이터 생성을 빠르게 하기 위해 ``--num_envs`` parameter의 environment 수를 조정할 수 있습니다.
권장값인 10은 중간 정도 성능의 laptop GPU에서 실행할 수 있습니다.
더 강력한 desktop machine에서는 더 많은 environment 수를 사용하면 이 단계가 크게 빨라집니다.

Robomimic 설정
^^^^^^^^^^^^^^

예제로, policy를 학습하기 위해 `Robomimic <https://robomimic.github.io/>`__ 에 구현된 BC agent를 학습합니다.
다른 framework나 training method도 사용할 수 있습니다.

robomimic framework를 설치하려면 다음 명령을 사용합니다.

.. code:: bash

   # install the dependencies
   sudo apt install cmake build-essential
   # install python module (for robomimic)
   ./isaaclab.sh -i robomimic

Agent 학습
^^^^^^^^^^

Mimic으로 생성한 데이터를 사용해 ``Isaac-Stack-Cube-Franka-IK-Rel-v0``용 state-based BC agent,
또는 ``Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-v0``용 visuomotor BC agent를 학습할 수 있습니다.

.. tab-set::
   :sync-group: policy_type

   .. tab-item:: State-based policy
      :sync: state

      .. code:: bash

         ./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
         --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --algo bc \
         --dataset ./datasets/generated_dataset.hdf5

   .. tab-item:: Visuomotor policy
      :sync: visuomotor

      .. code:: bash

         ./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
         --task Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-v0 --algo bc \
         --dataset ./datasets/generated_dataset.hdf5

.. note::
   기본적으로 학습된 model과 log는 ``IssacLab/logs/robomimic``에 저장됩니다.

결과 시각화
^^^^^^^^^^

.. tip::

   **중요: 여러 checkpoint epoch 테스트**

   policy 성능을 평가할 때 서로 다른 training epoch가 크게 다른 결과를 내는 경우가 흔합니다.
   기대한 성능이 보이지 않는다면 최종 checkpoint만 보지 말고, **항상 다양한 epoch의 policy를 테스트하여**
   가장 성능이 좋은 model을 찾으십시오. model 성능은 학습 과정에서 크게 달라질 수 있으며,
   final epoch가 항상 최적인 것은 아닙니다.

생성된 model로 inference를 수행하여 policy 결과를 시각화할 수 있습니다.

.. tab-set::
   :sync-group: policy_type

   .. tab-item:: State-based policy
      :sync: state

      .. code:: bash

         ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
         --device cpu --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --num_rollouts 50 \
         --checkpoint /PATH/TO/desired_model_checkpoint.pth

   .. tab-item:: Visuomotor policy
      :sync: visuomotor

      .. code:: bash

         ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
         --device cpu --enable_cameras --task Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-v0 --num_rollouts 50 \
         --checkpoint /PATH/TO/desired_model_checkpoint.pth

.. tip::

   **기대한 성능 결과가 보이지 않는 경우:** 최종 checkpoint 하나만 테스트하지 말고 여러 checkpoint epoch의 policy를 테스트하십시오.
   policy 성능은 training epoch에 따라 크게 달라질 수 있으며, 중간 checkpoint가 final model보다 더 나은 경우가 많습니다.

.. note::

   **Franka Cube Stack Task의 예상 성공률과 소요 시간**

   * 데이터 생성 성공률: 약 50%(state + visuomotor 모두)
   * 데이터 생성 시간: state는 약 30분, visuomotor는 약 4시간(사용자가 실행하는 env 수에 따라 달라짐)
   * BC RNN 학습 시간: state는 1000 epoch + 약 30분, visuomotor는 600 epoch + 약 6시간
   * BC RNN policy 성공률: 약 40-60%(state + visuomotor 모두)
   * **권장:** 학습 중 여러 epoch의 checkpoint를 평가하여 가장 성능이 좋은 model을 식별하십시오.


Demo 1: 휴머노이드 로봇을 위한 데이터 생성 및 policy 학습
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/gr-1_steering_wheel_pick_place.gif
   :width: 100%
   :align: center
   :alt: pick and place task를 수행하는 GR-1 휴머노이드 로봇
   :figclass: align-center


Isaac Lab Mimic은 여러 end effector를 가진 로봇의 데이터 생성을 지원합니다.
다음 demonstration에서는 Fourier GR-1 휴머노이드 로봇이 pick and place task를 수행하도록 학습시키기 위한 데이터를 생성하는 방법을 보여줍니다.

선택 사항: demonstration 수집 및 annotation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Human demonstration 수집
""""""""""""""""""""""""
.. note::

   GR-1 휴머노이드 로봇 환경의 데이터 수집에는 Apple Vision Pro headset이 필요합니다.
   Apple Vision Pro를 사용할 수 없다면 이 단계를 건너뛰고 다음 단계인 `Generate the dataset`_ 으로 진행할 수 있습니다.
   사전 기록된 annotated dataset은 다음 단계에서 제공됩니다.

.. tip::
   GR1 scene은 Apple Vision Pro(AVP)의 wrist pose를 differential IK controller(Pink-IK)의 setpoint로 사용합니다.
   differential IK controller가 최적으로 동작하려면 사용자의 wrist pose가 로봇의 초기 pose 또는 현재 pose와 가까워야 합니다.
   사용자의 wrist가 급격히 움직이면 goal state에서 크게 벗어날 수 있으며, 이 경우 IK controller가 최적 solution을 찾지 못할 수 있습니다.
   그 결과 사용자의 wrist와 로봇 wrist 사이에 mismatch가 생길 수 있습니다.
   AVP wrist pose를 더 낮은 latency로 추적하려면 `Pink-IK controller's FrameTasks <https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/pick_place/pickplace_gr1t2_env_cfg.py>`__ 의 gain을 모두 높일 수 있습니다.
   하지만 이렇게 하면 motion이 더 jerky해질 수 있습니다.
   별도로, 로봇의 finger joint는 `dex-retargeting <https://github.com/dexsuite/dex-retargeting>`_ library를 사용해 사용자의 finger joint로 retarget됩니다.

:ref:`cloudxr-teleoperation`의 단계를 따라 CloudXR Runtime과 Apple Vision Pro를 텔레오퍼레이션용으로 설정합니다.
단일 environment 실행 시 XR 성능을 높이기 위해 다음 단계에서는 CPU simulation을 사용합니다.

human demonstration 세트를 수집합니다.
성공 demonstration은 object가 bin에 놓이고 로봇의 오른팔이 시작 위치로 retract되어야 합니다.

Isaac Lab Mimic Env GR-1 휴머노이드 로봇은 왼손에 단일 subtask가 있고, 오른손에는 두 개의 subtask가 있도록 설정되어 있습니다.
첫 번째 subtask에서는 왼손이 object를 집어 오른손이 잡을 위치로 이동하는 동안 오른손이 idle 상태를 유지합니다.
이 설정을 통해 Isaac Lab Mimic은 특히 데이터 생성 중 pose가 randomize될 때 object pose를 사용해 오른손 trajectory를 정확하게 interpolate할 수 있습니다.
따라서 왼손이 object를 집어 안정적인 위치로 가져오는 동안 오른손을 움직이지 마십시오.


.. |good_demo| image:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/gr-1_steering_wheel_pick_place_good_demo.gif
   :width: 49%
   :alt: 좋은 pick and place demonstration을 수행하는 GR-1 휴머노이드 로봇

.. |bad_demo| image:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/gr-1_steering_wheel_pick_place_bad_demo.gif
   :width: 49%
   :alt: 나쁜 pick and place demonstration을 수행하는 GR-1 휴머노이드 로봇

|good_demo| |bad_demo|

.. centered:: 왼쪽: 부드럽고 안정적인 motion의 좋은 human demonstration. 오른쪽: jerky하고 과장된 motion의 나쁜 demonstration.


다음 명령을 실행하여 5개의 demonstration을 수집합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/tools/record_demos.py \
   --device cpu \
   --task Isaac-PickPlace-GR1T2-Abs-v0 \
   --teleop_device handtracking \
   --dataset_file ./datasets/dataset_gr1.hdf5 \
   --num_demos 5 --enable_pinocchio

.. note::
   waist 자유도가 활성화된 GR-1 pick and place task인 ``Isaac-PickPlace-GR1T2-WaistEnabled-Abs-v0``도 제공합니다.
   사용 가능한 environment와 GR1 Waist Enabled variant에 대한 자세한 내용은 :ref:`environments`를 참고하십시오.
   위와 같은 명령을 사용하되 task 이름만 ``Isaac-PickPlace-GR1T2-WaistEnabled-Abs-v0``로 변경하면 됩니다.

.. tip::
   데이터 수집 중 demo가 실패하면 Apple Vision Pro의 XR teleop client에 있는 teleoperation controls panel을 사용하거나
   "reset"이라고 말하는 voice control로 environment를 reset할 수 있습니다. 자세한 내용은 :ref:`teleoperate-apple-vision-pro`를 참고하십시오.

   로봇은 physics 계산을 위해 simulation에 표시되는 상세 visual mesh와 다른 단순화된 collision mesh를 사용합니다.
   이 차이 때문에 physics simulation에서는 적절한 collision handling이 발생하고 있어도,
   로봇 일부가 다른 object나 자기 자신을 관통하는 것처럼 보이는 visual artifact가 가끔 관찰될 수 있습니다.

다음 명령을 실행하여 수집한 demonstration을 replay할 수 있습니다.

.. code:: bash

   ./isaaclab.sh -p scripts/tools/replay_demos.py \
   --device cpu \
   --task Isaac-PickPlace-GR1T2-Abs-v0 \
   --dataset_file ./datasets/dataset_gr1.hdf5 --enable_pinocchio

.. note::
   IsaacLab의 physics는 ``env.reset`` 사용 시 결정적으로 재현되지 않으므로 replay 중 non-determinism이 관찰될 수 있습니다.


Demonstration annotation
""""""""""""""""""""""""

이전 Franka stacking task와 달리 GR-1 pick and place task는 subtask를 정의하기 위해 manual annotation을 사용합니다.

pick and place task에는 왼팔용 subtask 하나(pick)와 오른팔용 subtask 두 개(idle, place)가 있습니다.
annotation은 subtask의 끝을 표시합니다. pick and place task의 경우 왼팔에는 annotation이 없고,
오른팔에는 annotation 하나가 있습니다. 마지막 subtask의 끝은 항상 implicit입니다.

각 demo에는 오른팔의 첫 번째 subtask와 두 번째 subtask 사이에 annotation 하나가 필요합니다.
이 annotation("S" 버튼 누름)은 오른쪽 로봇 팔이 "idle" subtask를 끝내고 target object를 향해 움직이기 시작할 때 수행해야 합니다.
올바른 annotation 예시는 아래와 같습니다.

.. figure:: ../../_static/tasks/manipulation/gr-1_pick_place_annotation.jpg
   :width: 100%
   :align: center

다음 명령을 실행하여 demonstration에 annotation을 추가합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
   --device cpu \
   --task Isaac-PickPlace-GR1T2-Abs-Mimic-v0 \
   --input_file ./datasets/dataset_gr1.hdf5 \
   --output_file ./datasets/dataset_annotated_gr1.hdf5 --enable_pinocchio

.. note::

   스크립트는 manual annotation용 keyboard command와 현재 annotation 중인 subtask를 출력합니다.

   .. code:: text

      Annotating episode #0 (demo_0)
         Playing the episode for subtask annotations for eef "right".
         Subtask signals to annotate:
            - Termination:	['idle_right']

         Press "N" to begin.
         Press "B" to pause.
         Press "S" to annotate subtask signals.
         Press "Q" to skip the episode.

.. tip::

   annotation 중 object가 bin에 놓이지 않으면 "N"을 눌러 episode를 replay하고 다시 annotation할 수 있습니다.
   또는 "Q"를 눌러 해당 episode를 건너뛰고 다음 episode를 annotation할 수 있습니다.

Dataset 생성
^^^^^^^^^^^^

앞의 수집 및 annotation 단계를 건너뛰었다면 사전 기록된 annotated dataset ``dataset_annotated_gr1.hdf5``를
여기에서 다운로드하십시오. `[Annotated GR1 Dataset] <https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/IsaacLab/Mimic/pick_place_datasets/dataset_annotated_gr1.hdf5>`_.
파일을 ``IsaacLab/datasets`` 아래에 두고 다음 명령을 실행하여 1000개의 demonstration이 포함된 새 dataset을 생성합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
   --device cpu --headless --num_envs 20 --generation_num_trials 1000 --enable_pinocchio \
   --input_file ./datasets/dataset_annotated_gr1.hdf5 --output_file ./datasets/generated_dataset_gr1.hdf5

Policy 학습
^^^^^^^^^^^

`Robomimic <https://robomimic.github.io/>`__ 을 사용해 생성된 dataset에 대한 policy를 학습합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
   --task Isaac-PickPlace-GR1T2-Abs-v0 --algo bc \
   --normalize_training_actions \
   --dataset ./datasets/generated_dataset_gr1.hdf5

training script는 dataset의 action을 [-1, 1] 범위로 normalize합니다.
normalization parameter는 model directory의 ``PATH_TO_MODEL_DIRECTORY/logs/normalization_params.txt`` 아래에 저장됩니다.
시각화 단계에서 나중에 사용할 수 있도록 normalization parameter를 기록해 두십시오.

.. note::
   기본적으로 학습된 model과 log는 ``IssacLab/logs/robomimic``에 저장됩니다.

결과 시각화
^^^^^^^^^^

이전 training 단계에서 기록한 normalization parameter를 사용하여 다음 명령으로 학습된 policy 결과를 시각화합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
   --device cpu \
   --enable_pinocchio \
   --task Isaac-PickPlace-GR1T2-Abs-v0 \
   --num_rollouts 50 \
   --horizon 400 \
   --norm_factor_min <NORM_FACTOR_MIN> \
   --norm_factor_max <NORM_FACTOR_MAX> \
   --checkpoint /PATH/TO/desired_model_checkpoint.pth

.. note::
   위 명령의 ``NORM_FACTOR``를 training 단계에서 생성된 값으로 변경하십시오.

.. tip::

   **기대한 성능 결과가 보이지 않는 경우:** 다양한 checkpoint epoch의 policy를 테스트하는 것이 중요합니다.
   성능은 epoch마다 크게 달라질 수 있으며, 가장 성능이 좋은 checkpoint가 final checkpoint가 아닌 경우가 많습니다.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/gr-1_steering_wheel_pick_place_policy.gif
   :width: 100%
   :align: center
   :alt: pick and place task를 수행하는 GR-1 휴머노이드 로봇
   :figclass: align-center

   Isaac Lab에서 pick and place task를 수행하는 학습된 policy.

.. note::

   **Pick and Place GR1T2 Task의 예상 성공률과 소요 시간**

   * 데이터 생성 성공률은 human demonstration 품질(사용자가 얼마나 잘 수행하는지)과 dataset annotation 품질에 따라 달라집니다. 데이터 생성과 downstream policy 성공률 모두 이 요인들에 민감하며 높은 variance를 보일 수 있습니다. dataset 개선 팁은 :ref:`Common Pitfalls when Generating Data <common-pitfalls-generating-data>`를 참고하십시오.
   * 이 task의 데이터 생성 성공률은 보통 1000 demonstration 기준 65-80%이며, GPU hardware와 성공률에 따라 18-40분이 걸립니다(RTX ADA 6000에서 성공률 80%일 때 19분).
   * Behavior Cloning(BC) policy 성공률은 1000개의 생성 demonstration으로 2000 epoch(default) 학습했을 때, demonstration 품질에 따라 보통 75-86%(50 rollout 평가)입니다. RTX ADA 6000에서 학습에는 약 29분이 걸립니다.
   * **권장:** 1000개의 생성 demonstration으로 2000 epoch 학습하고, **1000번째와 2000번째 epoch 사이에 저장된 여러 checkpoint를 평가하여** 가장 성능이 좋은 policy를 선택하십시오. 최적 성능을 찾으려면 다양한 epoch 테스트가 필수입니다.


Demo 2: Unitree G1을 이용한 휴머노이드 로봇 locomanipulation 데이터 생성 및 policy 학습
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

이 demo에서는 단일 휴머노이드 로봇 시스템 안에서 locomotion과 manipulation 기능을 통합하는 방법을 보여줍니다.
이 locomanipulation 환경은 navigation과 object manipulation을 결합한 복잡한 task의 데이터 수집을 가능하게 합니다.
demonstration은 여러 단계로 진행됩니다. 먼저 Demo 1과 유사한 pick and place task를 생성한 다음,
휴머노이드 로봇이 point A에서 point B로 이동해야 하는 scene을 생성하기 위해 특수 script를 사용하는 navigation component를 도입합니다.
로봇은 초기 위치(point A)에서 object를 집어 target destination(point B)에 놓습니다.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/locomanipulation-g-1_steering_wheel_pick_place.gif
   :width: 100%
   :align: center
   :alt: locomanipulation으로 pick and place task를 수행하는 G1 휴머노이드 로봇
   :figclass: align-center

.. note::
   **Locomotion policy 학습**

   이 integration 예제에서 사용된 locomotion policy는 `AGILE <https://github.com/nvidia-isaac/WBC-AGILE>`__ framework를 사용해 학습되었습니다.
   AGILE은 Isaac Lab의 manager based environment를 활용하는 공식 지원 휴머노이드 제어 학습 pipeline입니다.
   또한 Isaac 제품 전반의 다른 evaluation 및 deployment tool과 매끄럽게 통합될 예정입니다.
   이를 통해 팀은 policy training에 필요한 모든 infrastructure와 tooling을 포함하며 real-world deployment로 쉽게 export할 수 있는
   단일 유지보수 stack에 의존할 수 있습니다.
   AGILE repository에는 flexibility를 위해 upper body와 lower body policy가 분리된 업데이트된 pre-trained policy가 포함되어 있습니다.
   이들은 real world에서 검증되었으며 직접 배포할 수 있습니다.
   사용자는 AGILE framework를 사용해 자신만의 locomotion 또는 whole-body control policy도 학습할 수 있습니다.

Manipulation dataset 생성
^^^^^^^^^^^^^^^^^^^^^^^^^

Demo 1.0과 동일한 데이터 생성 및 policy 학습 단계를 locomanipulation 기능이 있는 G1 휴머노이드 로봇에 적용할 수 있습니다.
이 demonstration은 full-body locomotion과 manipulation으로 pick and place task를 수행하도록 G1 로봇을 학습하는 방법을 보여줍니다.

과정은 Demo 1.0과 동일한 workflow를 따르지만 ``Isaac-PickPlace-Locomanipulation-G1-Abs-v0`` task environment를 사용합니다.

Demo 1.0에서 보여준 것과 동일한 데이터 수집, annotation, generation 과정을 따르되 G1 locomanipulation task에 맞게 적용하십시오.

.. hint::

   원한다면 dataset 검증을 위해 이전 예제와 같은 명령으로 데이터 수집과 annotation을 수행할 수 있습니다.

   locomanipulation 기능이 있는 G1 로봇은 full-body locomotion과 manipulation을 결합하여 pick and place task를 수행합니다.

   **다음 명령은 참고 및 dataset 검증 목적일 뿐이며, 이 demo에 필수는 아닙니다.**

   demonstration 수집:

   .. code:: bash

      ./isaaclab.sh -p scripts/tools/record_demos.py \
      --device cpu \
      --task Isaac-PickPlace-Locomanipulation-G1-Abs-v0 \
      --teleop_device handtracking \
      --dataset_file ./datasets/dataset_g1_locomanip.hdf5 \
      --num_demos 5 --enable_pinocchio

   .. note::

      Apple Vision Pro app이 어떻게 초기화되었는지에 따라 operator의 손이 G1 로봇의 손에 비해 매우 높거나 낮을 수 있습니다.
      이 경우 Isaac Lab의 AR tab에서 **Stop AR**을 클릭하고 AR Anchor prim을 움직일 수 있습니다.
      operator의 손을 낮추려면 아래로 조정하고, 높이려면 위로 조정하십시오.
      텔레오퍼레이션 session을 재개하려면 **Start AR**을 클릭하십시오.
      Apple Vision Pro에서 **Play**를 클릭하기 전에 로봇 손과 맞추십시오. 그렇지 않으면 초기에 원치 않는 큰 force가 생성됩니다.

   다음을 실행하여 수집된 demonstration을 replay할 수 있습니다.

   .. code:: bash

      ./isaaclab.sh -p scripts/tools/replay_demos.py \
      --device cpu \
      --task Isaac-PickPlace-Locomanipulation-G1-Abs-v0 \
      --dataset_file ./datasets/dataset_g1_locomanip.hdf5 --enable_pinocchio

   demonstration annotation:

   .. code:: bash

      ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
      --device cpu \
      --task Isaac-Locomanipulation-G1-Abs-Mimic-v0 \
      --input_file ./datasets/dataset_g1_locomanip.hdf5 \
      --output_file ./datasets/dataset_annotated_g1_locomanip.hdf5 --enable_pinocchio


앞의 수집 및 annotation 단계를 건너뛰었다면 사전 기록된 annotated dataset ``dataset_annotated_g1_locomanip.hdf5``를
여기에서 다운로드하십시오. `[Annotated G1 Dataset] <https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/IsaacLab/Mimic/pick_place_datasets/dataset_annotated_g1_locomanip.hdf5>`_.
파일을 ``IsaacLab/datasets`` 아래에 두고 다음 명령을 실행하여 1000개의 demonstration이 포함된 새 dataset을 생성합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
   --device cpu --headless --num_envs 20 --generation_num_trials 1000 --enable_pinocchio \
   --input_file ./datasets/dataset_annotated_g1_locomanip.hdf5 --output_file ./datasets/generated_dataset_g1_locomanip.hdf5


Manipulation-only policy 학습
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

이 시점에서 생성된 dataset을 사용해 manipulation task만 수행하는 policy를 학습할 수 있습니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
   --task Isaac-PickPlace-Locomanipulation-G1-Abs-v0 --algo bc \
   --normalize_training_actions \
   --dataset ./datasets/generated_dataset_g1_locomanip.hdf5

결과 시각화
^^^^^^^^^^

학습된 policy 성능을 시각화합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
   --device cpu \
   --enable_pinocchio \
   --task Isaac-PickPlace-Locomanipulation-G1-Abs-v0 \
   --num_rollouts 50 \
   --horizon 400 \
   --norm_factor_min <NORM_FACTOR_MIN> \
   --norm_factor_max <NORM_FACTOR_MAX> \
   --checkpoint /PATH/TO/desired_model_checkpoint.pth

.. note::
   위 명령의 ``NORM_FACTOR``를 training 단계에서 생성된 값으로 변경하십시오.

.. tip::

   **기대한 성능 결과가 보이지 않는 경우:** 항상 다양한 checkpoint epoch의 policy를 테스트하십시오.
   epoch마다 결과가 크게 달라질 수 있으므로 여러 checkpoint를 평가하여 최적 model을 찾으십시오.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/locomanipulation-g-1_steering_wheel_pick_place.gif
   :width: 100%
   :align: center
   :alt: pick and place task를 수행하는 G1 휴머노이드 로봇
   :figclass: align-center

   Isaac Lab에서 pick and place task를 수행하는 학습된 policy.

.. note::

   **Locomanipulation Pick and Place Task의 예상 성공률과 소요 시간**

   * 데이터 생성 성공률은 human demonstration 품질(사용자가 얼마나 잘 수행하는지)과 dataset annotation 품질에 따라 달라집니다. 데이터 생성과 downstream policy 성공률 모두 이 요인들에 민감하며 높은 variance를 보일 수 있습니다. dataset 개선 팁은 :ref:`Common Pitfalls when Generating Data <common-pitfalls-generating-data>`를 참고하십시오.
   * 이 task의 데이터 생성 성공률은 보통 1000 demonstration 기준 65-82%이며, GPU hardware와 성공률에 따라 18-40분이 걸립니다(RTX ADA 6000에서 성공률 82%일 때 18분).
   * Behavior Cloning(BC) policy 성공률은 1000개의 생성 demonstration으로 2000 epoch(default) 학습했을 때, demonstration 품질에 따라 보통 75-85%(50 rollout 평가)입니다. RTX ADA 6000에서 학습에는 약 40분이 걸립니다.
   * **권장:** 1000개의 생성 demonstration으로 2000 epoch 학습하고, **1000번째와 2000번째 epoch 사이에 저장된 여러 checkpoint를 평가하여** 가장 성능이 좋은 policy를 선택하십시오. 최적 성능을 찾으려면 다양한 epoch 테스트가 필수입니다.

Manipulation과 point-to-point navigation을 포함한 dataset 생성
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

manipulation과 navigation 기능을 모두 결합한 포괄적인 locomanipulation dataset을 만들려면,
이전 단계의 manipulation dataset을 input으로 사용해 navigation dataset을 생성할 수 있습니다.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/disjoint_navigation.gif
   :width: 100%
   :align: center
   :alt: navigation과 locomanipulation을 결합하는 G1 휴머노이드 로봇
   :figclass: align-center

   navigation 기능과 함께 locomanipulation을 수행하는 G1 휴머노이드 로봇.

locomanipulation dataset 생성 과정은 이전에 생성한 manipulation dataset을 가져와,
로봇이 manipulation task를 수행하면서 한 위치에서 다른 위치로 navigate해야 하는 scenario를 만듭니다.
이렇게 하면 locomotion과 manipulation behavior를 모두 포함하는 더 복잡한 dataset이 생성됩니다.

locomanipulation dataset을 생성하려면 다음 명령을 사용합니다.

.. code:: bash

   ./isaaclab.sh -p \
       scripts/imitation_learning/locomanipulation_sdg/generate_data.py \
       --device cpu \
       --kit_args="--enable isaacsim.replicator.mobility_gen" \
       --task="Isaac-G1-SteeringWheel-Locomanipulation" \
       --dataset ./datasets/generated_dataset_g1_locomanip.hdf5 \
       --num_runs 1 \
       --lift_step 60 \
       --navigate_step 130 \
       --enable_pinocchio \
       --output_file ./datasets/generated_dataset_g1_locomanipulation_sdg.hdf5 \
       --enable_cameras

.. note::

   input dataset(``--dataset``)은 이전 단계에서 생성한 manipulation dataset이어야 합니다.
   ``--output_file_name`` parameter를 사용해 원하는 output filename을 지정할 수 있습니다.

locomanipulation dataset 생성의 핵심 parameter는 다음과 같습니다.

* ``--lift_step 70``: manipulation task의 lifting phase step 수입니다. 로봇이 object를 grasp한 직후의 지점을 표시해야 합니다.
* ``--navigate_step 120``: 위치 사이 navigation phase의 step 수입니다. 로봇이 object를 들어 올리고 걸을 준비가 된 지점을 표시해야 합니다.
* ``--output_file``: output dataset file 이름입니다.

이 과정은 로봇이 서로 다른 위치에서 manipulation task를 수행하고,
학습된 manipulation behavior를 유지하면서 지점 사이를 navigate해야 하는 dataset을 만듭니다.
결과 dataset은 locomotion과 manipulation 기능을 모두 결합하는 policy를 학습하는 데 사용할 수 있습니다.

.. note::

   다음 script command로 로봇 trajectory 결과를 시각화할 수 있습니다.

   .. code:: bash

      ./isaaclab.sh -p scripts/imitation_learning/locomanipulation_sdg/plot_navigation_trajectory.py --input_file datasets/generated_dataset_g1_locomanipulation_sdg.hdf5 --output_dir /PATH/TO/DESIRED_OUTPUT_DIR

이 locomanipulation pipeline에서 생성된 데이터는 GR00T N1.5를 사용한 imitation learning policy fine-tuning에도 사용할 수 있습니다.
이를 위해 생성된 dataset을 GR00T N1.5가 기대하는 LeRobot format으로 변환한 다음,
GR00T N1.5 repository에서 제공하는 fine-tuning script를 실행할 수 있습니다.
closed-loop policy rollout 예시는 아래 video에 나와 있습니다.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/locomanipulation_sdg_disjoint_nav_groot_policy_4x.gif
   :width: 100%
   :align: center
   :alt: locomanipulation을 위해 fine-tune된 GR00T N1.5 policy의 simulation rollout
   :figclass: align-center

   locomanipulation을 위해 fine-tune된 GR00T N1.5 policy의 simulation rollout.

위에 표시된 policy는 camera image, hand pose, hand joint position, object pose, base goal pose를 input으로 사용합니다.
model output은 다음 여러 timestep에 대한 target base velocity, hand pose, hand joint position입니다.


Demo 3: 휴머노이드 로봇을 위한 Visuomotor Policy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/gr-1_nut_pouring_policy.gif
   :width: 100%
   :align: center
   :alt: pouring task를 수행하는 GR-1 휴머노이드 로봇
   :figclass: align-center

Dataset 다운로드
^^^^^^^^^^^^^^^^

사전 생성된 dataset을 `여기 <https://download.isaacsim.omniverse.nvidia.com/isaaclab/dataset/generated_dataset_gr1_nut_pouring.hdf5>`__ 에서 다운로드하고
``IsaacLab/datasets/generated_dataset_gr1_nut_pouring.hdf5`` 아래에 두십시오.
(**참고: dataset 크기는 약 12GB입니다**). 이 dataset에는 ``Isaac-NutPour-GR1T2-Pink-IK-Abs-Mimic-v0`` task를 위해
Isaac Lab Mimic으로 생성된, 휴머노이드 로봇이 pouring/placing task를 수행하는 1000개의 demonstration이 포함되어 있습니다.

.. hint::

   원한다면 이전 예제와 같은 명령으로 데이터 수집, annotation, generation을 수행할 수 있습니다.

   로봇은 먼저 빨간 beaker를 집어 내용물을 노란 bowl에 붓습니다.
   그런 다음 빨간 beaker를 파란 bin에 떨어뜨립니다. 마지막으로 노란 bowl을 흰색 scale 위에 놓습니다.
   task의 시각적 demonstration은 아래 :ref:`visualize-results-demo-2` 섹션의 video를 참고하십시오.

   **이 task의 success criteria는 빨간 beaker가 파란 bin에 놓이고, 초록 nut가 노란 bowl 안에 있으며,
   노란 bowl이 흰색 scale 위에 놓이는 것을 요구합니다.**

   .. attention::
      **다음 명령은 참고용일 뿐이며 이 demo에 필수는 아닙니다.**

   demonstration 수집:

   .. code:: bash

      ./isaaclab.sh -p scripts/tools/record_demos.py \
      --device cpu \
      --task Isaac-NutPour-GR1T2-Pink-IK-Abs-v0 \
      --teleop_device handtracking \
      --dataset_file ./datasets/dataset_gr1_nut_pouring.hdf5 \
      --num_demos 5 --enable_pinocchio

   이것은 visuomotor environment이므로 annotation과 data generation command에 ``--enable_cameras`` flag를 추가해야 합니다.

   demonstration annotation:

   .. code:: bash

      ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
      --device cpu \
      --enable_cameras \
      --rendering_mode balanced \
      --task Isaac-NutPour-GR1T2-Pink-IK-Abs-Mimic-v0 \
      --input_file ./datasets/dataset_gr1_nut_pouring.hdf5 \
      --output_file ./datasets/dataset_annotated_gr1_nut_pouring.hdf5 --enable_pinocchio

   .. warning::
      이 task에는 right eef annotation이 여러 개 있습니다. 같은 eef에 대한 subtask annotation은 같은 action index를 가질 수 없습니다.
      right eef subtask를 서로 다른 action index로 annotation해야 합니다.


   dataset 생성:

   .. code:: bash

      ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
      --device cpu \
      --headless \
      --enable_pinocchio \
      --enable_cameras \
      --rendering_mode balanced \
      --task Isaac-NutPour-GR1T2-Pink-IK-Abs-Mimic-v0 \
      --generation_num_trials 1000 \
      --num_envs 5 \
      --input_file ./datasets/dataset_annotated_gr1_nut_pouring.hdf5 \
      --output_file ./datasets/generated_dataset_gr1_nut_pouring.hdf5


Policy 학습
^^^^^^^^^^^

`Robomimic <https://robomimic.github.io/>`__ 을 사용해 이 task의 visuomotor BC agent를 학습합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
   --task Isaac-NutPour-GR1T2-Pink-IK-Abs-v0 --algo bc \
   --normalize_training_actions \
   --dataset ./datasets/generated_dataset_gr1_nut_pouring.hdf5

training script는 dataset의 action을 [-1, 1] 범위로 normalize합니다.
normalization parameter는 model directory의 ``PATH_TO_MODEL_DIRECTORY/logs/normalization_params.txt`` 아래에 저장됩니다.
시각화 단계에서 나중에 사용할 수 있도록 normalization parameter를 기록해 두십시오.

.. note::
   기본적으로 학습된 model과 log는 ``IsaacLab/logs/robomimic``에 저장됩니다.

또한 `GR00T <https://github.com/NVIDIA/Isaac-GR00T>`__ foundation model을 post-train하여 이 task용 Vision-Language-Action policy를 배포할 수 있습니다.

자세한 내용은 `IsaacLabEvalTasks <https://github.com/isaac-sim/IsaacLabEvalTasks/>`__ repository를 참고하십시오.

.. _visualize-results-demo-2:

결과 시각화
^^^^^^^^^^

이전 training 단계에서 기록한 normalization parameter를 사용하여 다음 명령으로 학습된 policy 결과를 시각화합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
   --device cpu \
   --enable_pinocchio \
   --enable_cameras \
   --rendering_mode balanced \
   --task Isaac-NutPour-GR1T2-Pink-IK-Abs-v0 \
   --num_rollouts 50 \
   --horizon 350 \
   --norm_factor_min <NORM_FACTOR_MIN> \
   --norm_factor_max <NORM_FACTOR_MAX> \
   --checkpoint /PATH/TO/desired_model_checkpoint.pth

.. note::
   위 명령의 ``NORM_FACTOR``를 training 단계에서 생성된 값으로 변경하십시오.

.. tip::

   **기대한 성능 결과가 보이지 않는 경우:** 최종 checkpoint 하나만 보지 말고 다양한 checkpoint epoch의 policy를 테스트하십시오.
   policy 성능은 학습 과정에서 크게 달라질 수 있으며, 중간 checkpoint가 더 나은 결과를 내는 경우가 많습니다.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/gr-1_nut_pouring_policy.gif
   :width: 100%
   :align: center
   :alt: pouring task를 수행하는 GR-1 휴머노이드 로봇
   :figclass: align-center

   Isaac Lab에서 pouring task를 수행하는 학습된 visuomotor policy.

.. note::

   **Visuomotor Nut Pour GR1T2 Task의 예상 성공률과 소요 시간**

   * 데이터 생성 성공률은 human demonstration 품질(사용자가 얼마나 잘 수행하는지)과 dataset annotation 품질에 따라 달라집니다. 데이터 생성과 downstream policy 성공률 모두 이 요인들에 민감하며 높은 variance를 보일 수 있습니다. dataset 개선 팁은 :ref:`Common Pitfalls when Generating Data <common-pitfalls-generating-data>`를 참고하십시오.
   * 1000 demonstration 데이터 생성은 RTX ADA 6000에서 약 10시간이 걸립니다.
   * Behavior Cloning(BC) policy 성공률은 1000개의 생성 demonstration으로 600 epoch(default) 학습했을 때 보통 50-60%(50 rollout 평가)입니다. RTX ADA 6000에서 학습에는 약 15시간이 걸립니다.
   * **권장:** 1000개의 생성 demonstration으로 600 epoch 학습하고, **300번째와 600번째 epoch 사이에 저장된 여러 checkpoint를 평가하여** 가장 성능이 좋은 policy를 선택하십시오. 최적 성능을 얻으려면 다양한 epoch 테스트가 중요합니다.

.. _common-pitfalls-generating-data:

데이터 생성 시 흔한 문제
~~~~~~~~~~~~~~~~~~~~~~~~

**Demonstration이 너무 김:**

* time horizon이 길수록 policy가 학습하기 어렵습니다.
* 첫 object 가까이에서 시작하고 motion을 최소화하십시오.

**Demonstration이 부드럽지 않음:**

* 불규칙한 motion은 policy가 해석하기 어렵습니다.
* 더 좋은 teleop device는 더 좋은 데이터를 만듭니다. 예를 들어 SpaceMouse가 Keyboard보다 좋습니다.

**Demonstration 중 pause:**

* pause는 학습하기 어렵습니다.
* 사람의 motion을 부드럽고 유동적으로 유지하십시오.

**Subtask 수가 과도함:**

* 주어진 task를 완료하는 데 필요한 정의된 subtask 수를 최소화하십시오.
* subtask가 적을수록 trajectory stitching이 줄어 데이터 생성 성공률이 높아집니다.

**Action noise 부족:**

* action noise는 policy를 더 robust하게 만듭니다.

**Recording이 너무 타이트하게 잘림:**

* success term이 trigger되는 frame에서 recording이 멈추면 replay 중 다시 trigger되지 않을 수 있습니다.
* recording 끝에 약간의 buffer를 허용하십시오.

**Non-deterministic replay:**

* IsaacLab의 physics는 ``env.reset`` 사용 시 결정적으로 재현되지 않으므로 replay에서 demonstration이 실패할 수 있습니다.
* 필요한 것보다 더 많은 human demo를 수집하고, annotation 중 성공하는 것을 사용하십시오.
* Isaac Lab Mimic이 생성한 HDF5 file의 모든 데이터는 성공 demo를 나타내며 training에 사용할 수 있습니다. replay 시 non-determinism 때문에 실패하더라도 그렇습니다.


자신만의 Isaac Lab Mimic 호환 Environment 만들기
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

동작 방식
^^^^^^^^^

Isaac Lab Mimic은 input demonstration을 subtask로 나누는 방식으로 동작합니다.
subtask는 모든 demonstration에 공통적으로 나타나는 사용자가 정의한 demonstration segment입니다.
subtask 예시는 "object grasp", "end effector를 사전 정의된 위치로 이동", "object release" 등이 있습니다.
대부분의 subtask는 로봇이 상호작용하는 어떤 object를 기준으로 정의된다는 점에 유의하십시오.

subtask를 정의한 다음 각 input demonstration에 대해 annotation해야 합니다.
annotation은 위 예제처럼 subtask detection heuristic을 정의하여 algorithmically 수행할 수도 있고, 수동으로 수행할 수도 있습니다.

subtask가 정의되고 annotation되면 Isaac Lab Mimic은 몇 가지 helper method를 사용해 subtask segment를 변환하고,
현재 새 task에 맞도록 이들을 이어 붙여 새 demonstration을 생성합니다.

이렇게 생성된 각 candidate demonstration에 대해 Isaac Lab Mimic은 boolean success criteria를 사용해 demonstration이 task 수행에 성공했는지 판단하고,
성공했다면 output dataset에 추가합니다. candidate demonstration의 성공률은 task 난이도와 로봇 자체의 복잡도에 따라
단순한 경우 70%까지 높을 수 있고, <1%까지 낮을 수 있습니다.

Configuration과 subtask 정의
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Isaac Lab Mimic의 기타 configuration setting과 함께 subtask는 기존 environment config를 추가 Mimic required parameter로 확장하여 만든
Mimic compatible environment configuration class 안에서 정의됩니다.

모든 Mimic required config parameter는 :class:`~isaaclab.envs.MimicEnvCfg` class에 지정되어 있습니다.

:class:`~isaaclab_mimic.envs.FrankaCubeStackIKRelMimicEnvCfg` config class는 위 예제에서 사용한 Franka stacking task에 대해
Mimic compatible environment config class를 만드는 예시입니다.

``DataGenConfig`` member에는 데이터 생성 방식에 영향을 주는 다양한 parameter가 들어 있습니다.
처음에는 ``name`` parameter만 설정하고 나머지는 나중에 수정해도 충분합니다.

subtask는 :class:`~isaaclab.envs.SubTaskConfig` object의 list이며, 가장 중요한 member는 다음과 같습니다.

* ``object_ref`` 는 상호작용 중인 object입니다. 데이터 생성 중 이 object를 기준으로 motion을 조정하는 데 사용됩니다. 현재 subtask가 어떤 object도 포함하지 않는 경우 ``None``일 수 있습니다.
* ``subtask_term_signal`` 은 subtask가 active인지 아닌지를 나타내는 signal의 ID입니다.

multi end-effector environment에서는 subtask constraint를 지정하여 end-effector 간 subtask ordering을 강제할 수 있습니다.
이 constraint는 :class:`~isaaclab.envs.SubTaskConstraintConfig` class에 정의되어 있습니다.

Subtask annotation
^^^^^^^^^^^^^^^^^^

subtask가 정의되면 source data에 annotation해야 합니다.
source demonstration의 subtask boundary를 annotation하는 방법은 manual annotation과 heuristic 사용 두 가지입니다.

input demonstration 수가 보통 매우 적기 때문에 manual annotation을 수행하는 것이 가장 쉬운 경우가 많습니다.
manual annotation을 수행하려면 ``--auto`` flag 없이 ``annotate_demos.py`` script를 사용합니다.
그런 다음 ``B``를 눌러 pause, ``N``을 눌러 continue, ``S``를 눌러 subtask boundary를 annotation합니다.

더 정확한 boundary가 필요하거나 실험을 위해 특정 task의 반복 처리를 빠르게 하려면 같은 작업을 수행하는 heuristic을 구현할 수 있습니다.
heuristic은 environment의 observation입니다.
subtask term을 추가하는 예시는 ``source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/stack/stack_env_cfg.py``에서 확인할 수 있으며,
여기서는 ``SubtaskCfg``라는 observation group으로 추가됩니다.
이 예제는 prebuilt heuristic을 사용하지만 custom heuristic도 쉽게 구현할 수 있습니다.


Demonstration 생성을 위한 helper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Isaac Lab Mimic에 필요한 helper는 environment에 정의됩니다.
Isaac Lab Mimic과 함께 사용할 모든 task는 :class:`~isaaclab.envs.ManagerBasedRLMimicEnv` base class에서 파생되며,
다음 function을 구현해야 합니다.

* ``get_robot_eef_pose``: 로봇 end effector controller가 사용하는 frame과 같은 frame에서 현재 robot end effector pose를 반환합니다.

* ``target_eef_pose_to_action``: target pose와 end effector controller용 gripper action을 받아 target pose를 달성하는 action을 반환합니다.

* ``action_to_target_eef_pose``: action을 받아 end effector controller용 target pose를 반환합니다.

* ``actions_to_gripper_actions``: action sequence를 받아 action 중 gripper actuation 부분을 반환합니다.

* ``get_object_poses``: 데이터 생성에 사용되는 scene 내 각 object의 pose를 반환합니다.

* ``get_subtask_term_signals``: task 내 각 subtask에 대한 binary flag dictionary를 반환합니다. subtask가 완료되면 flag가 true로 설정되고 그렇지 않으면 false입니다.

:class:`~isaaclab_mimic.envs.FrankaCubeStackIKRelMimicEnv` class는 기존 Isaac Lab environment에서 Mimic compatible environment를 만드는 예시를 보여줍니다.

Environment 등록
^^^^^^^^^^^^^^^^

Mimic compatible environment와 environment config class가 모두 만들어지면 ``gym.register``를 사용해 새 Mimic compatible environment를 등록할 수 있습니다.
위 예제의 Franka stacking task에서는 Mimic environment가 ``Isaac-Stack-Cube-Franka-IK-Rel-Mimic-v0``로 등록됩니다.

등록된 environment는 이제 Isaac Lab Mimic과 함께 사용할 준비가 되었습니다.


Isaac Lab Mimic으로 성공적인 데이터 생성을 하기 위한 팁
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Subtask 나누기
^^^^^^^^^^^^^^

일반적인 원칙은 task를 완료할 수 있는 범위에서 subtask 수를 최대한 적게 나누는 것입니다.
Isaac Lab Mimic 데이터 생성은 linear interpolation을 사용해 subtask segment를 연결하고 stitching합니다.
subtask가 많을수록 trajectory stitching이 많아져 motion이 덜 부드러워지고 실패 demonstration이 늘 수 있습니다.
이 때문에 로봇 motion이 다른 object와 충돌할 가능성이 낮은 지점에 subtask boundary를 annotation하는 것이 가장 좋은 경우가 많습니다.

예를 들어 아래 scenario에서는 로봇의 왼팔이 object를 grasp한 뒤 subtask partition이 있습니다.
왼쪽에서는 grasp 직후 subtask annotation이 표시되어 있고, 오른쪽에서는 로봇이 object를 grasp하고 들어 올린 뒤 annotation이 표시되어 있습니다.
왼쪽의 경우 interpolation 때문에 로봇 왼팔이 table과 충돌하고 motion이 지연됩니다.
반면 오른쪽에서는 motion이 연속적이고 부드럽습니다.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/lagging_subtask.gif
   :width: 99%
   :align: center
   :alt: Subtask splitting example
   :figclass: align-center

.. centered:: 잘못된 subtask splitting으로 인한 motion lag/collision(왼쪽)


Interpolation step 수 선택
^^^^^^^^^^^^^^^^^^^^^^^^^^

subtask segment 사이의 interpolation step 수는 :class:`~isaaclab.envs.SubTaskConfig` class에서 지정할 수 있습니다.
변환 후 subtask segment는 같은 위치에서 시작/종료하지 않으므로, 연속적인 motion을 만들기 위해 Isaac Lab Mimic은
이전 subtask의 마지막 지점과 다음 subtask의 첫 지점 사이에 linear interpolation을 적용합니다.

interpolation step 수는 이 stitching 과정에서 생성되는 demonstration의 부드러움을 제어하도록 조정할 수 있습니다.
적절한 interpolation step 수는 로봇 속도와 task 복잡도에 따라 달라집니다.
object reset distribution이 큰 복잡한 task는 subtask segment 사이의 gap이 더 커지고, 부드러운 motion을 만들기 위해 더 많은 interpolation step이 필요합니다.
반대로 subtask segment 사이 gap이 작은 task는 너무 많은 step으로 인해 불필요한 motion lag가 생기지 않도록 적은 interpolation step을 사용해야 합니다.

interpolation step 수가 생성 demonstration에 어떤 영향을 줄 수 있는지에 대한 예시는 아래와 같습니다.
이 예시에서는 왼팔 grasp와 오른팔 placement 사이의 gap을 연결하기 위해 로봇 오른팔에 interpolation을 적용합니다.
0 step에서는 오른팔 motion이 jerky하게 jump하고, 20 step에서는 motion이 laggy합니다. 5 step에서는 motion이 부드럽고 자연스럽습니다.

.. |0_interp_steps| image:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/0_interpolation_steps.gif
   :width: 32%
   :alt: interpolation step 0인 GR-1 로봇

.. |5_interp_steps| image:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/5_interpolation_steps.gif
   :width: 32%
   :alt: interpolation step 5인 GR-1 로봇

.. |20_interp_steps| image:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/20_interpolation_steps.gif
   :width: 32%
   :alt: interpolation step 20인 GR-1 로봇

|0_interp_steps| |5_interp_steps| |20_interp_steps|

.. centered:: 왼쪽: 0 step. 가운데: 5 step. 오른쪽: 20 step.
