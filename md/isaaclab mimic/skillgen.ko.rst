.. _skillgen:

자동 Demonstration 생성을 위한 SkillGen
=======================================

SkillGen은 motion planning을 통합하여 Isaac Lab Mimic을 강화하는 고급 demonstration 생성 시스템입니다.
사람이 제공한 subtask segment와 자동 motion planning을 결합하여 고품질, adaptive, collision-free 로봇 demonstration을 생성합니다.

SkillGen이란?
~~~~~~~~~~~~~

SkillGen은 기존 demonstration 생성 방식의 주요 한계를 해결합니다.

* **Motion Quality**: cuRobo의 GPU 가속 motion planner를 사용해 부드럽고 collision-free한 trajectory를 생성합니다.
* **Validity**: skill segment 사이에 kinematically feasible한 plan을 생성합니다.
* **Diversity**: configurable sampling 및 planning parameter를 통해 다양한 demonstration을 생성합니다.
* **Adaptability**: 데이터 생성 중 새로운 object placement와 scene configuration에 적응할 수 있는 demonstration을 생성합니다.

이 시스템은 수동 annotation된 human demonstration을 입력으로 받아 localized subtask skill(`Subtasks in SkillGen`_ 참고)을 추출하고,
로봇 kinematics와 collision constraint를 만족하면서 이러한 skill segment 사이의 feasible motion을 cuRobo로 plan하는 방식으로 동작합니다.

사전 요구 사항
~~~~~~~~~~~~~~

SkillGen을 사용하기 전에 다음을 이해해야 합니다.

1. **Teleoperation**: keyboard, SpaceMouse, hand tracking을 사용해 로봇을 제어하고 demonstration을 기록하는 방법
2. **Isaac Lab Mimic**: 데이터 수집, annotation, generation, policy training을 포함한 전체 workflow

.. important::

   SkillGen을 진행하기 전에 :ref:`teleoperation-imitation-learning` 문서를 충분히 검토하십시오.

.. _skillgen-installation:

설치
~~~~

SkillGen에는 Isaac Lab, Isaac Sim, cuRobo가 필요합니다. Isaac Lab conda environment에서 다음 단계를 따르십시오.

Step 1: Isaac Sim과 Isaac Lab 설치 및 확인
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

공식 Isaac Sim 및 Isaac Lab 설치 가이드는 `여기 <https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html#installing-isaac-lab>`__ 를 따르십시오.

Step 2: cuRobo 설치
^^^^^^^^^^^^^^^^^^^

cuRobo는 SkillGen의 motion planning 기능을 제공합니다. 아래 설치 방법은 Isaac Lab의 PyTorch 및 CUDA 요구사항과 함께 동작하는 것으로 테스트되었습니다.

.. code:: bash

   # One line installation of cuRobo (formatted for readability)
   conda install -c nvidia cuda-toolkit=12.8 -y && \
   export CUDA_HOME="$CONDA_PREFIX" && \
   export PATH="$CUDA_HOME/bin:$PATH" && \
   export LD_LIBRARY_PATH="$CUDA_HOME/lib:$LD_LIBRARY_PATH" && \
   export TORCH_CUDA_ARCH_LIST="8.0+PTX" && \
   pip install -e "git+https://github.com/NVlabs/curobo.git@ebb71702f3f70e767f40fd8e050674af0288abe8#egg=nvidia-curobo" --no-build-isolation

.. note::
   * commit hash ``ebb71702f3f70e767f40fd8e050674af0288abe8``는 Isaac Lab과 함께 테스트되었습니다. 다른 버전을 사용하면 compatibility issue가 생길 수 있습니다. 이 commit에는 cuRobo가 USD를 collision object로 parse하는 데 필요한 quad face mesh triangulation 지원이 포함되어 있습니다.

   * cuRobo는 source에서 설치되며 editable install됩니다. 즉 cuRobo source code가 현재 directory의 ``src/nvidia-curobo`` 아래에 clone됩니다. 사용자는 cuRobo를 설치할 working directory를 선택할 수 있습니다.

   * 위 명령의 ``TORCH_CUDA_ARCH_LIST``는 GPU의 CUDA compute capability와 맞아야 합니다. 예를 들어 A100은 ``8.0``, 많은 RTX 30-series는 ``8.6``, RTX 4090은 ``8.9``입니다. ``+PTX`` suffix는 forward compatibility를 위해 PTX를 포함하므로 native SASS가 포함되지 않은 newer GPU도 JIT-compile할 수 있습니다.

.. warning::

   **Isaac Sim environment script가 source된 상태에서는 cuRobo 설치가 실패할 수 있습니다**

   Omniverse Kit/Isaac Sim environment script(예: ``setup_conda_env.sh``)를 source하면 ``PYTHONHOME``과 ``PYTHONPATH``가 Kit runtime 및 bundled Python package로 export됩니다. cuRobo 설치 중 이로 인해 ``conda``가 initialization 전에 Omniverse bundled library(예: ``requests``/``urllib3``)를 import할 수 있고, crash가 발생할 수 있습니다. 흔히 ``omni.kit.pip_archive``를 참조하는 ``TypeError``로 나타납니다.

   다음 중 하나를 수행하십시오.

   - Omniverse/Isaac Sim script를 source하지 않은 clean shell에서 cuRobo를 설치합니다.
   - Conda를 호출하기 전에 상속된 Python environment variable, 특히 ``PYTHONPATH``와 ``PYTHONHOME``을 임시로 reset하거나 무시하여 Kit Python이 Conda environment를 shadow하지 않게 합니다.
   - shell activation에 의존하지 않고 현재 shell의 Python variable을 상속하지 않는 Conda mechanism을 사용합니다.

   설치가 완료된 후에는 정상 사용을 위해 Isaac Lab/Isaac Sim script를 다시 source해도 됩니다.



Step 3: Rerun 설치
^^^^^^^^^^^^^^^^^^

개발 중 trajectory visualization을 위해 다음을 설치합니다.

.. code:: bash

   pip install rerun-sdk==0.23

.. note::

   **Rerun Visualization Setup:**

   * Rerun은 선택 사항이지만 개발 중 planned trajectory를 debug하고 validate하는 데 매우 권장됩니다.
   * cuRobo planner configuration에서 ``visualize_plan = True``로 설정하면 trajectory visualization을 활성화할 수 있습니다.
   * 활성화되면 cuRobo planner interface가 planned end-effector trajectory, waypoint, collision data를 Rerun으로 stream하여 interactive inspection을 가능하게 합니다.
   * Visualization은 전체 dataset generation 전에 planning issue, collision problem, trajectory smoothness를 식별하는 데 도움이 됩니다.
   * ``--headless``와 함께 실행하여 isaacsim visualization은 비활성화하면서 end effector trajectory는 계속 visualize/debug할 수도 있습니다.

Step 4: 설치 확인
^^^^^^^^^^^^^^^^^

cuRobo가 Isaac Lab과 함께 동작하는지 테스트합니다.

.. code:: bash

   # This should run without import errors
   python -c "import curobo; print('cuRobo installed successfully')"

.. tip::

   ``libstdc++.so.6: version 'GLIBCXX_3.4.30' not found`` 오류가 발생하면 다음 명령으로 해결을 시도할 수 있습니다.

   .. code:: bash

      conda config --env --set channel_priority strict
      conda config --env --add channels conda-forge
      conda install -y -c conda-forge "libstdcxx-ng>=12" "libgcc-ng>=12"

SkillGen Dataset 다운로드
~~~~~~~~~~~~~~~~~~~~~~~~~

SkillGen을 빠르게 시작할 수 있도록 사전 annotation된 dataset을 제공합니다.

Dataset 내용
^^^^^^^^^^^^

dataset에는 다음이 포함됩니다.

* Franka arm cube stacking의 human demonstration
* 각 demonstration에 대해 수동 annotation된 subtask boundary
* basic cube stacking과 adaptive bin cube stacking task 모두와 호환

다운로드 및 설정
^^^^^^^^^^^^^^^^

1. `여기 <https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/IsaacLab/Mimic/franka_stack_datasets/annotated_dataset_skillgen.hdf5>`__ 를 클릭하여 사전 annotation된 dataset을 다운로드합니다.

2. datasets directory를 준비하고 다운로드한 file을 이동합니다.

.. code:: bash

   # Make sure you are in the root directory of your Isaac Lab workspace
   cd /path/to/your/IsaacLab

   # Create the datasets directory if it does not exist
   mkdir -p datasets

   # Move the downloaded dataset into the datasets directory
   mv /path/to/annotated_dataset_skillgen.hdf5 datasets/annotated_dataset_skillgen.hdf5

.. tip::

   SkillGen의 큰 장점은 같은 annotated dataset을 여러 관련 task(예: basic stacking과 adaptive bin stacking)에 재사용할 수 있다는 점입니다. variant마다 새 데이터를 수집하고 annotation하는 일을 피할 수 있습니다.

.. admonition:: {이 tutorial의 task에서는 선택 사항} 새 dataset 수집(source + annotated)

      새 source dataset을 수집한 다음 SkillGen용 annotated dataset을 만들고 싶다면 다음 명령을 따르십시오. 사용자는 Isaac Lab Mimic workflow에 대한 지식이 있다고 가정합니다.

   **시작 전 중요한 포인터**

   * 제공된 annotated dataset을 사용하는 것이 이 tutorial의 SkillGen task를 가장 빠르게 시작하는 경로입니다.
   * 자체 dataset을 만드는 경우 SkillGen은 subtask start와 termination boundary 모두의 manual annotation을 요구합니다(auto-annotation 없음).
   * start boundary signal은 SkillGen에 필수입니다. annotation 중 ``--annotate_subtask_start_signals``를 사용하지 않으면 data generation이 실패합니다.
   * subtask 정의(``object_ref``, ``subtask_term_signal``)를 SkillGen environment config와 일관되게 유지하십시오.

   **Demonstration 기록** (모든 teleop device가 지원됩니다. 필요하면 ``spacemouse``를 교체하십시오):

   .. code:: bash

      ./isaaclab.sh -p scripts/tools/record_demos.py \
      --task Isaac-Stack-Cube-Franka-IK-Rel-Skillgen-v0 \
      --teleop_device spacemouse \
      --dataset_file ./datasets/dataset_skillgen.hdf5 \
      --num_demos 10

   **SkillGen용 demonstration annotation** (term boundary와 start boundary를 모두 기록):

   .. code:: bash

      ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
      --device cpu \
      --task Isaac-Stack-Cube-Franka-IK-Rel-Skillgen-v0 \
      --input_file ./datasets/dataset_skillgen.hdf5 \
      --output_file ./datasets/annotated_dataset_skillgen.hdf5 \
      --annotate_subtask_start_signals

Dataset Annotation 이해
~~~~~~~~~~~~~~~~~~~~~~~

SkillGen은 subtask start와 termination boundary가 annotation된 dataset을 요구합니다. Auto-annotation은 지원되지 않습니다.

SkillGen의 Subtask
^^^^^^^^^^^^^^^^^^

**기술적 정의:** subtask는 manipulation objective를 달성하는 연속 demo segment이며, ``SubTaskConfig``를 통해 정의됩니다.

* ``object_ref``: 이 subtask의 spatial reference로 사용되는 object 또는 ``None``
* ``subtask_term_signal``: binary termination signal 이름. subtask가 완료될 때 0에서 1로 transition합니다.
* ``subtask_start_signal``: binary start signal 이름. subtask가 시작될 때 0에서 1로 transition합니다. SkillGen에는 필수입니다.

subtask localization process는 다음을 수행합니다.

* signal transition point(0에서 1)를 감지하여 subtask boundary ``[t_start, t_end]``를 식별합니다.
* boundary 사이의 subtask segment를 추출합니다.
* ``object_ref``가 제공된 경우 이를 사용하여 object-relative 또는 task-relative frame에서 end-effector trajectory와 key pose를 계산합니다.

이를 통해 absolute하고 scene-specific한 motion을 object-relative skill segment로 변환하며, data generation 중 새로운 object placement와 scene configuration에 적응할 수 있게 됩니다.

Manual Annotation Workflow
^^^^^^^^^^^^^^^^^^^^^^^^^^
Isaac Lab Mimic workflow와 달리 SkillGen은 subtask start와 termination boundary의 manual annotation을 요구합니다.
예를 들어 cube grasping의 경우 start signal은 gripper가 닫히기 직전이고 termination signal은 object가 grasp된 직후입니다.
subtask 정의에 맞게 start와 termination signal을 조정할 수 있습니다.

.. tip::

   **Manual Annotation Controls:**

   * ``N``을 눌러 playback을 시작/계속합니다.
   * ``B``를 눌러 pause합니다.
   * ``S``를 눌러 subtask boundary를 표시합니다.
   * ``Q``를 눌러 현재 demonstration을 건너뜁니다.

   skill segment(예: grasp, stack 등)의 start와 end signal을 annotation할 때는 skill 몇 step 전에 ``B``로 playback을 pause하고,
   ``S``로 start signal을 annotation한 다음 ``N``으로 playback을 재개합니다.
   skill이 완료된 후 몇 step 뒤에 다시 pause하여 ``S``로 end signal을 annotation합니다.

SkillGen을 이용한 Data Generation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SkillGen은 motion planning을 사용해 annotation된 demonstration을 다양하고 고품질인 dataset으로 변환합니다.

SkillGen 동작 방식
^^^^^^^^^^^^^^^^^^

SkillGen pipeline은 annotated dataset과 environment의 Mimic API를 사용해 새 demonstration을 합성합니다.

1. **Subtask boundary 사용**: annotated dataset에서 subtask별 start와 termination index를 읽습니다.
2. **Goal sampling**: task constraint와 datagen config에 따라 subtask별 target pose를 sample합니다.
3. **Trajectory planning**: ``--use_skillgen`` 사용 시 cuRobo를 사용해 subtask segment 사이의 collision-free motion을 plan합니다.
4. **Trajectory stitching**: skill segment와 planned trajectory를 이어 붙여 complete demonstration을 만듭니다.
5. **Success evaluation**: task success term을 validate하며, 성공한 trial만 output dataset에 기록합니다.

Usage Parameter
^^^^^^^^^^^^^^^

SkillGen data generation의 주요 parameter는 다음과 같습니다.

* ``--use_skillgen``: SkillGen planner를 활성화합니다. 필수입니다.
* ``--generation_num_trials``: 생성할 demonstration 수입니다.
* ``--num_envs``: parallel environment 수입니다. GPU memory에 맞게 조정하십시오.
* ``--device``: computation device(cpu/cuda)입니다. stable physics에는 cpu를 사용하십시오.
* ``--headless``: 더 빠른 generation을 위해 visualization을 비활성화합니다.

.. _task-basic-cube-stacking:

Task 1: Basic Cube Stacking
~~~~~~~~~~~~~~~~~~~~~~~~~~~

표준 Isaac Lab Mimic cube stacking task용 demonstration을 생성합니다. 이 task에서 Franka robot은 다음을 수행해야 합니다.

1. 빨간 cube를 집어 파란 cube 위에 놓습니다.
2. 초록 cube를 집어 빨간 cube 위에 놓습니다.
3. 최종 stack 순서: 파란색(아래), 빨간색(중간), 초록색(위).

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/cube_stack_data_gen_skillgen.gif
   :width: 75%
   :align: center
   :alt: SkillGen으로 생성된 cube stacking task
   :figclass: align-center

   Cube stacking dataset example.

Small-Scale Generation
^^^^^^^^^^^^^^^^^^^^^^

모든 것이 동작하는지 확인하기 위해 작은 dataset으로 시작합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
   --device cpu \
   --num_envs 1 \
   --generation_num_trials 10 \
   --input_file ./datasets/annotated_dataset_skillgen.hdf5 \
   --output_file ./datasets/generated_dataset_small_skillgen_cube_stack.hdf5 \
   --task Isaac-Stack-Cube-Franka-IK-Rel-Skillgen-v0 \
   --use_skillgen

Full-Scale Generation
^^^^^^^^^^^^^^^^^^^^^

small-scale 결과가 만족스럽다면 전체 training dataset을 생성합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
   --device cpu \
   --headless \
   --num_envs 1 \
   --generation_num_trials 1000 \
   --input_file ./datasets/annotated_dataset_skillgen.hdf5 \
   --output_file ./datasets/generated_dataset_skillgen_cube_stack.hdf5 \
   --task Isaac-Stack-Cube-Franka-IK-Rel-Skillgen-v0 \
   --use_skillgen

.. note::

   * 더 빠른 generation을 위해 ``--headless``를 사용해 visualization을 비활성화합니다. debugging을 위해서는 cuRobo planner configuration에서 ``visualize_plan = True``로 설정하면 ``--headless``가 활성화되어 있어도 Rerun visualization을 사용할 수 있습니다.
   * GPU memory에 맞게 ``--num_envs``를 조정하십시오. 1에서 시작해 점진적으로 늘리는 것이 좋습니다. num_envs가 1보다 클 때 성능 향상은 매우 크지 않습니다. 대부분의 GPU에서는 5 정도가 cuRobo instance와 simulation environment 사이의 performance와 memory usage 균형에 적절한 sweet spot으로 보입니다.
   * Generation time: RTX 6000 Ada GPU에서 ``--headless``를 활성화하고 environment 1개로 1000 demonstration을 생성할 때 약 90-120분입니다. 시간은 GPU, environment 수, demonstration 성공률(annotation dataset 품질에 따라 달라짐)에 따라 달라집니다.
   * cuRobo planner interface와 configuration은 :ref:`cuRobo-interface-features`에 설명되어 있습니다.

.. _task-bin-cube-stacking:

Task 2: Bin 안에서 Adaptive Cube Stacking
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
SkillGen은 adaptive task용 dataset 생성에도 사용할 수 있습니다. 이 예제에서는 좁은 bin 안에서 adaptive cube stacking dataset을 생성합니다.
bin은 workspace의 고정 위치와 orientation에 놓이고, 파란 cube는 bin 중앙에 배치됩니다.
로봇은 bin과 충돌하지 않고 빨간 cube와 초록 cube를 파란 cube 위에 쌓는 성공 demonstration을 생성해야 합니다.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/bin_cube_stack_data_gen_skillgen.gif
   :width: 75%
   :align: center
   :alt: SkillGen으로 생성된 adaptive bin cube stacking task
   :figclass: align-center

   Adaptive bin stacking data generation example.

Small-Scale Generation
^^^^^^^^^^^^^^^^^^^^^^

adaptive stacking setup을 테스트합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
   --device cpu \
   --num_envs 1 \
   --generation_num_trials 10 \
   --input_file ./datasets/annotated_dataset_skillgen.hdf5 \
   --output_file ./datasets/generated_dataset_small_skillgen_bin_cube_stack.hdf5 \
   --task Isaac-Stack-Cube-Bin-Franka-IK-Rel-Mimic-v0 \
   --use_skillgen

Full-Scale Generation
^^^^^^^^^^^^^^^^^^^^^

complete adaptive stacking dataset을 생성합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
   --device cpu \
   --headless \
   --num_envs 1 \
   --generation_num_trials 1000 \
   --input_file ./datasets/annotated_dataset_skillgen.hdf5 \
   --output_file ./datasets/generated_dataset_skillgen_bin_cube_stack.hdf5 \
   --task Isaac-Stack-Cube-Bin-Franka-IK-Rel-Mimic-v0 \
   --use_skillgen

.. warning::

   Adaptive task는 복잡도가 증가하므로 일반적으로 success rate가 낮고 data generation time이 더 깁니다. dataset 생성 시간도 vanilla cube stacking보다 낮은 success rate와 어려운 planning problem 때문에 더 길어집니다.

.. note::

   pre-annotated dataset을 사용하고 data generation command를 ``--headless``로 실행하는 경우, RTX 6000 Ada GPU에서 single environment로 1000 demonstration을 생성하는 데 보통 약 220분이 걸립니다.

.. note::

   **VRAM 사용량과 GPU 권장 사항**

   아래 수치는 RTX 6000 Ada에서 10개의 생성 demonstration을 기준으로 측정했습니다.
    * Vanilla Cube Stacking: 1 env는 steady 상태에서 약 9.3-9.6 GB, 5 envs는 약 21.8-22.2 GB(초기화 중 잠시 더 높을 수 있음).
    * Adaptive Bin Cube Stacking: 1 env는 steady 상태에서 약 9.3-9.6 GB, 5 envs는 약 22.0-22.3 GB(초기화 중 잠시 더 높을 수 있음).
    * 최소 권장 GPU: ``--num_envs`` 1-2에는 VRAM 24 GB 이상, ``--num_envs`` 최대 약 5에는 VRAM 48 GB 이상.
    * VRAM을 줄이려면 ``--headless``를 선호하고 ``--num_envs``를 적당히 유지하십시오. 수치는 scene asset과 demonstration 수에 따라 달라질 수 있습니다.

SkillGen Data로 Policy 학습
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Isaac Lab Mimic workflow와 유사하게, 생성된 SkillGen dataset과 Robomimic을 사용해 imitation learning policy를 학습할 수 있습니다.

Basic Cube Stacking Policy
^^^^^^^^^^^^^^^^^^^^^^^^^^

basic cube stacking task용 state-based policy를 학습합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
   --task Isaac-Stack-Cube-Franka-IK-Rel-Skillgen-v0 \
   --algo bc \
   --dataset ./datasets/generated_dataset_skillgen_cube_stack.hdf5

Adaptive Bin Cube Stacking Policy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

더 복잡한 adaptive bin stacking용 policy를 학습합니다.

.. code:: bash

   ./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
   --task Isaac-Stack-Cube-Bin-Franka-IK-Rel-Mimic-v0 \
   --algo bc \
   --dataset ./datasets/generated_dataset_skillgen_bin_cube_stack.hdf5

.. note::

   training script는 model checkpoint를 model directory의 ``IssacLab/logs/robomimic`` 아래에 저장합니다.

학습된 Policy 평가
^^^^^^^^^^^^^^^^^^

학습된 policy를 테스트합니다.

.. code:: bash

   # Basic cube stacking evaluation
   ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
   --device cpu \
   --task Isaac-Stack-Cube-Franka-IK-Rel-Skillgen-v0 \
   --num_rollouts 50 \
   --checkpoint /path/to/model_checkpoint.pth

.. code:: bash

   # Adaptive bin cube stacking evaluation
   ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
   --device cpu \
   --task Isaac-Stack-Cube-Bin-Franka-IK-Rel-Mimic-v0 \
   --num_rollouts 50 \
   --checkpoint /path/to/model_checkpoint.pth

.. note::

   **Cube Stacking 및 Bin Cube Stacking Task의 예상 성공률과 권장 사항**

   * SkillGen data generation과 downstream policy success는 task와 dataset annotation 품질에 민감하며 높은 variance를 보일 수 있습니다.
   * cube stacking과 bin cube stacking의 경우, 지침에 따라 dataset이 적절히 annotation되면 data generation success는 보통 40-70%입니다.
   * 1000개의 생성 demonstration으로 2000 epoch(default) 학습한 Behavior Cloning(BC) policy success는 데이터 품질에 따라 보통 40-85%입니다.
   * 1000개의 demonstration으로 2000 epoch policy를 학습하는 데 RTX 6000 Ada GPU에서 약 30-35분이 걸립니다. training time은 demonstration 수와 epoch 수에 따라 증가합니다.
   * dataset generation time은 :ref:`task-basic-cube-stacking` 및 :ref:`task-bin-cube-stacking`을 참고하십시오.
   * 권장: 기본값인 2000 epoch로 약 1000개의 생성 demonstration을 사용해 학습하고, 1000번째 epoch 이후 저장된 여러 checkpoint를 평가해 가장 성능이 좋은 policy를 선택하십시오.

.. _cuRobo-interface-features:

cuRobo Interface 기능
~~~~~~~~~~~~~~~~~~~~

이 섹션은 cuRobo planner interface와 기능을 요약합니다. SkillGen pipeline은 cuRobo planner를 사용해 subtask segment 사이의 collision-free motion을 생성합니다.
하지만 사용자는 cuRobo를 자신의 task를 위한 standalone motion planner로 사용할 수도 있습니다.
또한 base motion planner를 subclass하고 같은 API를 구현하여 자신만의 motion planner를 구현할 수도 있습니다.

Base Motion Planner (확장 가능)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* 위치: ``isaaclab_mimic/motion_planners/base_motion_planner.py``
* 목적: SkillGen에서 사용하는 모든 motion planner를 위한 uniform interface
* 확장성: subclassing하고 같은 API를 구현하여 새 planner를 추가할 수 있습니다. SkillGen은 code change 없이 API를 사용합니다.

cuRobo Planner (GPU, collision-aware)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* 위치: ``isaaclab_mimic/motion_planners/curobo``
* Multi-phase planning:

  * subtask별 Retreat -> Contact -> Approach phase
  * contact phase에서 configurable collision filtering
  * SkillGen에서는 retreat와 approach phase가 collision-free입니다. transit phase는 collision-check됩니다.

* World synchronization:

  * 각 trial마다 Isaac Lab scene에서 robot state, attached object, collision sphere를 update합니다.
  * grasp/place 중 object의 dynamic attach/detach

* Collision representation:

  * phase별 enable/filter가 있는 contact-aware sphere set

* Outputs:

  * stitching용 time-parameterized, collision-checked trajectory

* Tests:

  * ``source/isaaclab_mimic/test/test_curobo_planner_cube_stack.py``
  * ``source/isaaclab_mimic/test/test_curobo_planner_franka.py``
  * ``source/isaaclab_mimic/test/test_generate_dataset_skillgen.py``

.. list-table::
   :widths: 50 50
   :header-rows: 0

   * - .. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/cube_stack_end_to_end_curobo.gif
         :height: 260px
         :align: center
         :alt: Franka Panda robot을 사용한 cube stack의 cuRobo planner test

         Cube stack planner test.
     - .. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/obstacle_avoidance_curobo.gif
         :height: 260px
         :align: center
         :alt: Franka Panda robot을 사용한 obstacle avoidance의 cuRobo planner test

         Franka planner test.

이 테스트들은 cuRobo를 standalone motion planner로 사용하는 방법에 대한 reference로도 활용할 수 있습니다.

.. note::

   자세한 cuRobo config 생성과 parameter는 ``isaaclab_mimic/motion_planners/curobo/curobo_planner_config.py`` file을 참고하십시오.

Generation Pipeline 통합
^^^^^^^^^^^^^^^^^^^^^^^^

``generate_dataset.py``에서 ``--use_skillgen``이 활성화되면 다음 pipeline이 실행됩니다.

1. **Subtask boundary randomize**: task-configured offset range를 사용해 각 subtask의 demo별 start와 termination index를 randomize합니다.

2. **Subtask별 trajectory build**:
   각 end-effector와 subtask에 대해:

   - source demonstration segment를 선택합니다(strategy-driven, coordination/sequential constraint를 준수).
   - segment를 현재 scene으로 transform합니다(object-relative 또는 coordination delta, optional first-pose interpolation).
   - transformed segment를 waypoint trajectory로 wrap합니다.

3. **Subtask 사이 transition**:
   - cuRobo로 subtask의 첫 waypoint까지 collision-aware transition을 plan합니다(world sync, optional attach/detach). planned waypoint를 실행한 다음 subtask trajectory를 재개합니다.

4. **Constraint와 함께 실행**:
   - subtask constraint(sequential, synchronous step이 있는 coordination)를 enforce하면서 end-effector 전반에 대해 waypoint를 step-by-step으로 실행합니다. 활성화된 경우 planner visualization을 update할 수 있습니다.

5. **Record and export**:
   - state/observation/action을 accumulate하고 episode success flag를 설정한 뒤 episode를 export합니다. outer pipeline은 success를 filter/consume합니다.

Visualization and Debugging
^^^^^^^^^^^^^^^^^^^^^^^^^^^

사용자는 Rerun 기반 plan visualizer를 사용해 planned trajectory를 시각화하고 collision을 debug할 수 있습니다.
cuRobo planner configuration에서 ``visualize_plan = True``로 설정하면 활성화됩니다.
planned trajectory를 시각화하려면 rerun이 설치되어 있어야 합니다.
설치 지침은 :ref:`skillgen-installation`의 Step 3을 참고하십시오.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/rerun_cube_stack.gif
   :width: 80%
   :align: center
   :alt: planned trajectory와 collision의 Rerun visualization
   :figclass: align-center

   Rerun integration: collision sphere가 포함된 planned trajectory.

.. note::

   cuRobo 사용 license는 ``docs/licenses/dependencies/cuRobo-license.txt``에서 확인하십시오.
