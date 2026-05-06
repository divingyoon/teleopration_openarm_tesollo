.. _augmented-imitation-learning:

Augmented Imitation Learning
============================

이 섹션은 Isaac Lab의 imitation learning 기능과 `Cosmos <https://www.nvidia.com/en-us/ai/cosmos/>`_ model의 visual augmentation 기능을 함께 사용하여,
visual variation에 robust한 visuomotor policy를 학습하기 위한 demonstration을 대규모로 생성하는 방법을 설명합니다.

Demonstration 생성
~~~~~~~~~~~~~~~~~~

소수의 annotation된 demonstration으로부터 추가 demonstration을 자동 생성할 수 있는 Isaac Lab Mimic 기능을 사용합니다.

.. note::
    이 섹션은 이미 수집된 demonstration의 annotated dataset이 있다고 가정합니다.
    없다면 :ref:`teleoperation-imitation-learning`의 지침을 따라 직접 demonstration을 수집하고 annotation할 수 있습니다.

다음 예제에서는 Isaac Lab Mimic을 사용해 추가 demonstration을 생성하는 방법을 보여줍니다.
이 demonstration은 visuomotor policy를 직접 학습하는 데 사용할 수도 있고,
Cosmos를 사용해 visual variation으로 augment할 수도 있습니다.
사용 환경은 ``Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Cosmos-Mimic-v0``입니다.

.. note::
    ``Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Cosmos-Mimic-v0`` 환경은 표준 visuomotor environment(``Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Mimic-v0``)와 유사하지만,
    생성 dataset에 segmentation mask, depth map, normal map이 추가됩니다.
    이러한 추가 modality는 Cosmos를 이용한 visual augmentation에서 최상의 결과를 얻기 위해 필요합니다.

.. code:: bash

    ./isaaclab.sh -p scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
    --device cpu --enable_cameras --headless --num_envs 10 --generation_num_trials 1000 \
    --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/mimic_dataset_1k.hdf5 \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Cosmos-Mimic-v0 \
    --rendering_mode performance

demonstration 수는 늘리거나 줄일 수 있으며, 이 task에서는 1000개의 demonstration이 좋은 학습 결과를 제공하는 것으로 나타났습니다.

또한 데이터 생성을 빠르게 하기 위해 ``--num_envs`` parameter의 environment 수를 조정할 수 있습니다.
권장값인 10은 중간 정도 성능의 laptop CPU에서 실행할 수 있습니다.
더 강력한 desktop machine에서는 더 많은 environment 수를 사용하면 이 단계가 크게 빨라집니다.

Cosmos Augmentation
~~~~~~~~~~~~~~~~~~~

HDF5에서 MP4로 변환
^^^^^^^^^^^^^^^^^^^

``hdf5_to_mp4.py`` script는 HDF5 demonstration file에 저장된 camera frame을 MP4 video로 변환합니다.
RGB, segmentation, depth, normal map을 포함한 여러 camera modality를 지원합니다.
Cosmos를 이용한 visual augmentation은 HDF5 data가 아니라 video file에서만 동작하므로 이 변환이 필요합니다.

.. rubric:: Required Arguments

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``--input_file``
      - input HDF5 file 경로입니다.
    * - ``--output_dir``
      - output MP4 file을 저장할 directory입니다.

.. rubric:: Optional Arguments

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``--input_keys``
      - HDF5 file에서 처리할 input key list입니다. 기본값: ["table_cam", "wrist_cam", "table_cam_segmentation", "table_cam_normals", "table_cam_shaded_segmentation", "table_cam_depth"]
    * - ``--video_height``
      - output video height(pixel)입니다. 기본값: 704
    * - ``--video_width``
      - output video width(pixel)입니다. 기본값: 1280
    * - ``--framerate``
      - output video의 frames per second입니다. 기본값: 30

.. note::
    default input key는 ``Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Cosmos-Mimic-v0`` 환경에서 따르는 naming convention에 맞춰 모든 camera modality를 포함합니다.
    추가 modality인 "table_cam_shaded_segmentation"도 포함되어 있는데, 이는 HDF5 data file에서 simulation이 생성한 modality의 일부는 아닙니다.
    대신 이 script가 segmentation과 normal map의 조합을 사용해 자동 생성하며, Cosmos augmentation을 더 잘 제어하기 위한 pseudo-textured segmentation video를 얻기 위해 사용됩니다.

.. note::
    Cosmos augmentation에서 최상의 결과를 얻으려면 위에 제시한 output video height, width, framerate의 default value 사용을 권장합니다.

cube stacking task의 사용 예시는 다음과 같습니다.

.. code:: bash

    python scripts/tools/hdf5_to_mp4.py \
    --input_file datasets/mimic_dataset_1k.hdf5 \
    --output_dir datasets/mimic_dataset_1k_mp4

.. _running-cosmos:

Visual Augmentation을 위한 Cosmos 실행
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

demonstration을 MP4 format으로 변환한 후, `Cosmos`_ model을 사용해 video를 visual augmentation할 수 있습니다.
augmentation process의 자세한 내용은 Cosmos 문서를 따르십시오.
visual augmentation은 task 관련 핵심 feature를 보존하면서 lighting, texture, background 및 기타 visual element를 변경할 수 있습니다.

아래와 같이 이전 단계의 RGB, depth, shaded segmentation video를 Cosmos model의 input으로 사용합니다.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/cosmos_inputs.gif
   :width: 100%
   :align: center
   :alt: RGB, depth and segmentation control inputs to Cosmos

아래에는 `Cosmos Transfer1 <https://github.com/nvidia-cosmos/cosmos-transfer1/tree/e4055e39ee9c53165e85275bdab84ed20909714a>`_ 의 augmentation output 예시를 제공합니다.

.. figure:: https://download.isaacsim.omniverse.nvidia.com/isaaclab/images/cosmos_output.gif
   :width: 100%
   :align: center
   :alt: Cosmos Transfer1 augmentation output

visual augmentation에는 `Cosmos Transfer1 <https://github.com/nvidia-cosmos/cosmos-transfer1/tree/e4055e39ee9c53165e85275bdab84ed20909714a>`_ model 사용을 권장합니다.
넓은 범위의 visual variation을 가진 매우 다양한 dataset을 생성하는 데 가장 좋은 결과를 보였기 때문입니다.
Transfer1을 이 use case에 사용하는 방법은 `installation instructions <https://github.com/nvidia-cosmos/cosmos-transfer1/blob/e4055e39ee9c53165e85275bdab84ed20909714a/INSTALL.md#environment-setup>`_,
`checkpoint download instructions <https://github.com/nvidia-cosmos/cosmos-transfer1/blob/e4055e39ee9c53165e85275bdab84ed20909714a/examples/inference_cosmos_transfer1_7b.md#download-checkpoints>`_,
그리고 `this example <https://github.com/nvidia-cosmos/cosmos-transfer1/blob/e4055e39ee9c53165e85275bdab84ed20909714a/examples/inference_cosmos_transfer1_7b.md#example-2-multimodal-control>`_ 을 참고하십시오.
이 task에서 Transfer1 model과 함께 다음 설정을 사용하는 것을 추가로 권장합니다.

.. note::
    이 workflow는 Cosmos Transfer1 repository의 commit ``e4055e39ee9c53165e85275bdab84ed20909714a``로 테스트되었으며, 이 버전 사용을 권장합니다.
    Cosmos Transfer1 repository를 clone한 후 ``git checkout e4055e39ee9c53165e85275bdab84ed20909714a``를 실행해 이 특정 commit으로 checkout하십시오.

.. rubric:: Hyperparameters

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``negative_prompt``
      - "The video captures a game playing, with bad crappy graphics and cartoonish frames. It represents a recording of old outdated games. The images are very pixelated and of poor CG quality. There are many subtitles in the footage. Overall, the video is unrealistic and appears cg. Plane background."
    * - ``sigma_max``
      - 50
    * - ``control_weight``
      - "0.3,0.3,0.6,0.7"
    * - ``hint_key``
      - "blur,canny,depth,segmentation"

좋은 augmentation을 얻기 위한 또 다른 중요한 요소는 Cosmos generation을 제어하는 prompt set입니다.
``cosmos_prompt_gen.py`` script를 제공하며, augmentation process의 다양한 측면을 처리하는 신중히 선택된 template set에서 prompt를 구성합니다.

.. rubric:: Required Arguments

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``--templates_path``
      - prompt template이 들어 있는 file 경로입니다.

.. rubric:: Optional Arguments

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``--num_prompts``
      - 생성할 prompt 수입니다. 기본값: 1
    * - ``--output_path``
      - 생성된 prompt를 기록할 output file 경로입니다. 기본값: prompts.txt

.. code:: bash

    python scripts/tools/cosmos/cosmos_prompt_gen.py \
    --templates_path scripts/tools/cosmos/transfer1_templates.json \
    --num_prompts 10 --output_path prompts.txt

자체 prompt를 만들고 싶다면 다음 guideline을 참고하는 것을 권장합니다.

1. prompt를 가능한 한 자세하게 유지하십시오. generation이 각 visible object/region of interest를 어떻게 처리해야 하는지에 대한 지시를 포함하는 것이 가장 좋습니다. 예를 들어 제공된 prompt는 table, lighting, background, robot arm, cube, general setting에 대한 명시적인 detail을 포함합니다.

2. augmentation instruction은 가능한 한 realistic하고 coherent하게 유지하십시오. prompt가 비현실적이거나 비관습적일수록 model은 input control video의 핵심 feature를 유지하는 데 더 나쁜 성능을 보입니다.

3. 각 aspect의 augmentation instruction을 서로 sync되게 유지하십시오. 즉 모든 object/region of interest에 대한 augmentation이 서로 coherent하고 conventional해야 합니다. 예를 들어 "The table is of old dark wood with faded polish and food stains and the background consists of a suburban home" 같은 prompt가 "The table is of old dark wood with faded polish and food stains and the background consists of a spaceship hurtling through space" 같은 prompt보다 좋습니다.

4. input control video에서 유지되어야 하거나 변경하지 않아야 하는 key aspect에 대한 detail을 포함하는 것이 매우 중요합니다. 우리의 prompt에서는 cube color가 변경되지 않아야 하며 bottom cube는 blue, middle은 red, top은 green이어야 한다고 매우 명확히 언급합니다. 변경하지 않아야 할 것뿐 아니라 그 aspect가 현재 어떤 형태인지에 대한 detail도 함께 제공합니다.

이 use case에서 Cosmos Transfer1 model을 사용하는 예시 command는 다음과 같습니다.

.. code:: bash

    export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:=0}"
    export CHECKPOINT_DIR="${CHECKPOINT_DIR:=./checkpoints}"
    export NUM_GPU="${NUM_GPU:=1}"
    PYTHONPATH=$(pwd) torchrun --nproc_per_node=$NUM_GPU --nnodes=1 --node_rank=0 cosmos_transfer1/diffusion/inference/transfer.py \
        --checkpoint_dir $CHECKPOINT_DIR \
        --video_save_folder outputs/cosmos_dataset_1k_mp4 \
        --controlnet_specs ./controlnet_specs/demo_0.json \
        --offload_text_encoder_model \
        --offload_guardrail_models \
        --num_gpus $NUM_GPU

위 command와 함께 사용할 예시 ``./controlnet_specs/demo_0.json`` json file은 다음과 같습니다.

.. code:: json

    {
        "prompt": "A robotic arm is picking up and stacking cubes inside a foggy industrial scrapyard at dawn, surrounded by piles of old robotic parts and twisted metal. The background includes large magnetic cranes, rusted conveyor belts, and flickering yellow floodlights struggling to penetrate the fog. The robot arm is bright teal with a glossy surface and silver stripes on the outer edges; the joints rotate smoothly and the pistons reflect a pale cyan hue. The robot arm is mounted on a table that is light oak wood with a natural grain pattern and a glossy varnish that reflects overhead lights softly; small burn marks dot one corner. The arm is connected to the base mounted on the table. The bottom cube is deep blue, the second cube is bright red, and the top cube is vivid green, maintaining their correct order after stacking. Sunlight pouring in from a large, open window bathes the table and robotic arm in a warm golden light. The shadows are soft, and the scene feels natural and inviting with a slight contrast between light and shadow.",
        "negative_prompt": "The video captures a game playing, with bad crappy graphics and cartoonish frames. It represents a recording of old outdated games. The images are very pixelated and of poor CG quality. There are many subtitles in the footage. Overall, the video is unrealistic and appears cg. Plane background.",
        "input_video_path" : "mimic_dataset_1k_mp4/demo_0_table_cam.mp4",
        "sigma_max": 50,
        "vis": {
            "input_control": "mimic_dataset_1k_mp4/demo_0_table_cam.mp4",
            "control_weight": 0.3
        },
        "edge": {
            "control_weight": 0.3
        },
        "depth": {
            "input_control": "mimic_dataset_1k_mp4/demo_0_table_cam_depth.mp4",
            "control_weight": 0.6
        },
        "seg": {
            "input_control": "mimic_dataset_1k_mp4/demo_0_table_cam_shaded_segmentation.mp4",
            "control_weight": 0.7
        }
    }

MP4에서 HDF5로 변환
^^^^^^^^^^^^^^^^^^^

``mp4_to_hdf5.py`` script는 visually augmented MP4 video를 training용 HDF5 format으로 다시 변환합니다.
이 단계는 augmented visual data가 Isaac Lab에서 visuomotor policy를 학습하기 위한 올바른 format이 되도록 하고,
video를 원본 dataset의 해당 demonstration data와 pair로 맞추기 때문에 중요합니다.

.. rubric:: Required Arguments

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``--input_file``
      - 원본 demonstration이 들어 있는 input HDF5 file 경로입니다.
    * - ``--videos_dir``
      - visually augmented MP4 video가 들어 있는 directory입니다.
    * - ``--output_file``
      - augmented video가 포함된 새 HDF5 file을 저장할 경로입니다.

.. note::
    input HDF5 file은 robot state와 action 같은 non-visual data를 보존하면서 visual data를 augmented version으로 교체하는 데 사용됩니다.

.. important::
    visually augmented MP4 file은 ``demo_{demo_id}_*.mp4`` naming convention을 따라야 합니다.

    - ``demo_id``는 원본 MP4 file의 demonstration ID와 일치해야 합니다.

    - ``*``는 이 지점부터 file name을 사용자 선호대로 지정할 수 있음을 의미합니다.

    이 naming convention은 script가 augmented video를 해당 demonstration과 올바르게 pair로 맞추기 위해 필요합니다.

cube stacking task의 사용 예시는 다음과 같습니다.

.. code:: bash

    python scripts/tools/mp4_to_hdf5.py \
    --input_file datasets/mimic_dataset_1k.hdf5 \
    --videos_dir datasets/cosmos_dataset_1k_mp4 \
    --output_file datasets/cosmos_dataset_1k.hdf5

사전 생성 Dataset
^^^^^^^^^^^^^^^^^

cube stacking task를 위한 visually augmented demonstration이 들어 있는 HDF5 format의 사전 생성 dataset을 제공합니다.
Cosmos를 local에서 실행해 직접 augmented data를 생성하고 싶지 않은 경우 이 dataset을 사용할 수 있습니다.
dataset은 `Hugging Face <https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Augmented>`_ 에서 사용할 수 있으며,
visuomotor policy 학습에 사용할 수 있는 original demonstration과 augmented demonstration을 별도 dataset file로 모두 포함합니다.

Dataset 병합
^^^^^^^^^^^^

``merge_hdf5_datasets.py`` script는 여러 HDF5 dataset을 하나의 file로 결합합니다.
original demonstration과 augmented demonstration을 결합하여 더 크고 다양한 training dataset을 만들고 싶을 때 유용합니다.

.. rubric:: Required Arguments

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``--input_files``
      - merge할 HDF5 file 경로 list입니다.

.. rubric:: Optional Arguments

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``--output_file``
      - merged output file path입니다. 기본값: merged_dataset.hdf5

.. tip::
    dataset 병합은 training 중 model이 original visual condition과 augmented visual condition을 모두 접하게 하여 policy robustness를 높이는 데 도움이 됩니다.

cube stacking task의 사용 예시는 다음과 같습니다.

.. code:: bash

    python scripts/tools/merge_hdf5_datasets.py \
    --input_files datasets/mimic_dataset_1k.hdf5 datasets/cosmos_dataset_1k.hdf5 \
    --output_file datasets/mimic_cosmos_dataset.hdf5

Model Training and Evaluation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Robomimic Setup
^^^^^^^^^^^^^^^

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

생성된 데이터를 사용해 ``Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Cosmos-v0``용 visuomotor BC agent를 학습할 수 있습니다.

.. code:: bash

    ./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Cosmos-v0 --algo bc \
    --dataset ./datasets/mimic_cosmos_dataset.hdf5 \
    --name bc_rnn_image_franka_stack_mimic_cosmos

.. note::
   기본적으로 학습된 model과 log는 ``IssacLab/logs/robomimic``에 저장됩니다.

Evaluation
^^^^^^^^^^

``robust_eval.py`` script는 simulation에서 학습된 visuomotor policy를 평가합니다.
이 평가는 policy가 서로 다른 visual variation에 얼마나 잘 generalize하는지,
그리고 visually augmented data가 policy robustness를 개선했는지 평가하는 데 도움이 됩니다.

아래는 evaluation에 사용되는 다양한 setting에 대한 설명입니다.

.. rubric:: Evaluation Settings

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``Vanilla``
      - Mimic data generation 중 사용한 것과 정확히 같은 setting입니다.
    * - ``Light Intensity``
      - light intensity/brightness가 변하고, 다른 aspect는 모두 동일하게 유지됩니다.
    * - ``Light Color``
      - light color가 변하고, 다른 aspect는 모두 동일하게 유지됩니다.
    * - ``Light Texture (Background)``
      - light texture/background가 변하고, 다른 aspect는 모두 동일하게 유지됩니다.
    * - ``Table Texture``
      - table의 visual texture가 변하고, 다른 aspect는 모두 동일하게 유지됩니다.
    * - ``Robot Arm Texture``
      - robot arm의 visual texture가 변하고, 다른 aspect는 모두 동일하게 유지됩니다.

.. rubric:: Required Arguments

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``--task``
      - environment 이름입니다.
    * - ``--input_dir``
      - 평가할 model checkpoint가 들어 있는 directory입니다.

.. rubric:: Optional Arguments

.. list-table::
    :widths: 30 70
    :header-rows: 0

    * - ``--start_epoch``
      - evaluation을 시작할 checkpoint epoch입니다. 기본값: 100
    * - ``--horizon``
      - 각 rollout의 step horizon입니다. 기본값: 400
    * - ``--num_rollouts``
      - setting별 model당 rollout 수입니다. 기본값: 15
    * - ``--num_seeds``
      - 평가할 random seed 수입니다. 기본값: 3
    * - ``--seeds``
      - random seed 대신 사용할 specific seed list입니다.
    * - ``--log_dir``
      - result를 기록할 directory입니다. 기본값: /tmp/policy_evaluation_results
    * - ``--log_file``
      - output file 이름입니다. 기본값: results
    * - ``--norm_factor_min``
      - action space normalization factor의 minimum value입니다.
    * - ``--norm_factor_max``
      - action space normalization factor의 maximum value입니다.
    * - ``--disable_fabric``
      - fabric을 비활성화하고 USD I/O operation을 사용할지 여부입니다.
    * - ``--enable_pinocchio``
      - IK controller에 Pinocchio를 활성화할지 여부입니다.

.. note::
    evaluation result는 visual augmentation이 policy performance와 robustness를 개선했는지 이해하는 데 도움이 됩니다.
    augmentation의 영향을 측정하려면 이 결과를 original dataset에 대한 evaluation과 비교하십시오.

cube stacking task의 사용 예시는 다음과 같습니다.

.. code:: bash

    ./isaaclab.sh -p scripts/imitation_learning/robomimic/robust_eval.py \
    --task Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Cosmos-v0 \
    --input_dir logs/robomimic/Isaac-Stack-Cube-Franka-IK-Rel-Visuomotor-Cosmos-v0/bc_rnn_image_franka_stack_mimic_cosmos/*/models \
    --log_dir robust_results/bc_rnn_image_franka_stack_mimic_cosmos \
    --log_file result \
    --enable_cameras \
    --seeds 0 \
    --num_rollouts 15 \
    --rendering_mode performance

.. note::
   이 script는 실행하는 데 하루 이상 또는 그보다 더 오래 걸릴 수 있습니다(사용 hardware에 따라 다름). 이는 예상된 동작입니다.

위 script를 사용해 각각 1000개의 Mimic-generated demonstration, 2000개의 Mimic-generated demonstration,
2000개의 Cosmos-Mimic-generated demonstration(1000 original mimic + 1000 Cosmos augmented)으로 학습한 model을 비교합니다.
세 model 모두에 동일한 seed(0, 1000, 5000)를 사용하고, 아래에는 각 seed의 best checkpoint 평균 metric을 제공합니다.

.. rubric:: Model Comparison

.. list-table::
    :widths: 25 25 25 25
    :header-rows: 0

    * - **Evaluation Setting**
      - **Mimic 1k Baseline**
      - **Mimic 2k Baseline**
      - **Cosmos-Mimic 2k**
    * - ``Vanilla``
      - 62%
      - 96.6%
      - 86.6%
    * - ``Light Intensity``
      - 11.1%
      - 20%
      - 62.2%
    * - ``Light Color``
      - 24.6%
      - 30%
      - 77.7%
    * - ``Light Texture (Background)``
      - 16.6%
      - 20%
      - 68.8%
    * - ``Table Texture``
      - 0%
      - 0%
      - 20%
    * - ``Robot Arm Texture``
      - 0%
      - 0%
      - 4.4%

위에서 학습한 model의 checkpoint는 model을 직접 사용하려는 경우 `여기 <https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Augmented/tree/main/robomimic_bc_rnn_visuomotor_models>`_ 에서 접근할 수 있습니다.
