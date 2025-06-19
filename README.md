<h1 align="center" style="font-size: 3em;">Vision in Action: Learning Active Perception from Human Demonstrations</h1>

[[Project page]](https://vision-in-action.github.io)
[[Paper]](https://arxiv.org/abs/2506.15666)


[Haoyu Xiong](https://haoyu-x.github.io/),
[Xiaomeng Xu](https://xxm19.github.io/),
[Jimmy Wu](https://jimmyyhwu.github.io/),
[Yifan Hou](https://yifan-hou.github.io/),
[Jeannette Bohg](https://web.stanford.edu/~bohg/),
[Shuran Song](https://shurans.github.io/)

<img width="50%" src="assets/teaser_v2.5.png">


## Installation

<p dir="auto">Tested on Ubuntu 22.04. We recommend <a href="https://github.com/conda-forge/miniforge#mambaforge">Mambaforge</a> for faster installation:</p>

<div class="highlight highlight-source-shell notranslate position-relative overflow-auto" dir="auto" data-snippet-clipboard-copy-content="cd Vision-in-Action
mamba env create -f environment.yaml
mamba activate via">
<pre><code>cd Vision-in-Action
mamba env create -f environment.yaml
mamba activate via</code></pre>
</div>


<p dir="auto">Install <a href="https://robostack.github.io/GettingStarted.html">ROS</a>.
<!-- Please build the ARX SDK first, before proceeding with the ROS installation. -->
</p>


<div class="highlight highlight-source-shell notranslate position-relative overflow-auto" dir="auto" data-snippet-clipboard-copy-content="conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults
mamba install ros-noetic-desktop-full
">
<pre><code>conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults
mamba install ros-noetic-desktop-full
</code></pre>
</div>

After completing the ROS installation, open a new terminal and run <code>roscore</code>. If everything is set up correctly, ROS should be installed successfully.


<p dir="auto">Build ARX robot SDK</p>


<div class="highlight highlight-source-shell notranslate position-relative overflow-auto" dir="auto" data-snippet-clipboard-copy-content="cd arx5-sdk
mkdir build && cd build
cmake ..
make -j">
<pre><code>cd arx5-sdk
mkdir build && cd build
cmake ..
make -j</code></pre>
</div>

## Table of Contents




### ⭐ Quick Start

👀 <a href="/async_point_cloud_render">Async Point Cloud Rendering.</a> Using an iPhone as a robot camera, use a VisionPro for VR rendering.

⚙️ <a href="/arx5-sdk">ARX Arm Setup.</a> Make sure you can run the single-arm test scripts after the USB-CAN setup.


### ⭐ The Vision in Action System

🛠️ <a href="/hardware">Hardware Guide.</a> 

📍 <a href="/data_collection">Data Collection & Processing.</a> 

🖥 <a href="/via_diffusion_policy#policy-training">Model Training.</a>

🤖 <a href="/via_diffusion_policy#policy-deployment">Robot Deployment.</a>

<!-- <li> <a href="/real-stanford/umi-on-legs/blob/main/mani-centric-wbc/docs/starter.md#downloads">Checkpoint &amp; Data</a></li>
<li>🕹️ <a href="/real-stanford/umi-on-legs/blob/main/mani-centric-wbc/docs/starter.md#rollout-controller">Rollout</a></li>
<li>📊 <a href="/real-stanford/umi-on-legs/blob/main/mani-centric-wbc/docs/starter.md#evaluation">Evaluation</a></li>
<li>📈 <a href="https://api.wandb.ai/links/columbia-ai-robotics/rrudtifq" rel="nofollow">Curves</a></li>
</ul> -->


<!-- 
<p dir="auto">
Using iPhone as a robot camera
</p>

<p dir="auto">
Using iPhone as a robot camera
</p> -->

## Acknowledgments




This project would not have been possible without the open-source contributions and support from the community. 

Our robot controller code is powered by 
[ARX-SDK](https://github.com/real-stanford/arx5-sdk),
The VR code is supported by 
[Vuer](https://github.com/vuer-ai/vuer) and 
[OpenTeleVision](https://github.com/OpenTeleVision/TeleVision).
Arm teleoperation is enabled using 
[Gello](https://github.com/wuphilipp/gello_software). 
The data processing code is adapted from 
[BiDex](https://github.com/leap-hand/Bidex_VisionPro_Teleop/tree/main). 
The mobile base is provided by 
[Tidybot++](https://tidybot2.github.io/docs/), 
and iPhone camera streaming is from 
[Record3D app](https://apps.apple.com/us/app/record3d-3d-videos/id1477716895?ls=1). 
Model training is based on the 
[Diffusion Policy](https://github.com/real-stanford/diffusion_policy/). 

## Citing


If you find this codebase useful, consider citing:

<pre><code>@misc{xiong2025visionactionlearningactive,
      title={Vision in Action: Learning Active Perception from Human Demonstrations}, 
      author={Haoyu Xiong and Xiaomeng Xu and Jimmy Wu and Yifan Hou and Jeannette Bohg and Shuran Song},
      year={2025},
      eprint={2506.15666},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2506.15666}, 
}</code></pre>

