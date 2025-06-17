## ARX Arm Quick Start

For the original ARX doc, please visit [arx5-sdk by Yihuai Gao](https://github.com/real-stanford/arx5-sdk).

Follow this guide to quickly test a single ARX arm using CAN communication.



## USB-CAN setup

We recommend using this [CAN adapter](https://www.amazon.com/DSD-TECH-Adapter-Hardware-Canable/dp/B0BQ5G3KLR/ref=asc_df_B0BQ5G3KLR?mcid=4fd05eea7a413a2dbe7ebca1e51c5f33&hvocijid=8032424359625485838-B0BQ5G3KLR-&hvexpln=73&tag=hyprod-20&linkCode=df0&hvadid=721245378154&hvpos=&hvnetw=g&hvrand=8032424359625485838&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1014226&hvtargid=pla-2281435178618&psc=1). For other types of CAN adapter connection, please visit [arx5-sdk](https://github.com/real-stanford/arx5-sdk).



``` sh
sudo apt install can-utils
sudo apt install net-tools
```

<p dir="auto">Plug in the CAN device and run the following command to check the available CAN devices.</p>
<div class="highlight highlight-source-shell notranslate position-relative overflow-auto" dir="auto" data-snippet-clipboard-copy-content="ls -l /sys/class/net/can*"><pre>ls -l /sys/class/net/can<span class="pl-k">*</span></pre></div>
<p dir="auto">This should give you something like this</p>

<div class="highlight highlight-source-shell notranslate position-relative overflow-auto" dir="auto" data-snippet-clipboard-copy-content="lrwxrwxrwx 1 root root 0 Jul 15 14:35 /sys/class/net/can0 -&gt; ../../devices/platform/soc/your_can_device/can0">
<pre><code>lrwxrwxrwx 1 root root 0 Jul 15 14:35 /sys/class/net/can0 -&gt; ../../devices/platform/soc/your_can_device/can0
</code></pre>
</div>


<p dir="auto">Where can0 is the CAN device name.</p>
<p dir="auto">You need to bring up the CAN interface with</p>


<div class="highlight highlight-source-shell notranslate position-relative overflow-auto" dir="auto" data-snippet-clipboard-copy-content="sudo ip link set can0 up type can bitrate 1000000">
<pre><code>sudo ip link <span class="pl-c1">set</span> can0 up <span class="pl-c1">type</span> can bitrate 1000000
</code></pre>
</div>




## Test scripts


Quick test a single ARX arm.

Arguments for `test_joint_control.py`, `keyboard_teleop.py`, `spacemouse_teleop.py` and `teach_replay.py`: 
- (required) model: `L5` (silver and black) or `L5` (all black metal with blue or red LED light). **Choosing the wrong model may lead to dangerous movements!**
- (required) interface: `can0`, `enx6c1ff70ac436` etc. (run `ip a` to check your interface name)
- (optional) urdf_path `-u`: by default `../models/arx5.urdf`

```bash
cd python
python examples/test_joint_control.py L5 can0 # replace L5 with X5 for the other model 
python examples/keyboard_teleop.py L5 can0
python examples/teach_replay.py L5 can0
```
