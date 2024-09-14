# Mathematics and Simulation for Robotics <br> 로봇수학및시뮬레이션 


<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Maintainer status: maintained**

- Maintainers
  - Jeongmin Jeon (nicky707@g.skku.edu)
  - Hyungpil Moon (hyungpil@g.skku.edu)
- Author
  - Jeongmin Jeon (nicky707@g.skku.edu)
  - Hong-ryul Jung (jung.hr.1206@g.skku.edu)
- Revision date: August 27, 2024

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [Overview](#overview)
- [Installation methods](#installation-methods)
    - [1. Tutorial](#1-tutorial-package)
    - [2. IsaacLab](#2-isaac-lab)

</div>
</div>

---

## Overview


- 성균관대학교 로봇수학및시뮬레이션 수업 Nvidia Isaac 튜토리얼을 위한 예제 패키지입니다.

---

## Installation methods



#### 1. Tutorial package

Tested on Isaac Sim v4.10 and Isaac Lab v1.1.0

```bash
sudo apt-get install git
git clone https://github.com/rise-lab-skku/isaac_tutorial
cd isaac_tutorial 
source isaac_tutorial.sh
```

#### 2. Isaac Lab
```bash
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout 5444fa399
cd source/extensions
pip install --editable omni.isaac.lab
pip install --editable omni.isaac.lab_assets
pip install --editable omni.isaac.lab_tasks
```
