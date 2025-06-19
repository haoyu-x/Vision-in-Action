
## 3D Printing Guide

### [Robot sholder & neck mount](./shoulder) 

Bambu Lab 3D printer settings:
You can start by loading the 3MF file, which automatically applies all the necessary settings.
If that doesn't work, load the STL file and follow the detailed settings manually:

- **Shoulder Mount.stl**
  - `Process > Global`
    - `Support > Enable support`
  - Right click on part → Add modifier → Load… → `Shoulder Mount Modifier.stl`
  - `Process > Objects` → Select `Shoulder Mount Modifier.stl`
    - `Strength > Wall loops`: **8** (default is 2)
    - `Strength > Top shell thickness`: **5** (default is 1)

- **Neck Mount.stl**
  - Skip **“Top shell thickness”** modification, follow same process otherwise



### [iPhone mount](./iPhone_mounting)

### [ARX gello](./ARX_gello)



