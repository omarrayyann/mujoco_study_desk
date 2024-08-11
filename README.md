# MuJoCo Study Desk
**Part of the [MujocoAR](https://github.com/omarrayyann/MujocoAR) package demos**

A MuJoCo simulation environment of a study desk with a slider cabinet and a mug. The goal of this environment is to shwocase the position + rotation control of a frame using MuJoCo AR.


<table>
<!--   <tr>
    <th><code>camera_name="whole_view"</code></th>
    <th><code>camera_name="top_view"</code></th>
    <th><code>camera_name="side_view"</code></th>
    <th><code>camera_name="front_view"</code></th>
  </tr> -->
  <tr>
    <td><img src="https://github.com/user-attachments/assets/0e114e00-ce1d-4779-b3ac-e22d950273c2" width="500px" /></td>
    <td><img src="https://github.com/user-attachments/assets/5d1fdf9c-033b-4472-823e-fcb874b41db0" width="500px" /></td>
  </tr>
</table>

## MuJoCo AR Setup

```python
# MujocoAR Initialization
self.mujocoAR = MujocoARConnector(mujoco_model=self.mjmodel,mujoco_data=self.mjdata)

# Linking the target site with the AR position
self.mujocoAR.link_site(
    name="eef_target",
    scale=2.0,
    position_origin=self.pos_origin,
    rotation_origin=self.rot_origin,
    toggle_fn=lambda: setattr(self, 'grasp', not self.grasp),
)

# Start!
self.mujocoAR.start()
```

## Usage Guide

1. **Clone the repository**:

   ```bash
   git clone https://github.com/omarrayyann/mujoco_study_desk.git
   cd mujoco_study_desk
   
3. **Install MujocoAR and othe Requirements**:
   ```bash
   
   pip install mujoco_ar
   pip install requirements.txt
   
4. **Download the [MuJoCo AR App](https://apps.apple.com/jo/app/past-code/id1551535957) from the App Store.**
   
5. **Run the application**:

   ```bash
   mjpython main.py
   
6. **Enter the IP and Port shown into the app's start screen to start. Make sure to be connected to the same Wi-Fi network as the device. Incase of a latency, I recommend connecting to your phone's hotspot.**

## Author

Omar Rayyan (olr7742@nyu.edu)
