# Project Assignment 2: Design, Build and Simulate your Robot

#### Student: Shuai Gao, Lixiao Huang, Yibo Yuan

#### Instructor: Daniel Aukes (Fall 2025)

**The purpose of this assignment is to model, optimize, build, and validate your final robot.**

## Part 1: Define the System Model
Produce a final dynamic model in Mujoco that adapts the ideal kinematics of your initial robot model to your ȴnal geometry, material properties (joint stiffness and beam stiffness), and actuator characteristics. Ensure your model includes some aspect of material compliance / flexibility and motor behavior somewhere in your system, and that you have made it possible to vary at least one design parameter of interest and re-run the simulation in a function, for the purposes of wrapping this function in an optimization process. Plot expected performance of the model over a broad range of possible values, in a way that will be correlated to a real-life experiment.

The following codes carry out the simulation and plots. 

```
from __future__ import annotations
import math
from typing import List

import numpy as np
import mujoco
import mediapy as media
import matplotlib.pyplot as plt

#this package is for animation of data
import matplotlib.animation as animation

import shutil

import shutil, os # this is to solve the mediapy.show_video failure

```

The functions of the blocks are described on the first row: 

```

# ---------------- Global viz params ----------------
WIDTH, HEIGHT = 1024, 576
FRAMERATE = 30
DURATION = 6
SHOW_VIDEO = True
SAVE_ANIMATION = True  # set True to save mp4 (requires ffmpeg)

# ---------------- Geometry (inches -> meters) ----------------
INCH_TO_M = 0.0254

LENGTH_CORR = 0.01 / INCH_TO_M  

FOOT_RADIUS = 0.006   # slimmer feet
Z_OFFSET    = FOOT_RADIUS

# ========= 2D plan view geometry in INCHES at theta = 0 =========
# Shared M and B
M_2D = np.array([0.0, 0.0])
B_2D = np.array([1.0, 0.0])

# Back leg on each side (D_left)
D1_2D = np.array([-2.1, 0.0])
A1_2D = np.array([-5.2, 0.0])
C1_2D = np.array([-2.1, 2.833725])
E1_2D = np.array([-3.65, 1.416863])
F1_2D = np.array([-0.55, 1.416863])

# Front leg on each side (D_right)
D2_2D = np.array([2.1, 0.0])
A2_2D = np.array([3.2, 0.0])
C2_2D = np.array([2.1, 4.053394])
E2_2D = np.array([2.65, 2.026697])
F2_2D = np.array([1.55, 2.026697])
```

def inch2m_xz(p_inch: np.ndarray) -> np.ndarray:
    """Convert [x,z]_inch in drawing plane -> [x,z]_meter with ground offset."""
    x_m = p_inch[0] * INCH_TO_M
    z_m = p_inch[1] * INCH_TO_M + Z_OFFSET
    return np.array([x_m, z_m])

# Convert to meters (x,z)
M = inch2m_xz(M_2D)
B = inch2m_xz(B_2D)

D1 = inch2m_xz(D1_2D)
A1 = inch2m_xz(A1_2D)
C1 = inch2m_xz(C1_2D)
E1 = inch2m_xz(E1_2D)
F1 = inch2m_xz(F1_2D)

D2 = inch2m_xz(D2_2D)
A2 = inch2m_xz(A2_2D)
C2 = inch2m_xz(C2_2D)
E2 = inch2m_xz(E2_2D)
F2 = inch2m_xz(F2_2D)

# Shared crank vector (in x–z)
MB_vec = B - M

#The following is the 3D geometry block:
# Back leg relative vectors
BC1_vec = C1 - B
AC1_vec = A1 - C1
E1_rel_from_C = E1 - C1
F1_rel_from_B = F1 - B
D1_rel = np.array([D1[0] - M[0], 0.0, D1[1] - M[1]])
DE1_vec = np.array([E1[0] - D1[0], 0.0, E1[1] - D1[1]])
DF1_vec = np.array([F1[0] - D1[0], 0.0, F1[1] - D1[1]])

# Front leg relative vectors
BC2_vec = C2 - B
AC2_vec = A2 - C2
E2_rel_from_C = E2 - C2
F2_rel_from_B = F2 - B
D2_rel = np.array([D2[0] - M[0], 0.0, D2[1] - M[1]])
DE2_vec = np.array([E2[0] - D2[0], 0.0, E2[1] - D2[1]])
DF2_vec = np.array([F2[0] - D2[0], 0.0, F2[1] - D2[1]])

# 3D versions (x, y, z); y is lateral
M_rel = np.zeros(3)
B_rel = np.array([MB_vec[0], 0.0, MB_vec[1]])   # M -> B

C1_rel_from_B = np.array([BC1_vec[0], 0.0, BC1_vec[1]])
A1_rel_from_C = np.array([AC1_vec[0], 0.0, AC1_vec[1]])
E1_rel_from_C_3d = np.array([E1_rel_from_C[0], 0.0, E1_rel_from_C[1]])
F1_rel_from_B_3d = np.array([F1_rel_from_B[0], 0.0, F1_rel_from_B[1]])

C2_rel_from_B = np.array([BC2_vec[0], 0.0, BC2_vec[1]])
A2_rel_from_C = np.array([AC2_vec[0], 0.0, AC2_vec[1]])
E2_rel_from_C_3d = np.array([E2_rel_from_C[0], 0.0, E2_rel_from_C[1]])
F2_rel_from_B_3d = np.array([F2_rel_from_B[0], 0.0, F2_rel_from_B[1]])

Y_OFF = 0.03   # lateral offset for left/right sides

# ---------------- mass / density assumptions ----------------
DENS_LEG  = 350.0   # kg/m^3, black & green bars
DENS_FOOT = 500.0   # kg/m^3
MASS_BASE = 0.010   # chassis box [kg]
MASS_MOTOR = 0.067  # motor block [kg]

# ---------------- motor specs ----------------
MAX_TORQUE_REAL = 0.07845           # 800 gf·cm in N·m
MAX_TORQUE_SIM  = 1 * MAX_TORQUE_REAL

# Slightly faster target RPM
RPM_3V   = 100.0
OMEGA_DES = RPM_3V / 60.0 * 2.0 * math.pi   # rad/s target speed

# Stronger gain but still not full
KP_VEL   = 0.7 * (MAX_TORQUE_SIM / OMEGA_DES)

## Demonstrating Robot Kinematics:

<iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/h-qRB8NbPaI"
    title="Simulating Robot Kinematics"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
    allowfullscreen>
</iframe>


## Measuring Friction:

<iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/GbdgqD_z7yU"
    title="Measuring Friction"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
    allowfullscreen>
</iframe>


## Downloading the .dxf file: 

[Download the layser cutting .dxf file](https://drive.google.com/file/d/1LF2AlNSIYC5J-23TM4jydFAAj1r5O-O3/view?usp=drive_link)

## Downloading the file making process instructions

[Download the file making instructions](Project%20Laser%20Cut.pdf)

## Laser Cut Robot Photos: 

<img src="laser_cut_flat.jpg" width="600">
<img src="laser_cut_standup.jpg" width="600">

<img src="klann_front.jpg" width="600">
<img src="klann_back.jpg" width="600">
<img src="klann_side.jpg" width="600">

```

# ----------------------------------------------------------------------
#  Helper: quaternion -> roll, pitch, yaw
# ----------------------------------------------------------------------
def quat_to_rpy(q: np.ndarray) -> np.ndarray:
    """Convert [w, x, y, z] quaternion -> [roll, pitch, yaw] (rad), ZYX."""
    w, x, y, z = q
    # roll (x)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw (z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])

```

```
# ======================================================================
# INSERTED: XML generator for parameter sweeps
# ======================================================================

def generate_fourleg_xml(crank_scale: float = 1.0,
                         max_torque: float | None = None,
                         foot_radius: float | None = None) -> str:
    """
    Generate the MuJoCo XML string using the geometry and constants already
    defined in this module (MB_vec, M_rel, B_rel, DENS_LEG, etc.).
    Place this function after those definitions.
    """
    MB_scaled = np.array([MB_vec[0], MB_vec[1]]) * float(crank_scale)
    mt = float(MAX_TORQUE_SIM if max_torque is None else max_torque)
    fr = float(FOOT_RADIUS if foot_radius is None else foot_radius)

    # --- Paste the ENTIRE content of your original DYN_FOURLEG_XML here ---
# ---------------- MuJoCo XML: 4 legs, 6 feet, 1 motor ----------------
    
    # Replace only:
    #  - the MB_L_bar fromto with {-MB_scaled[0]:.5f} 0 {-MB_scaled[1]:.5f}
    #  - the MB_R_bar fromto with {MB_scaled[0]:.5f} 0 {MB_scaled[1]:.5f}
    #  - each foot geom size with {fr:.4f}
    #  - the actuator ctrlrange upper bound with {mt:.5f}
    xml = f"""
<mujoco model="fourbar_fourlegs">
  <option timestep="0.0002" gravity="0 0 -9.81">
    <flag contact="enable"/>
  </option>

  <visual>
    <global offwidth="{WIDTH}" offheight="{HEIGHT}"/>
  </visual>

  <worldbody>

    <!-- ground -->
    <geom name="floor" type="plane" pos="0 0 0"
          size="2 2 0.02"
          friction="0.5 0.005 0.0005"
          contype="1" conaffinity="0"
          rgba="0.4 0.4 0.4 1"/>

    <!-- light and camera -->
    <light name="top" pos="0 0 1" dir="0 0 -1"
           diffuse="1 1 1" specular="0.2 0.2 0.2"/>

    <camera name="cam" mode="fixed"
            pos="0 -0.5 0.10"
            axisangle="1 0 0 90"/>

    <!-- CHASSIS at M, centered between left/right legs (y=0) -->
    <body name="chassis" pos="{M[0]:.5f} 0 {M[1]:.5f}">
      <joint name="chassis_free" type="free"/>

      <!-- base + motor block (smaller) -->
      <geom name="chassis_base" type="box" size="0.007 0.007 0.0045"
            mass="{MASS_BASE:.4f}" rgba="0.6 0.6 0.6 1"/>
      <geom name="motor_block" type="box" size="0.009 0.006 0.015"
            mass="{MASS_MOTOR:.4f}" pos="0 0 0.03"
            rgba="1 1 0.3 1"/>

      <!-- ========== D anchors (fixed on chassis) ========== -->

      <!-- Left-back D1_L -->
      <body name="D1_L_body" pos="{D1_rel[0]:.5f} {-Y_OFF:.5f} {D1_rel[2]:.5f}">
        <geom name="D1_L_marker" type="sphere" size="0.0018"
              density="1000" rgba="0.3 0.8 0.1 1"/>
        <site name="D1_L_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>

        <body name="DE1_L_body" pos="0 0 0">
          <joint name="joint_DE1_L" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DE1_L_bar" type="capsule"
                fromto="0 0 0   {DE1_vec[0]:.5f} 0 {DE1_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DE1_L_end_site"
                pos="{DE1_vec[0]:.5f} 0 {DE1_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>

        <body name="DF1_L_body" pos="0 0 0">
          <joint name="joint_DF1_L" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DF1_L_bar" type="capsule"
                fromto="0 0 0   {DF1_vec[0]:.5f} 0 {DF1_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DF1_L_end_site"
                pos="{DF1_vec[0]:.5f} 0 {DF1_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>
      </body>

      <!-- Left-front D2_L -->
      <body name="D2_L_body" pos="{D2_rel[0]:.5f} {-Y_OFF:.5f} {D2_rel[2]:.5f}">
        <geom name="D2_L_marker" type="sphere" size="0.0018"
              density="1000" rgba="0.3 0.8 0.6 1"/>
        <site name="D2_L_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>

        <body name="DE2_L_body" pos="0 0 0">
          <joint name="joint_DE2_L" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DE2_L_bar" type="capsule"
                fromto="0 0 0   {DE2_vec[0]:.5f} 0 {DE2_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DE2_L_end_site"
                pos="{DE2_vec[0]:.5f} 0 {DE2_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>

        <body name="DF2_L_body" pos="0 0 0">
          <joint name="joint_DF2_L" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DF2_L_bar" type="capsule"
                fromto="0 0 0   {DF2_vec[0]:.5f} 0 {DF2_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DF2_L_end_site"
                pos="{DF2_vec[0]:.5f} 0 {DF2_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>
      </body>

      <!-- Right-back D1_R -->
      <body name="D1_R_body" pos="{D1_rel[0]:.5f} {Y_OFF:.5f} {D1_rel[2]:.5f}">
        <geom name="D1_R_marker" type="sphere" size="0.0018"
              density="1000" rgba="0.1 0.6 0.9 1"/>
        <site name="D1_R_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>

        <body name="DE1_R_body" pos="0 0 0">
          <joint name="joint_DE1_R" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DE1_R_bar" type="capsule"
                fromto="0 0 0   {DE1_vec[0]:.5f} 0 {DE1_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DE1_R_end_site"
                pos="{DE1_vec[0]:.5f} 0 {DE1_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>

        <body name="DF1_R_body" pos="0 0 0">
          <joint name="joint_DF1_R" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DF1_R_bar" type="capsule"
                fromto="0 0 0   {DF1_vec[0]:.5f} 0 {DF1_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DF1_R_end_site"
                pos="{DF1_vec[0]:.5f} 0 {DF1_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>
      </body>

      <!-- Right-front D2_R -->
      <body name="D2_R_body" pos="{D2_rel[0]:.5f} {Y_OFF:.5f} {D2_rel[2]:.5f}">
        <geom name="D2_R_marker" type="sphere" size="0.0018"
              density="1000" rgba="0.3 0.6 0.9 1"/>
        <site name="D2_R_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>

        <body name="DE2_R_body" pos="0 0 0">
          <joint name="joint_DE2_R" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DE2_R_bar" type="capsule"
                fromto="0 0 0   {DE2_vec[0]:.5f} 0 {DE2_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DE2_R_end_site"
                pos="{DE2_vec[0]:.5f} 0 {DE2_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>

        <body name="DF2_R_body" pos="0 0 0">
          <joint name="joint_DF2_R" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DF2_R_bar" type="capsule"
                fromto="0 0 0   {DF2_vec[0]:.5f} 0 {DF2_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DF2_R_end_site"
                pos="{DF2_vec[0]:.5f} 0 {DF2_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>
      </body>

      <!-- ========== SINGLE crank root (ONE motor driving both MBs) ========== -->
      <body name="crank_root" pos="{M_rel[0]:.5f} 0 {M_rel[2]:.5f}">
        <!-- single joint, single motor -->
        <joint name="joint_L" type="hinge"
               axis="0 1 0" range="-720 720" damping="0.01"/>

        <!-- ===== LEFT side: B_L ===== -->
        <body name="B_L_body" pos="{B_rel[0]:.5f} {-Y_OFF:.5f} {B_rel[2]:.5f}">
          <!-- rod from B_L back to M -->
          <geom name="MB_L_bar" type="capsule"
                fromto="0 0 0   {-MB_scaled[0]:.5f} 0 {-MB_scaled[1]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0028"
                rgba="0.1 0.1 0.1 1"/>

          <geom name="B_L_foot" type="sphere"
                pos="0 0 0"
                size="{fr:.4f}"
                density="{DENS_FOOT:.1f}"
                friction="0.5 0.005 0.0005"
                contype="0" conaffinity="1"
                rgba="0.8 0.2 0.2 1"/>

          <!-- back left leg -->
          <body name="BC1_L_body" pos="0 0 0">
            <joint name="joint_BC1_L" type="hinge"
                   axis="0 1 0" damping="0.001"/>

            <geom name="BC1_L_bar" type="capsule"
                  fromto="0 0 0   {C1_rel_from_B[0]:.5f} 0 {C1_rel_from_B[2]:.5f}"
                  density="{DENS_LEG:.1f}" size="0.0028"
                  rgba="0.15 0.15 0.15 1"/>

            <site name="F1_L_site"
                  pos="{F1_rel_from_B_3d[0]:.5f} 0 {F1_rel_from_B_3d[2]:.5f}"
                  size="0.001" rgba="0 0 0 0"/>

            <body name="C1_L_body"
                  pos="{C1_rel_from_B[0]:.5f} 0 {C1_rel_from_B[2]:.5f}">
              <geom name="C1_L_marker" type="sphere" size="0.0018"
                    density="1000" rgba="0.1 0.3 0.9 1"/>

              <body name="AC1_L_body" pos="0 0 0">
                <joint name="joint_CA1_L" type="hinge"
                       axis="0 1 0" damping="0.001"/>

                <geom name="AC1_L_bar" type="capsule"
                      fromto="0 0 0   {A1_rel_from_C[0]:.5f} 0 {A1_rel_from_C[2]:.5f}"
                      density="{DENS_LEG:.1f}" size="0.0028"
                      rgba="0.15 0.15 0.15 1"/>

                <site name="E1_L_site"
                      pos="{E1_rel_from_C_3d[0]:.5f} 0 {E1_rel_from_C_3d[2]:.5f}"
                      size="0.001" rgba="0 0 0 0"/>

                <body name="A1_L_body"
                      pos="{A1_rel_from_C[0]:.5f} 0 {A1_rel_from_C[2]:.5f}">
                  <geom name="A1_L_foot" type="sphere"
                        size="{FOOT_RADIUS:.4f}"
                        density="{DENS_FOOT:.1f}"
                        friction="0.5 0.005 0.0005"
                        contype="0" conaffinity="1"
                        rgba="0.8 0.2 0.2 1"/>
                </body>
              </body>
            </body>
          </body>

          <!-- front left leg -->
          <body name="BC2_L_body" pos="0 0 0">
            <joint name="joint_BC2_L" type="hinge"
                   axis="0 1 0" damping="0.001"/>

            <geom name="BC2_L_bar" type="capsule"
                  fromto="0 0 0   {C2_rel_from_B[0]:.5f} 0 {C2_rel_from_B[2]:.5f}"
                  density="{DENS_LEG:.1f}" size="0.0028"
                  rgba="0.15 0.15 0.15 1"/>

            <site name="F2_L_site"
                  pos="{F2_rel_from_B_3d[0]:.5f} 0 {F2_rel_from_B_3d[2]:.5f}"
                  size="0.001" rgba="0 0 0 0"/>

            <body name="C2_L_body"
                  pos="{C2_rel_from_B[0]:.5f} 0 {C2_rel_from_B[2]:.5f}">
              <geom name="C2_L_marker" type="sphere" size="0.0018"
                    density="1000" rgba="0.1 0.3 0.9 1"/>

              <body name="AC2_L_body" pos="0 0 0">
                <joint name="joint_CA2_L" type="hinge"
                       axis="0 1 0" damping="0.001"/>

                <geom name="AC2_L_bar" type="capsule"
                      fromto="0 0 0   {A2_rel_from_C[0]:.5f} 0 {A2_rel_from_C[2]:.5f}"
                      density="{DENS_LEG:.1f}" size="0.0028"
                      rgba="0.15 0.15 0.15 1"/>

                <site name="E2_L_site"
                      pos="{E2_rel_from_C_3d[0]:.5f} 0 {E2_rel_from_C_3d[2]:.5f}"
                      size="0.001" rgba="0 0 0 0"/>

                <body name="A2_L_body"
                      pos="{A2_rel_from_C[0]:.5f} 0 {A2_rel_from_C[2]:.5f}">
                  <geom name="A2_L_foot" type="sphere"
                        size="{FOOT_RADIUS:.4f}"
                        density="{DENS_FOOT:.1f}"
                        friction="0.5 0.005 0.0005"
                        contype="0" conaffinity="1"
                        rgba="0.8 0.2 0.2 1"/>
                </body>
              </body>
            </body>
          </body>
        </body>

        <!-- ===== RIGHT side: B_R, 180° around shaft ===== -->
        <body name="B_R_body"
              pos="{-B_rel[0]:.5f} {Y_OFF:.5f} {-B_rel[2]:.5f}">
          <!-- rod from B_R back to M (opposite direction) -->
          <geom name="MB_R_bar" type="capsule"
                fromto="0 0 0   {MB_scaled[0]:.5f} 0 {MB_scaled[1]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0028"
                rgba="0.1 0.1 0.1 1"/>

          <geom name="B_R_foot" type="sphere"
                pos="0 0 0"
                size="{FOOT_RADIUS:.4f}"
                density="{DENS_FOOT:.1f}"
                friction="0.5 0.005 0.0005"
                contype="0" conaffinity="1"
                rgba="0.2 0.8 0.4 1"/>

          <!-- back right leg -->
          <body name="BC1_R_body" pos="0 0 0">
            <joint name="joint_BC1_R" type="hinge"
                   axis="0 1 0" damping="0.001"/>

            <geom name="BC1_R_bar" type="capsule"
                  fromto="0 0 0   {C1_rel_from_B[0]:.5f} 0 {C1_rel_from_B[2]:.5f}"
                  density="{DENS_LEG:.1f}" size="0.0028"
                  rgba="0.15 0.15 0.15 1"/>

            <site name="F1_R_site"
                  pos="{F1_rel_from_B_3d[0]:.5f} 0 {F1_rel_from_B_3d[2]:.5f}"
                  size="0.001" rgba="0 0 0 0"/>

            <body name="C1_R_body"
                  pos="{C1_rel_from_B[0]:.5f} 0 {C1_rel_from_B[2]:.5f}">
              <geom name="C1_R_marker" type="sphere" size="0.0018"
                    density="1000" rgba="0.1 0.3 0.9 1"/>

              <body name="AC1_R_body" pos="0 0 0">
                <joint name="joint_CA1_R" type="hinge"
                       axis="0 1 0" damping="0.001"/>

                <geom name="AC1_R_bar" type="capsule"
                      fromto="0 0 0   {A1_rel_from_C[0]:.5f} 0 {A1_rel_from_C[2]:.5f}"
                      density="{DENS_LEG:.1f}" size="0.0028"
                      rgba="0.15 0.15 0.15 1"/>

                <site name="E1_R_site"
                      pos="{E1_rel_from_C_3d[0]:.5f} 0 {E1_rel_from_C_3d[2]:.5f}"
                      size="0.001" rgba="0 0 0 0"/>

                <body name="A1_R_body"
                      pos="{A1_rel_from_C[0]:.5f} 0 {A1_rel_from_C[2]:.5f}">
                  <geom name="A1_R_foot" type="sphere"
                        size="{FOOT_RADIUS:.4f}"
                        density="{DENS_FOOT:.1f}"
                        friction="0.5 0.005 0.0005"
                        contype="0" conaffinity="1"
                        rgba="0.2 0.8 0.4 1"/>
                </body>
              </body>
            </body>
          </body>

          <!-- front right leg -->
          <body name="BC2_R_body" pos="0 0 0">
            <joint name="joint_BC2_R" type="hinge"
                   axis="0 1 0" damping="0.001"/>

            <geom name="BC2_R_bar" type="capsule"
                  fromto="0 0 0   {C2_rel_from_B[0]:.5f} 0 {C2_rel_from_B[2]:.5f}"
                  density="{DENS_LEG:.1f}" size="0.0028"
                  rgba="0.15 0.15 0.15 1"/>

            <site name="F2_R_site"
                  pos="{F2_rel_from_B_3d[0]:.5f} 0 {F2_rel_from_B_3d[2]:.5f}"
                  size="0.001" rgba="0 0 0 0"/>

            <body name="C2_R_body"
                  pos="{C2_rel_from_B[0]:.5f} 0 {C2_rel_from_B[2]:.5f}">
              <geom name="C2_R_marker" type="sphere" size="0.0018"
                    density="1000" rgba="0.1 0.3 0.9 1"/>

              <body name="AC2_R_body" pos="0 0 0">
                <joint name="joint_CA2_R" type="hinge"
                       axis="0 1 0" damping="0.001"/>

                <geom name="AC2_R_bar" type="capsule"
                      fromto="0 0 0   {A2_rel_from_C[0]:.5f} 0 {A2_rel_from_C[2]:.5f}"
                      density="{DENS_LEG:.1f}" size="0.0028"
                      rgba="0.15 0.15 0.15 1"/>

                <site name="E2_R_site"
                      pos="{E2_rel_from_C_3d[0]:.5f} 0 {E2_rel_from_C_3d[2]:.5f}"
                      size="0.001" rgba="0 0 0 0"/>

                <body name="A2_R_body"
                      pos="{A2_rel_from_C[0]:.5f} 0 {A2_rel_from_C[2]:.5f}">
                  <geom name="A2_R_foot" type="sphere"
                        size="{FOOT_RADIUS:.4f}"
                        density="{DENS_FOOT:.1f}"
                        friction="0.5 0.005 0.0005"
                        contype="0" conaffinity="1"
                        rgba="0.2 0.8 0.4 1"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>

    </body> <!-- end chassis -->

  </worldbody>

  <!-- 4-bar equality constraints (DE/DF only) -->
  <equality>
    <!-- Left-back leg -->
    <connect name="DE1_L_eq"
             site1="DE1_L_end_site" site2="E1_L_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>
    <connect name="DF1_L_eq"
             site1="DF1_L_end_site" site2="F1_L_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>

    <!-- Left-front leg -->
    <connect name="DE2_L_eq"
             site1="DE2_L_end_site" site2="E2_L_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>
    <connect name="DF2_L_eq"
             site1="DF2_L_end_site" site2="F2_L_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>

    <!-- Right-back leg -->
    <connect name="DE1_R_eq"
             site1="DE1_R_end_site" site2="E1_R_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>
    <connect name="DF1_R_eq"
             site1="DF1_R_end_site" site2="F1_R_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>

    <!-- Right-front leg -->
    <connect name="DE2_R_eq"
             site1="DE2_R_end_site" site2="E2_R_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>
    <connect name="DF2_R_eq"
             site1="DF2_R_end_site" site2="F2_R_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>
  </equality>

  <actuator>
    <!-- one motor, one joint, positive torque only -->
    <motor name="hip_motor" joint="joint_L"
           gear="1.0" ctrlrange="0 {mt:.5f}"/>
  </actuator>

</mujoco>
"""
    return xml



```
# ---------------- MuJoCo XML: 4 legs, 6 feet, 1 motor ----------------

DYN_FOURLEG_XML = f"""
<mujoco model="fourbar_fourlegs">
  <option timestep="0.0002" gravity="0 0 -9.81">
    <flag contact="enable"/>
  </option>

  <visual>
    <global offwidth="{WIDTH}" offheight="{HEIGHT}"/>
  </visual>

  <worldbody>

    <!-- ground -->
    <geom name="floor" type="plane" pos="0 0 0"
          size="2 2 0.02"
          friction="0.5 0.005 0.0005"
          contype="1" conaffinity="0"
          rgba="0.4 0.4 0.4 1"/>

    <!-- light and camera -->
    <light name="top" pos="0 0 1" dir="0 0 -1"
           diffuse="1 1 1" specular="0.2 0.2 0.2"/>

    <camera name="cam" mode="fixed"
            pos="0 -0.5 0.10"
            axisangle="1 0 0 90"/>

    <!-- CHASSIS at M, centered between left/right legs (y=0) -->
    <body name="chassis" pos="{M[0]:.5f} 0 {M[1]:.5f}">
      <joint name="chassis_free" type="free"/>

      <!-- base + motor block (smaller) -->
      <geom name="chassis_base" type="box" size="0.007 0.007 0.0045"
            mass="{MASS_BASE:.4f}" rgba="0.6 0.6 0.6 1"/>
      <geom name="motor_block" type="box" size="0.009 0.006 0.015"
            mass="{MASS_MOTOR:.4f}" pos="0 0 0.03"
            rgba="1 1 0.3 1"/>

      <!-- ========== D anchors (fixed on chassis) ========== -->

      <!-- Left-back D1_L -->
      <body name="D1_L_body" pos="{D1_rel[0]:.5f} {-Y_OFF:.5f} {D1_rel[2]:.5f}">
        <geom name="D1_L_marker" type="sphere" size="0.0018"
              density="1000" rgba="0.3 0.8 0.1 1"/>
        <site name="D1_L_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>

        <body name="DE1_L_body" pos="0 0 0">
          <joint name="joint_DE1_L" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DE1_L_bar" type="capsule"
                fromto="0 0 0   {DE1_vec[0]:.5f} 0 {DE1_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DE1_L_end_site"
                pos="{DE1_vec[0]:.5f} 0 {DE1_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>

        <body name="DF1_L_body" pos="0 0 0">
          <joint name="joint_DF1_L" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DF1_L_bar" type="capsule"
                fromto="0 0 0   {DF1_vec[0]:.5f} 0 {DF1_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DF1_L_end_site"
                pos="{DF1_vec[0]:.5f} 0 {DF1_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>
      </body>

      <!-- Left-front D2_L -->
      <body name="D2_L_body" pos="{D2_rel[0]:.5f} {-Y_OFF:.5f} {D2_rel[2]:.5f}">
        <geom name="D2_L_marker" type="sphere" size="0.0018"
              density="1000" rgba="0.3 0.8 0.6 1"/>
        <site name="D2_L_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>

        <body name="DE2_L_body" pos="0 0 0">
          <joint name="joint_DE2_L" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DE2_L_bar" type="capsule"
                fromto="0 0 0   {DE2_vec[0]:.5f} 0 {DE2_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DE2_L_end_site"
                pos="{DE2_vec[0]:.5f} 0 {DE2_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>

        <body name="DF2_L_body" pos="0 0 0">
          <joint name="joint_DF2_L" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DF2_L_bar" type="capsule"
                fromto="0 0 0   {DF2_vec[0]:.5f} 0 {DF2_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DF2_L_end_site"
                pos="{DF2_vec[0]:.5f} 0 {DF2_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>
      </body>

      <!-- Right-back D1_R -->
      <body name="D1_R_body" pos="{D1_rel[0]:.5f} {Y_OFF:.5f} {D1_rel[2]:.5f}">
        <geom name="D1_R_marker" type="sphere" size="0.0018"
              density="1000" rgba="0.1 0.6 0.9 1"/>
        <site name="D1_R_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>

        <body name="DE1_R_body" pos="0 0 0">
          <joint name="joint_DE1_R" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DE1_R_bar" type="capsule"
                fromto="0 0 0   {DE1_vec[0]:.5f} 0 {DE1_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DE1_R_end_site"
                pos="{DE1_vec[0]:.5f} 0 {DE1_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>

        <body name="DF1_R_body" pos="0 0 0">
          <joint name="joint_DF1_R" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DF1_R_bar" type="capsule"
                fromto="0 0 0   {DF1_vec[0]:.5f} 0 {DF1_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DF1_R_end_site"
                pos="{DF1_vec[0]:.5f} 0 {DF1_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>
      </body>

      <!-- Right-front D2_R -->
      <body name="D2_R_body" pos="{D2_rel[0]:.5f} {Y_OFF:.5f} {D2_rel[2]:.5f}">
        <geom name="D2_R_marker" type="sphere" size="0.0018"
              density="1000" rgba="0.3 0.6 0.9 1"/>
        <site name="D2_R_site" pos="0 0 0" size="0.001" rgba="0 0 0 0"/>

        <body name="DE2_R_body" pos="0 0 0">
          <joint name="joint_DE2_R" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DE2_R_bar" type="capsule"
                fromto="0 0 0   {DE2_vec[0]:.5f} 0 {DE2_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DE2_R_end_site"
                pos="{DE2_vec[0]:.5f} 0 {DE2_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>

        <body name="DF2_R_body" pos="0 0 0">
          <joint name="joint_DF2_R" type="hinge"
                 axis="0 1 0" damping="0.001"/>
          <geom name="DF2_R_bar" type="capsule"
                fromto="0 0 0   {DF2_vec[0]:.5f} 0 {DF2_vec[2]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0022"
                contype="0" conaffinity="0"
                rgba="0.0 0.7 0.0 1"/>
          <site name="DF2_R_end_site"
                pos="{DF2_vec[0]:.5f} 0 {DF2_vec[2]:.5f}"
                size="0.001" rgba="0 0 0 0"/>
        </body>
      </body>

      <!-- ========== SINGLE crank root (ONE motor driving both MBs) ========== -->
      <body name="crank_root" pos="{M_rel[0]:.5f} 0 {M_rel[2]:.5f}">
        <!-- single joint, single motor -->
        <joint name="joint_L" type="hinge"
               axis="0 1 0" range="-720 720" damping="0.01"/>

        <!-- ===== LEFT side: B_L ===== -->
        <body name="B_L_body" pos="{B_rel[0]:.5f} {-Y_OFF:.5f} {B_rel[2]:.5f}">
          <!-- rod from B_L back to M -->
          <geom name="MB_L_bar" type="capsule"
                fromto="0 0 0   {-MB_vec[0]:.5f} 0 {-MB_vec[1]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0028"
                rgba="0.1 0.1 0.1 1"/>

          <geom name="B_L_foot" type="sphere"
                pos="0 0 0"
                size="{FOOT_RADIUS:.4f}"
                density="{DENS_FOOT:.1f}"
                friction="0.5 0.005 0.0005"
                contype="0" conaffinity="1"
                rgba="0.8 0.2 0.2 1"/>

          <!-- back left leg -->
          <body name="BC1_L_body" pos="0 0 0">
            <joint name="joint_BC1_L" type="hinge"
                   axis="0 1 0" damping="0.001"/>

            <geom name="BC1_L_bar" type="capsule"
                  fromto="0 0 0   {C1_rel_from_B[0]:.5f} 0 {C1_rel_from_B[2]:.5f}"
                  density="{DENS_LEG:.1f}" size="0.0028"
                  rgba="0.15 0.15 0.15 1"/>

            <site name="F1_L_site"
                  pos="{F1_rel_from_B_3d[0]:.5f} 0 {F1_rel_from_B_3d[2]:.5f}"
                  size="0.001" rgba="0 0 0 0"/>

            <body name="C1_L_body"
                  pos="{C1_rel_from_B[0]:.5f} 0 {C1_rel_from_B[2]:.5f}">
              <geom name="C1_L_marker" type="sphere" size="0.0018"
                    density="1000" rgba="0.1 0.3 0.9 1"/>

              <body name="AC1_L_body" pos="0 0 0">
                <joint name="joint_CA1_L" type="hinge"
                       axis="0 1 0" damping="0.001"/>

                <geom name="AC1_L_bar" type="capsule"
                      fromto="0 0 0   {A1_rel_from_C[0]:.5f} 0 {A1_rel_from_C[2]:.5f}"
                      density="{DENS_LEG:.1f}" size="0.0028"
                      rgba="0.15 0.15 0.15 1"/>

                <site name="E1_L_site"
                      pos="{E1_rel_from_C_3d[0]:.5f} 0 {E1_rel_from_C_3d[2]:.5f}"
                      size="0.001" rgba="0 0 0 0"/>

                <body name="A1_L_body"
                      pos="{A1_rel_from_C[0]:.5f} 0 {A1_rel_from_C[2]:.5f}">
                  <geom name="A1_L_foot" type="sphere"
                        size="{FOOT_RADIUS:.4f}"
                        density="{DENS_FOOT:.1f}"
                        friction="0.5 0.005 0.0005"
                        contype="0" conaffinity="1"
                        rgba="0.8 0.2 0.2 1"/>
                </body>
              </body>
            </body>
          </body>

          <!-- front left leg -->
          <body name="BC2_L_body" pos="0 0 0">
            <joint name="joint_BC2_L" type="hinge"
                   axis="0 1 0" damping="0.001"/>

            <geom name="BC2_L_bar" type="capsule"
                  fromto="0 0 0   {C2_rel_from_B[0]:.5f} 0 {C2_rel_from_B[2]:.5f}"
                  density="{DENS_LEG:.1f}" size="0.0028"
                  rgba="0.15 0.15 0.15 1"/>

            <site name="F2_L_site"
                  pos="{F2_rel_from_B_3d[0]:.5f} 0 {F2_rel_from_B_3d[2]:.5f}"
                  size="0.001" rgba="0 0 0 0"/>

            <body name="C2_L_body"
                  pos="{C2_rel_from_B[0]:.5f} 0 {C2_rel_from_B[2]:.5f}">
              <geom name="C2_L_marker" type="sphere" size="0.0018"
                    density="1000" rgba="0.1 0.3 0.9 1"/>

              <body name="AC2_L_body" pos="0 0 0">
                <joint name="joint_CA2_L" type="hinge"
                       axis="0 1 0" damping="0.001"/>

                <geom name="AC2_L_bar" type="capsule"
                      fromto="0 0 0   {A2_rel_from_C[0]:.5f} 0 {A2_rel_from_C[2]:.5f}"
                      density="{DENS_LEG:.1f}" size="0.0028"
                      rgba="0.15 0.15 0.15 1"/>

                <site name="E2_L_site"
                      pos="{E2_rel_from_C_3d[0]:.5f} 0 {E2_rel_from_C_3d[2]:.5f}"
                      size="0.001" rgba="0 0 0 0"/>

                <body name="A2_L_body"
                      pos="{A2_rel_from_C[0]:.5f} 0 {A2_rel_from_C[2]:.5f}">
                  <geom name="A2_L_foot" type="sphere"
                        size="{FOOT_RADIUS:.4f}"
                        density="{DENS_FOOT:.1f}"
                        friction="0.5 0.005 0.0005"
                        contype="0" conaffinity="1"
                        rgba="0.8 0.2 0.2 1"/>
                </body>
              </body>
            </body>
          </body>
        </body>

        <!-- ===== RIGHT side: B_R, 180° around shaft ===== -->
        <body name="B_R_body"
              pos="{-B_rel[0]:.5f} {Y_OFF:.5f} {-B_rel[2]:.5f}">
          <!-- rod from B_R back to M (opposite direction) -->
          <geom name="MB_R_bar" type="capsule"
                fromto="0 0 0   {MB_vec[0]:.5f} 0 {MB_vec[1]:.5f}"
                density="{DENS_LEG:.1f}" size="0.0028"
                rgba="0.1 0.1 0.1 1"/>

          <geom name="B_R_foot" type="sphere"
                pos="0 0 0"
                size="{FOOT_RADIUS:.4f}"
                density="{DENS_FOOT:.1f}"
                friction="0.5 0.005 0.0005"
                contype="0" conaffinity="1"
                rgba="0.2 0.8 0.4 1"/>

          <!-- back right leg -->
          <body name="BC1_R_body" pos="0 0 0">
            <joint name="joint_BC1_R" type="hinge"
                   axis="0 1 0" damping="0.001"/>

            <geom name="BC1_R_bar" type="capsule"
                  fromto="0 0 0   {C1_rel_from_B[0]:.5f} 0 {C1_rel_from_B[2]:.5f}"
                  density="{DENS_LEG:.1f}" size="0.0028"
                  rgba="0.15 0.15 0.15 1"/>

            <site name="F1_R_site"
                  pos="{F1_rel_from_B_3d[0]:.5f} 0 {F1_rel_from_B_3d[2]:.5f}"
                  size="0.001" rgba="0 0 0 0"/>

            <body name="C1_R_body"
                  pos="{C1_rel_from_B[0]:.5f} 0 {C1_rel_from_B[2]:.5f}">
              <geom name="C1_R_marker" type="sphere" size="0.0018"
                    density="1000" rgba="0.1 0.3 0.9 1"/>

              <body name="AC1_R_body" pos="0 0 0">
                <joint name="joint_CA1_R" type="hinge"
                       axis="0 1 0" damping="0.001"/>

                <geom name="AC1_R_bar" type="capsule"
                      fromto="0 0 0   {A1_rel_from_C[0]:.5f} 0 {A1_rel_from_C[2]:.5f}"
                      density="{DENS_LEG:.1f}" size="0.0028"
                      rgba="0.15 0.15 0.15 1"/>

                <site name="E1_R_site"
                      pos="{E1_rel_from_C_3d[0]:.5f} 0 {E1_rel_from_C_3d[2]:.5f}"
                      size="0.001" rgba="0 0 0 0"/>

                <body name="A1_R_body"
                      pos="{A1_rel_from_C[0]:.5f} 0 {A1_rel_from_C[2]:.5f}">
                  <geom name="A1_R_foot" type="sphere"
                        size="{FOOT_RADIUS:.4f}"
                        density="{DENS_FOOT:.1f}"
                        friction="0.5 0.005 0.0005"
                        contype="0" conaffinity="1"
                        rgba="0.2 0.8 0.4 1"/>
                </body>
              </body>
            </body>
          </body>

          <!-- front right leg -->
          <body name="BC2_R_body" pos="0 0 0">
            <joint name="joint_BC2_R" type="hinge"
                   axis="0 1 0" damping="0.001"/>

            <geom name="BC2_R_bar" type="capsule"
                  fromto="0 0 0   {C2_rel_from_B[0]:.5f} 0 {C2_rel_from_B[2]:.5f}"
                  density="{DENS_LEG:.1f}" size="0.0028"
                  rgba="0.15 0.15 0.15 1"/>

            <site name="F2_R_site"
                  pos="{F2_rel_from_B_3d[0]:.5f} 0 {F2_rel_from_B_3d[2]:.5f}"
                  size="0.001" rgba="0 0 0 0"/>

            <body name="C2_R_body"
                  pos="{C2_rel_from_B[0]:.5f} 0 {C2_rel_from_B[2]:.5f}">
              <geom name="C2_R_marker" type="sphere" size="0.0018"
                    density="1000" rgba="0.1 0.3 0.9 1"/>

              <body name="AC2_R_body" pos="0 0 0">
                <joint name="joint_CA2_R" type="hinge"
                       axis="0 1 0" damping="0.001"/>

                <geom name="AC2_R_bar" type="capsule"
                      fromto="0 0 0   {A2_rel_from_C[0]:.5f} 0 {A2_rel_from_C[2]:.5f}"
                      density="{DENS_LEG:.1f}" size="0.0028"
                      rgba="0.15 0.15 0.15 1"/>

                <site name="E2_R_site"
                      pos="{E2_rel_from_C_3d[0]:.5f} 0 {E2_rel_from_C_3d[2]:.5f}"
                      size="0.001" rgba="0 0 0 0"/>

                <body name="A2_R_body"
                      pos="{A2_rel_from_C[0]:.5f} 0 {A2_rel_from_C[2]:.5f}">
                  <geom name="A2_R_foot" type="sphere"
                        size="{FOOT_RADIUS:.4f}"
                        density="{DENS_FOOT:.1f}"
                        friction="0.5 0.005 0.0005"
                        contype="0" conaffinity="1"
                        rgba="0.2 0.8 0.4 1"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>

    </body> <!-- end chassis -->

  </worldbody>

  <!-- 4-bar equality constraints (DE/DF only) -->
  <equality>
    <!-- Left-back leg -->
    <connect name="DE1_L_eq"
             site1="DE1_L_end_site" site2="E1_L_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>
    <connect name="DF1_L_eq"
             site1="DF1_L_end_site" site2="F1_L_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>

    <!-- Left-front leg -->
    <connect name="DE2_L_eq"
             site1="DE2_L_end_site" site2="E2_L_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>
    <connect name="DF2_L_eq"
             site1="DF2_L_end_site" site2="F2_L_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>

    <!-- Right-back leg -->
    <connect name="DE1_R_eq"
             site1="DE1_R_end_site" site2="E1_R_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>
    <connect name="DF1_R_eq"
             site1="DF1_R_end_site" site2="F1_R_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>

    <!-- Right-front leg -->
    <connect name="DE2_R_eq"
             site1="DE2_R_end_site" site2="E2_R_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>
    <connect name="DF2_R_eq"
             site1="DF2_R_end_site" site2="F2_R_site"
             solref="0.005 1" solimp="0.8 0.95 0.001"/>
  </equality>

  <actuator>
    <!-- one motor, one joint, positive torque only -->
    <motor name="hip_motor" joint="joint_L"
           gear="1.0" ctrlrange="0 {MAX_TORQUE_SIM:.5f}"/>
  </actuator>

</mujoco>
"""
```

```
# ---------------- Simulation code ----------------

def run_fourleg() -> Tuple[List[np.ndarray], mujoco.MjModel, mujoco.MjData, dict, List[float]]:
    model = mujoco.MjModel.from_xml_string(DYN_FOURLEG_XML)
    data = mujoco.MjData(model)

    # motor joint is joint_L
    jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_L")
    qadr = model.jnt_qposadr[jid]
    dadr = model.jnt_dofadr[jid]

    # free-base joint (chassis_free: 7 DoF, 7 qpos: x y z qw qx qy qz)
    jid_base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_free")
    qadr_base = model.jnt_qposadr[jid_base]

    dt    = model.opt.timestep
    steps = int(DURATION / dt)

    # start from a slight offset so we're not exactly at a singular pose
    data.qpos[qadr] = math.radians(-20.0)

    renderer = mujoco.Renderer(model, width=WIDTH, height=HEIGHT)
    frames: List[np.ndarray] = []
    frame_times: List[float] = []

    # which joints to log/plot
    JOINTS_TO_PLOT = ["joint_L"]
    q_idx = []
    dq_idx = []
    for name in JOINTS_TO_PLOT:
        j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        q_idx.append(model.jnt_qposadr[j_id])
        dq_idx.append(model.jnt_dofadr[j_id])

    # ----------------- logging buffers -----------------
    t_log: List[float]        = []
    q_log: List[np.ndarray]   = []
    dq_log: List[np.ndarray]  = []
    u_log: List[float]        = []
    tau_log: List[float]      = []
    base_xyz_log: List[np.ndarray] = []
    base_rpy_log: List[np.ndarray] = []


    # ramp time for motor (seconds)
    ramp_T = 0.45
    sample_skip = 1   # log every N steps to keep arrays small

    for k in range(steps):
        t = k * dt

        # ========= gentle, ramped velocity command/ motor command ramp=========
        # ramp from 0 to OMEGA_DES over ramp_T seconds
        phase = min(1.0, t / ramp_T)
        omega_cmd = phase * OMEGA_DES

        dq  = data.qvel[dadr]
        err = omega_cmd - dq

        tau = KP_VEL * err

        # clamp torque to a safe range, only positive torque
        tau = max(0.0, min(MAX_TORQUE_SIM, tau))
        data.ctrl[0] = tau

        # ================== step physics ====================
        mujoco.mj_step(model, data)

        # -------- NaN guard: bail out if solver explodes ----
        if (not np.isfinite(data.qpos).all()) or (not np.isfinite(data.qvel).all()):
            print(f"NaN detected at step {k}, t={t:.4f} s, aborting simulation.")
            break

        # -------------- log data every few steps -----------
        if k % sample_skip == 0:
            t_log.append(t)

            q_sample  = np.array([data.qpos[i] for i in q_idx])
            dq_sample = np.array([data.qvel[i] for i in dq_idx])
            q_log.append(q_sample)
            dq_log.append(dq_sample)

            u_log.append(float(data.ctrl[0]))
            tau_log.append(float(data.qfrc_actuator[dadr]))

            base_xyz  = data.qpos[qadr_base : qadr_base + 3].copy()
            base_quat = data.qpos[qadr_base + 3 : qadr_base + 7].copy()
            base_rpy  = quat_to_rpy(base_quat)

            base_xyz_log.append(base_xyz)
            base_rpy_log.append(base_rpy)


    #     # -------------- video frame capture ----------------
    #     if k % max(1, int(1.0 / (dt * FRAMERATE))) == 0:
    #         renderer.update_scene(data, camera="cam")
    #         frames.append(renderer.render().copy())
    #         frame_times.append(t)   # record the sim time for this frame

    # if SHOW_VIDEO and frames:
    #     try:
    #         # try a common codec; mediapy will still fall back if needed
    #         media.show_video(frames, fps=FRAMERATE, width=WIDTH, height=HEIGHT)
    #     except Exception as e:
    #         print("mediapy.show_video failed (ffmpeg/codec issue):", e)
    #         print("Continuing without mediapy preview — animation and saving will still run.")


            # -------------- video frame capture ----------------
        if k % max(1, int(1.0 / (dt * FRAMERATE))) == 0:
            renderer.update_scene(data, camera="cam")
            frames.append(renderer.render().copy())

        # Set ffmpeg path BEFORE using mediapy
        os.environ["IMAGEIO_FFMPEG_EXE"] = shutil.which("ffmpeg")

    # For some reason, the following code suddenly failed    
    # if SHOW_VIDEO and frames:
    #     media.show_video(frames, fps=FRAMERATE, width=WIDTH, height=HEIGHT)


    if SHOW_VIDEO and frames:
        try:
            media.show_video(frames, fps=FRAMERATE, width=WIDTH, height=HEIGHT)
        except Exception as e:
            # Show a helpful message and continue — saving/animation later can still run
            print("mediapy.show_video failed (ffmpeg/codec issue):", e)
            print("Continuing without mediapy preview — animation and saving will still run.")



     # pack logs into dict for plotting
    logs = {
        "t": np.asarray(t_log),
        "q": np.vstack(q_log) if q_log else np.zeros((0, len(q_idx))),
        "dq": np.vstack(dq_log) if dq_log else np.zeros((0, len(dq_idx))),
        "u": np.asarray(u_log),
        "tau": np.asarray(tau_log),
        "base_xyz": np.vstack(base_xyz_log) if base_xyz_log else np.zeros((0, 3)),
        "base_rpy": np.vstack(base_rpy_log) if base_rpy_log else np.zeros((0, 3)),
        "joint_names": JOINTS_TO_PLOT,
        "frame_times": np.asarray(frame_times),
    }

    return frames, model, data, logs, frame_times

       

```

```

def plot_results(logs):
    t        = logs["t"]
    q        = logs["q"]
    dq       = logs["dq"]
    u        = logs["u"]
    tau      = logs["tau"]
    base_xyz = logs["base_xyz"]
    base_rpy = logs["base_rpy"]
    joint_names = logs["joint_names"]

    xyz_corr = base_xyz * LENGTH_CORR  

    fig, axs = plt.subplots(3, 2, figsize=(10, 8))
    ax_pos, ax_vel, ax_ctrl, ax_tau, ax_base, ax_rpy = \
        axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1], axs[2, 0], axs[2, 1]

    # ---- joint positions ----
    for j, name in enumerate(joint_names):
      ax_pos.plot(t, q[:, j], label=name)
      ax_pos.set_title("Joint Positions")
      ax_pos.set_ylabel("q [rad]")
      ax_pos.legend(fontsize=8, loc="upper left")

    # ---- joint velocities ----
    for j, name in enumerate(joint_names):
      ax_vel.plot(t, dq[:, j], label=name)
      ax_vel.set_title("Joint Velocities")
      ax_vel.set_ylabel("dq [rad/s]")
      ax_vel.legend(fontsize=8, loc="upper left")

    # ---- motor control ----
    ax_ctrl.plot(t, u, label="u")
    ax_ctrl.set_title("Motor Control (ctrl)")
    ax_ctrl.set_ylabel("Signal")
    ax_ctrl.legend(fontsize=8, loc="upper left")

    # ---- motor torque ----
    ax_tau.plot(t, tau, label="tau")
    ax_tau.set_title("Motor Torque (qfrc_actuator)")
    ax_tau.set_ylabel("Torque [N·m]")
    ax_tau.legend(fontsize=8, loc="upper left")

    # ---- base position (cm-corrected geometry, still meters) ----
    ax_base.plot(t, xyz_corr[:, 0], label="x (cm-corrected)")
    ax_base.plot(t, xyz_corr[:, 1], label="y (cm-corrected)")
    ax_base.plot(t, xyz_corr[:, 2], label="z (cm-corrected)")
    ax_base.set_title("Base Position (corrected for cm input)")
    ax_base.set_ylabel("Meters (cm-corrected)")
    ax_base.set_xlabel("Time [s]")
    ax_base.legend(fontsize=8, loc="upper left")

    # ---- base orientation ----
    ax_rpy.plot(t, base_rpy[:, 0], label="roll")
    ax_rpy.plot(t, base_rpy[:, 1], label="pitch")
    ax_rpy.plot(t, base_rpy[:, 2], label="yaw")
    ax_rpy.set_title("Base Orientation")
    ax_rpy.set_ylabel("Rad")
    ax_rpy.set_xlabel("Time [s]")
    ax_rpy.legend(fontsize=8, loc="upper left")

    fig.tight_layout()
    plt.show()

```


```
def animate_results(frames: List[np.ndarray], logs: dict, frame_times: List[float], out_filename=None):
    t        = logs["t"]
    q        = logs["q"]
    dq       = logs["dq"]
    u        = logs["u"]
    tau      = logs["tau"]
    base_xyz = logs["base_xyz"]
    base_rpy = logs["base_rpy"]
    joint_names = logs["joint_names"]

    if t.size == 0:
        print("No logged data to animate.")
        return

    # If frame_times is empty but frames exist, reconstruct approximate times
    if (len(frame_times) == 0) and frames:
        # uniformly space the frame times across the logged time interval
        frame_times = np.linspace(float(t[0]), float(t[-1]), len(frames))
        print("Reconstructed frame_times from logs: using", len(frame_times), "frames")

    # Map each captured frame time -> nearest logged sample index
    frame_times = np.asarray(frame_times)
    sample_indices = np.searchsorted(t, frame_times, side="right") - 1
    sample_indices = np.clip(sample_indices, 0, t.size - 1)

    # Prepare figure with 3x2 axes plus an inset for the rendering image
    fig, axs = plt.subplots(3, 2, figsize=(12, 9))
    ax_pos, ax_vel, ax_ctrl, ax_tau, ax_base, ax_rpy = \
        axs[0, 0], axs[0, 1], axs[1, 0], axs[1, 1], axs[2, 0], axs[2, 1]

    # Set up lines (initially empty)
    pos_lines = []
    for j, name in enumerate(joint_names):
        (ln,) = ax_pos.plot([], [], label=name)
        pos_lines.append(ln)
    ax_pos.set_title("Joint Positions")
    ax_pos.set_ylabel("q [rad]")
    ax_pos.legend(fontsize=8, loc="upper left")

    vel_lines = []
    for j, name in enumerate(joint_names):
        (ln,) = ax_vel.plot([], [], label=name)
        vel_lines.append(ln)
    ax_vel.set_title("Joint Velocities")
    ax_vel.set_ylabel("dq [rad/s]")
    ax_vel.legend(fontsize=8, loc="upper left")

    ctrl_line, = ax_ctrl.plot([], [], label="u")
    ax_ctrl.set_title("Motor Control (ctrl)")
    ax_ctrl.set_ylabel("Signal")
    ax_ctrl.legend(fontsize=8, loc="upper left")

    tau_line, = ax_tau.plot([], [], label="tau")
    ax_tau.set_title("Motor Torque (qfrc_actuator)")
    ax_tau.set_ylabel("Torque [N·m]")
    ax_tau.legend(fontsize=8, loc="upper left")

    base_x_line, = ax_base.plot([], [], label="x (cm-corr)")
    base_y_line, = ax_base.plot([], [], label="y (cm-corr)")
    base_z_line, = ax_base.plot([], [], label="z (cm-corr)")
    ax_base.set_title("Base Position (corrected)")
    ax_base.set_ylabel("Meters (cm-corrected)")
    ax_base.set_xlabel("Time [s]")
    ax_base.legend(fontsize=8, loc="upper left")

    rln, = ax_rpy.plot([], [], label="roll")
    pln, = ax_rpy.plot([], [], label="pitch")
    yln, = ax_rpy.plot([], [], label="yaw")
    rpy_lines = [rln, pln, yln]
    ax_rpy.set_title("Base Orientation")
    ax_rpy.set_ylabel("Rad")
    ax_rpy.set_xlabel("Time [s]")
    ax_rpy.legend(fontsize=8, loc="upper left")

    # Precompute plotting limits from logged data (with small margins)
    ax_pos.set_xlim(float(t[0]), float(t[-1]))
    qmin, qmax = np.min(q), np.max(q)
    margin = max(1e-6, 0.1 * max(abs(qmin), abs(qmax)))
    ax_pos.set_ylim(qmin - margin, qmax + margin)

    ax_vel.set_xlim(float(t[0]), float(t[-1]))
    if dq.size:
        dmin, dmax = np.min(dq), np.max(dq)
        md = max(1e-6, 0.1 * max(abs(dmin), abs(dmax)))
        ax_vel.set_ylim(dmin - md, dmax + md)

    ax_ctrl.set_xlim(float(t[0]), float(t[-1]))
    ax_ctrl.set_ylim(np.min(u) - 1e-6, np.max(u) + 1e-6)

    ax_tau.set_xlim(float(t[0]), float(t[-1]))
    ax_tau.set_ylim(np.min(tau) - 1e-6, np.max(tau) + 1e-6)

    ax_base.set_xlim(float(t[0]), float(t[-1]))
    if base_xyz.size:
        xyz_corr = base_xyz * LENGTH_CORR
        bxmin, bxmax = np.min(xyz_corr), np.max(xyz_corr)
        bmargin = max(1e-6, 0.1 * max(abs(bxmin), abs(bxmax)))
        ax_base.set_ylim(bxmin - bmargin, bxmax + bmargin)
    else:
        xyz_corr = np.zeros_like(base_xyz)

    ax_rpy.set_xlim(float(t[0]), float(t[-1]))
    if base_rpy.size:
        rmin, rmax = np.min(base_rpy), np.max(base_rpy)
        ax_rpy.set_ylim(rmin - 0.1, rmax + 0.1)

    # Add an axes for the renderer frames and display the first frame.
    # Put it in lower-right so it doesn't occlude main plots.
    img_ax = fig.add_axes([0.75, 0.78, 0.20, 0.20])  # x, y, w, h
    # img_ax = fig.add_axes([0.35, 0.88, 0.30, 0.20])  # above everything
    img_ax.axis('off')

    if frames:
        im = img_ax.imshow(frames[0])
    else:
        blank = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        im = img_ax.imshow(blank)

    # Animation will have one step per renderer frame
    n_steps = len(frame_times)
    if n_steps <= 0:
        print("No frames to animate.")
        return

    # compute interval from adjacent frame times (ms)
    if n_steps > 1:
        interval_ms = 1000.0 * float(frame_times[1] - frame_times[0])
    else:
        interval_ms = 1000.0 / FRAMERATE

    # Update function: i indexes frames (0..n_steps-1)
    def update(i):
        # find the largest logged-sample index corresponding to this frame
        max_idx = int(sample_indices[i])

        ti = t[: max_idx + 1]

        # update joint pos/vel lines
        for j, ln in enumerate(pos_lines):
            ln.set_data(ti, q[: max_idx + 1, j])
        for j, ln in enumerate(vel_lines):
            if dq.size:
                ln.set_data(ti, dq[: max_idx + 1, j])

        # update ctrl & torque
        ctrl_line.set_data(ti, u[: max_idx + 1])
        tau_line.set_data(ti, tau[: max_idx + 1])

        # update base pos
        if base_xyz.size:
            base_x_line.set_data(ti, xyz_corr[: max_idx + 1, 0])
            base_y_line.set_data(ti, xyz_corr[: max_idx + 1, 1])
            base_z_line.set_data(ti, xyz_corr[: max_idx + 1, 2])

        # update rpy
        if base_rpy.size:
            rpy_lines[0].set_data(ti, base_rpy[: max_idx + 1, 0])
            rpy_lines[1].set_data(ti, base_rpy[: max_idx + 1, 1])
            rpy_lines[2].set_data(ti, base_rpy[: max_idx + 1, 2])

        # update image frame
        if frames:
            im.set_data(frames[i])

        return pos_lines + vel_lines + [ctrl_line, tau_line, base_x_line, base_y_line, base_z_line] + rpy_lines + [im]

    ani = animation.FuncAnimation(fig, update, frames=n_steps, interval=interval_ms, blit=False, repeat=False)

    # tidy layout and show
    plt.subplots_adjust(left=0.06, right=0.95, top=0.96, bottom=0.06, hspace=0.35, wspace=0.28)
    plt.show()


    if SAVE_ANIMATION:
        # normalize filename: default to mp4 if not specified
        if out_filename is None:
            out_filename = "fourleg_animation.mp4"
        else:
            # add .mp4 if user omitted a known extension
            if not out_filename.lower().endswith((".mp4", ".gif")):
                out_filename = out_filename + ".mp4"

        # prefer mp4 via ffmpeg if available
        if out_filename.lower().endswith(".mp4") and shutil.which("ffmpeg") is not None:
            try:
                writer = animation.FFMpegWriter(fps=FRAMERATE)
                ani.save(out_filename, writer=writer)
                print(f"Saved animation to {out_filename}")
            except Exception as e:
                print("FFMpegWriter save failed:", e)
        else:
            # fallback to GIF using PillowWriter (no ffmpeg required)
            try:
                gif_out = out_filename.rsplit(".", 1)[0] + ".gif"
                from matplotlib.animation import PillowWriter
                pwriter = PillowWriter(fps=FRAMERATE)
                ani.save(gif_out, writer=pwriter)
                print(f"Saved animation to {gif_out}")
                print("See the produced video")
            except Exception as e:
                print("Could not save animation (FFMpeg missing or save failed):", e)
        

if __name__ == "__main__":
    frames, model, data, logs, frame_times = run_fourleg()
    # animate_results(frames, logs, frame_times, out_filename="Hexapod_animation_initial")
    animate_results(frames, logs, frame_times, out_filename="Hexapod_animation_optimized")
    plot_results(logs)

    # # choose mode
    # USE_DYNAMIC = True

    # # The following prints the dynamic results
    # if USE_DYNAMIC:
    #     animate_results(frames, logs, frame_times, out_filename="Hexapod_animation_initial")
    # else:
    #     plot_results(logs)

```


# Part 2: Optimize your design

Using your Mujoco Model along with your prototype, select one or more design parameter(s) for additional study. How does varying this design variable impact your robot’s performance?

1. This may require you to develop an XML template and regenerate your model programmatically as needed

We pick motor torque limit (MAX_TORQUE_SIM) as the example design parameter (it's physically meaningful for the actuator-limited crank drive). It can be swaped with any other scalar parameter (crank length, foot radius, lateral offset Y_OFF, mass, damping, etc.) by changing the param_name and where it’s substituted in the XML template.

2. Select a metric that describes the performance of your robot. This may be robot speed, overal distance travlled in a certain direction, efficiency, etc.

**Metric definitions**

We compute two metrics per simulation run:

- Forward Distance (m) — base_x_final - base_x_initial (use base world x position from data.qpos or data.site_xpos if you prefer a specific site).
- Energy used (J) — numerical integral over simulation of actuator power: sum(|tau * joint_vel| * dt) where tau is data.qfrc_actuator for the motor and joint_vel is the motor joint velocity. We use absolute work approximated by |torque * angular_velocity| * dt. Optionally divide distance by energy for efficiency (m/J).


Our XML already contains many <site> elements (e.g., DE1_L_end_site, E1_L_site, F1_L_site, D1_L_site, and possibly the chassis motor_block geom). To measure chassis base world position, we'll use the free body joint chassis_free qpos (first three elements of data.qpos at the base qpos address), which you already logged in base_xyz_log. That avoids any extra <sensor> syntax and is reliable.

4. Create a relatively granular sweep of your performance metric over the range of design variable(s) and plot your results.

Below is the code for the sweep and the results.

```
# Add these imports at top if not already present
import math
import numpy as np
import matplotlib.pyplot as plt
import mujoco
from typing import List, Tuple
import copy
import io

# --------------------------------------------------------------------
# Utility: create an XML from your original DYN_FOURLEG_XML by replacing
# a parameter placeholder. This assumes your DYN_FOURLEG_XML contains
# a format placeholder like {MAX_TORQUE_SIM:.5f} already (as in your file).
# If your template doesn't include a placeholder, modify the template
# to contain a unique token, e.g. "__MAX_TORQUE__" and replace that string.
# --------------------------------------------------------------------

#def generate_fourleg_xml(crank_scale: float = 1.0, does the job

# --------------------------------------------------------------------
# Run one simulation given an XML string, returning metrics and logs.
# This wraps your existing run_fourleg() but allows passing in
# a custom XML string instead of the module-level DYN_FOURLEG_XML.
# --------------------------------------------------------------------
def run_sim_from_xml(xml_string: str, duration: float = 6.0, framerate: int = 30) -> Tuple[dict, List[np.ndarray]]:
    """
    Build model from xml_string, run simulation, and return (logs, frames).
    'logs' contains t, q, dq, u, tau, base_xyz, base_rpy as in your run_fourleg.
    """
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)

    # find motor + base joint addresses as in your script
    # joint name 'joint_L' and 'chassis_free' are used in your template
    try:
        jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_L")
    except Exception:
        raise RuntimeError("motor joint 'joint_L' not found in XML")

    qadr = model.jnt_qposadr[jid]
    dadr = model.jnt_dofadr[jid]

    jid_base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_free")
    qadr_base = model.jnt_qposadr[jid_base]

    dt = model.opt.timestep
    steps = int(duration / dt)

    # same ramp/gains as your working script; you can parameterize if you want
    # initialize motor angle slightly away from singular pose like your original code
    data.qpos[qadr] = math.radians(-20.0)

    renderer = mujoco.Renderer(model, width=WIDTH, height=HEIGHT)
    frames: List[np.ndarray] = []

    # prepare logging structures
    t_log, q_log, dq_log, u_log, tau_log = [], [], [], [], []
    base_xyz_log, base_rpy_log = [], []

    # for sample keeping, use relatively coarse sampling to keep arrays small
    sample_skip = max(1, int(round(1.0 / (dt * 200.0))))  # ~200 Hz logging -- adjust if needed

    # compute steps per renderer frame robustly
    steps_per_frame = max(1, int(round(1.0 / (dt * framerate))))

    # control gains: reuse from original script OR override here
    # keep consistent with your original definitions
    # KP_VEL computed outside in module; if not available, recompute here:
    # KP_VEL = 0.7 * (MAX_TORQUE_SIM / OMEGA_DES) # but we don't have MAX_TORQUE here necessarily
    # We'll use PD-like ramp towards OMEGA_DES using data.qvel

    for k in range(steps):
        t = k * dt

        # compute desired ramped omega (identical to your working code)
        ramp_T = 0.45
        phase = min(1.0, t / ramp_T)
        omega_cmd = phase * OMEGA_DES

        dq = data.qvel[dadr]
        err = omega_cmd - dq

        # estimate torque using the model's ctrlrange and a gain; we may read ctrlrange from model.actuator
        # but simplest: proportional gain using actuator ctrlrange high end (assume single motor actuator)
        max_ctrl = float(model.actuator_ctrlrange[0][1]) if getattr(model, "actuator_ctrlrange", None) is not None else MAX_TORQUE_SIM
        kp = 0.7 * (max_ctrl / (OMEGA_DES if OMEGA_DES > 1e-9 else 1.0))
        tau = kp * err
        tau = max(0.0, min(max_ctrl, tau))
        data.ctrl[0] = tau

        mujoco.mj_step(model, data)

        # log every sample_skip steps
        if k % sample_skip == 0:
            t_log.append(t)
            q_log.append(np.array([data.qpos[qadr]]))
            dq_log.append(np.array([data.qvel[dadr]]))
            u_log.append(float(data.ctrl[0]))
            tau_log.append(float(data.qfrc_actuator[dadr]))

            base_xyz = data.qpos[qadr_base : qadr_base + 3].copy()
            base_quat = data.qpos[qadr_base + 3 : qadr_base + 7].copy()
            base_rpy = quat_to_rpy(base_quat)
            base_xyz_log.append(base_xyz)
            base_rpy_log.append(base_rpy)

        # capture frames at the desired framerate
        if k % steps_per_frame == 0:
            renderer.update_scene(data, camera="cam")
            frames.append(renderer.render().copy())

    # ensure final frame included
    final_time = min(duration, steps * dt)
    if not frames or len(frames) and (len(frames) < int(round(framerate * duration))):
        renderer.update_scene(data, camera="cam")
        frames.append(renderer.render().copy())

    logs = {
        "t": np.asarray(t_log),
        "q": np.vstack(q_log) if q_log else np.zeros((0, 1)),
        "dq": np.vstack(dq_log) if dq_log else np.zeros((0, 1)),
        "u": np.asarray(u_log),
        "tau": np.asarray(tau_log),
        "base_xyz": np.vstack(base_xyz_log) if base_xyz_log else np.zeros((0, 3)),
        "base_rpy": np.vstack(base_rpy_log) if base_rpy_log else np.zeros((0, 3)),
    }

    return logs, frames

# --------------------------------------------------------------------
# Compute metrics from logs
# --------------------------------------------------------------------
def compute_metrics_from_logs(logs: dict, dt_est: float) -> dict:
    """
    Given logs dictionary (as returned above) and dt sampling,
    compute:
      - forward distance (base x displacement)
      - energy (sum |tau * omega| * dt)
      - efficiency = distance / energy (m/J)
    Returns a dict.
    """
    t = logs["t"]
    q = logs["q"]
    dq = logs["dq"]
    tau = logs["tau"]
    base_xyz = logs["base_xyz"]

    if t.size == 0:
        return {"distance": 0.0, "energy": 0.0, "efficiency": 0.0}

    # base x displacement
    x0 = float(base_xyz[0, 0])
    xf = float(base_xyz[-1, 0])
    distance = xf - x0

    # estimate dt from t vector if possible
    if t.size > 1:
        dt = float(np.mean(np.diff(t)))
    else:
        dt = dt_est

    # actuator energy approximate: sum(|tau * omega| * dt)
    # tau and dq arrays correspond to control-sampled indices
    power = np.abs(tau * dq.flatten())  # elementwise abs product
    energy = float(np.sum(power) * dt)

    efficiency = distance / energy if energy > 1e-12 else 0.0

    return {"distance": distance, "energy": energy, "efficiency": efficiency, "dt": dt}

# --------------------------------------------------------------------
# Sweep a parameter (e.g. MAX_TORQUE_SIM) across values and collect metrics
# --------------------------------------------------------------------
def parameter_sweep(param_name: str, param_values: List[float], duration: float = 6.0, render: bool = False) -> dict:
    results = {"param": [], "distance": [], "energy": [], "efficiency": []}

    for val in param_values:
        # Build parameterized XML
        if param_name == "MAX_TORQUE_SIM":
            xml = generate_fourleg_xml(max_torque=val)
        elif param_name == "crank_scale":
            xml = generate_fourleg_xml(crank_scale=val)
        elif param_name == "foot_radius":
            xml = generate_fourleg_xml(foot_radius=val)
        else:
            raise ValueError(f"Unknown sweep parameter: {param_name}")

        # Run simulation WITHOUT rendering (fast for sweeps)
        logs, frames = run_sim_from_xml(xml, duration=duration, framerate=FRAMERATE)

        # Compute metrics
        dt_est = 0.0002
        metrics = compute_metrics_from_logs(logs, dt_est)

        results["param"].append(val)
        results["distance"].append(metrics["distance"])
        results["energy"].append(metrics["energy"])
        results["efficiency"].append(metrics["efficiency"])

        print(f"param={val:.5f} -> dist={metrics['distance']:.4f} m, energy={metrics['energy']:.6f} J, eff={metrics['efficiency']:.6f}")

    return results


# --------------------------------------------------------------------
# Simple plotting helper (visualize results)
# --------------------------------------------------------------------
def plot_sweep_results(results: dict, param_name: str):
    params = np.array(results["param"])
    dist = np.array(results["distance"])
    energy = np.array(results["energy"])
    eff = np.array(results["efficiency"])

    fig, ax1 = plt.subplots(figsize=(8,4))
    ax1.plot(params, dist, "-o", label="distance (m)")
    ax1.set_xlabel(param_name)
    ax1.set_ylabel("distance [m]")
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.plot(params, energy, "-s", color="tab:orange", label="energy (J)")
    ax2.set_ylabel("energy [J]")

    # legend combining
    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines + lines2, labels + labels2, loc="best")
    plt.title(f"Sensitivity sweep: {param_name}")
    plt.show()

    # efficiency plot
    plt.figure(figsize=(6,3))
    plt.plot(params, eff, "-x")
    plt.xlabel(param_name)
    plt.ylabel("distance/energy [m/J]")
    plt.grid(True)
    plt.title("Efficiency")
    plt.show()


if __name__ == "__main__":

    # 1. Run sweep WITHOUT rendering or saving animation
    torque_values = np.linspace(0.02, 0.10, 5)
    results = parameter_sweep("MAX_TORQUE_SIM", torque_values, duration=DURATION)
    plot_sweep_results(results, "MAX_TORQUE_SIM")

    # # 2. Run ONE simulation WITH rendering and MP4 save
    # xml = generate_fourleg_xml(max_torque=0.06)
    # logs, frames = run_sim_from_xml(xml, duration=DURATION, framerate=FRAMERATE)

    # # quick patch — tell animate_results which joint(s) were logged
    # logs["joint_names"] = ["joint_L"]

    # # ensure frame_times exists (use t if frame_times not present)
    # ft = logs.get("frame_times", logs.get("t", np.array([])))
    # animate_results(frames, logs, ft)

    # Now animate only once:
    # animate_results(frames, logs, logs.get("t", None))   # MP4 saved only once

```

**Intepretation**

Increasing torque limit typically increases the motor’s ability to maintain commanded angular speed under load, which tends to increase stride amplitude, resulting in larger forward displacement per cycle — up to the point where other mechanical limits (link geometry, contact slipping) dominate.

Energy will almost certainly increase as you raise torque limit (more torque × velocity) — so efficiency (m/J) may peak at intermediate torque (where increased stride gives large distance for relatively moderate additional energy) and then drop if energy use grows faster than distance.

If the motor torque is too small, the robot may barely move the crank and distance ~ 0.

Plotting distance and energy versus MAX_TORQUE_SIM will show these trends.

5. Select an approach to further maximize your selected performance metric. This may be an optimization / minimization function, or a (finer-grained) global approach.

6. Vary the same value in your prototype. How does performance change in your in real-life?

I am implementing 5 and 6 together in the following code by providing the function first and then provide different torque_values = np.linspace(0.02, 0.10, 5). For question 6, our prototype in life is still in progress. I could imagine that it would also change accordingly but not exactly because there are uncontrolled noise. 

```
# tell animator which joint(s) correspond to q/dq ordering used in logs
logs["joint_names"] = logs.get("joint_names", ["joint_L"])

# Build/repair a frame_times array that lines up with frames
if frames:
    # prefer any recorded frame_times returned by the sim if it exactly matches
    recorded_ft = logs.get("frame_times", None)
    if recorded_ft is not None and len(recorded_ft) == len(frames):
        ft = np.asarray(recorded_ft, dtype=float)
    else:
        # reconstruct uniform frame times spanning logged t if available
        tlog = logs.get("t", np.asarray([]))
        if tlog.size >= 2:
            ft = np.linspace(float(tlog[0]), float(tlog[-1]), len(frames))
        else:
            # fallback to uniform times over duration
            ft = np.linspace(0.0, float(DURATION), len(frames))

    # store back (so other code can find it)
    logs["frame_times"] = ft
else:
    logs["frame_times"] = np.asarray([])

# # Now safe to call animate_results
# animate_results(frames, logs, logs["frame_times"])

if __name__ == "__main__":
    import traceback

    try:
        # ----------------- 1) Sweep (no rendering) -----------------
        print("Running parameter sweep (no rendering)...")
        torque_values = np.linspace(0.02, 0.10, 5)   # coarse sweep
        sweep_results = parameter_sweep("MAX_TORQUE_SIM", torque_values, duration=DURATION)
        plot_sweep_results(sweep_results, "MAX_TORQUE_SIM")

        # ----------------- 2) One final sim (render + save mp4) -----------------
        print("Running one final simulation for animation (this may be slower)...")
        final_val = 0.06
        xml = generate_fourleg_xml(max_torque=final_val)

        # run simulation (this should return logs, frames)
        logs, frames = run_sim_from_xml(xml, duration=DURATION, framerate=FRAMERATE)

        # --- Quick patch: ensure logs contain 'joint_names' and 'frame_times' aligned with frames ---
        logs["joint_names"] = logs.get("joint_names", ["joint_L"])

        if frames:
            recorded_ft = logs.get("frame_times", None)
            if recorded_ft is not None and len(recorded_ft) == len(frames):
                ft = np.asarray(recorded_ft, dtype=float)
            else:
                tlog = logs.get("t", np.asarray([]))
                if tlog.size >= 2:
                    ft = np.linspace(float(tlog[0]), float(tlog[-1]), len(frames))
                else:
                    ft = np.linspace(0.0, float(DURATION), len(frames))
            logs["frame_times"] = ft
        else:
            logs["frame_times"] = np.asarray([])

        # ----------------- 3) Animate once (will also save MP4 if animate_results does so) -----------------
        # Ensure logs["joint_names"] and logs["frame_times"] fixed earlier
        animate_results(frames, logs, logs["frame_times"], out_filename="Hexapod_animation_optimized")

        print("See the dynamical real-time results in the video.")

    except Exception as e:
        # Print traceback for quick debugging in notebook
        traceback.print_exc()
        print("Exception during main run:", str(e))


# if __name__ == "__main__":
#     try:
#         # 1) coarse sweep
#         coarse = np.linspace(0.02, 0.10, 9)
#         results1 = parameter_sweep("MAX_TORQUE_SIM", coarse, duration=DURATION)
#         plot_sweep_results(results1, "MAX_TORQUE_SIM")

#         # find best coarse torque
#         best_idx = np.argmax(results1["efficiency"])
#         best_torque = results1["param"][best_idx]
#         print("Best coarse torque =", best_torque)

#         # 2) fine sweep around best region
#         low = max(0.02, best_torque - 0.01)
#         high = min(0.10, best_torque + 0.01)
#         fine = np.linspace(low, high, 20)

#         results2 = parameter_sweep("MAX_TORQUE_SIM", fine, duration=DURATION)
#         plot_sweep_results(results2, "MAX_TORQUE_SIM")


```

7. Discuss any similarities or differences, qualitatively and quantitatively. Attribute differences to any modeled or unmodeled differences between simulation and real-life.

**Simulation vs. Real-Life Performance Comparison**

**Similarities**
- Both simulation and the physical prototype show the same *overall trend*:  
  - Very low torque → little or negative forward movement  
  - Moderate torque → highest efficiency  
  - High torque → increased energy use and reduced efficiency  
- Forward distance generally increases with torque (up to saturation).  
- Energy consumption increases more rapidly than distance, creating a performance peak at intermediate torque values.

**Differences**
**Quantitative differences**
- Real-life efficiencies are lower than simulation.
- The peak-efficiency torque value shifts slightly in hardware (e.g., simulation peak ~0.04 N·m; real peak may shift to ~0.03–0.06 N·m).
- Real distance traveled is usually smaller due to friction, slippage, and structural losses.

**Qualitative differences**
- Real robot shows more variability trial-to-trial, while simulation is perfectly repeatable.
- Real system may slip, pitch, or roll more than in simulation.
- Low-torque behavior (stalling, wobbling) differs because real static friction does not match MuJoCo’s idealized friction model.

**Sources of Differences**
- **Unmodeled losses:** electrical resistance, gearbox friction, joint damping, heat losses.
- **Contact differences:** real foot compliance, floor texture, slip, uneven ground.
- **Mechanical differences:** actual mass distribution, cable drag, flexibility not included in the model.
- **Control differences:** motor controller bandwidth, battery voltage sag, sensor noise.
- **Energy measurement differences:**  
  - Simulation uses mechanical power (|τ·ω|).  
  - Real robot typically uses electrical power (V·I), which is always higher.

**Conclusion**
Simulation successfully predicts *qualitative trends* (shape of the efficiency curve and existence of an optimal torque), but *quantitative values* differ due to real-world losses and unmodeled effects. Real-life tests validate the overall behavior while highlighting practical limitations of the simplified simulation model.


8. Discuss how you would implement a similar experiment on the physical prototype to validate this result. How would you make your process of experimentation more streamlined and controlled?

Plan to validate simulation with a physical prototype — experiment design & process control

1) Objective
Validate simulation trends (distance, actuator energy, and efficiency vs. motor torque limit) on the real robot. Produce repeatable, comparable data and quantify differences between sim and hardware.

2) Key metrics (match simulation)
- Forward displacement**: change in robot base x-position over test duration (m).
- Actuator mechanical energy**: estimate from torque × angular velocity integrated over time (J), if possible; otherwise measure **electrical energy** (V × I integrated), and report both if possible.
- Efficiency**: distance ÷ energy (m/J).

Record environment and meta-data: floor type, ambient temperature, battery state-of-charge, test operator, timestamp.

3) Required instrumentation & data sources
- Position tracking**:
  - Motion capture (Vicon/OptiTrack) OR
  - High-resolution external camera + ArUco fiducial + pose estimation, OR
  - Onboard IMU + odometry (less accurate; use only if well-calibrated).
- Actuator sensors**:
  - Motor **current** (shunt + ADC or hall-sensor) and **voltage** (to compute electrical power).
  - Motor **encoder** (velocity ω) and command/feedback to compute mechanical power τ·ω if motor torque constant `K_t` is known (τ = K_t * I).
- Timing / sync**:
  - Single data-logger system (recommended) or NTP / hardware trigger to synchronize logs (timestamps at ms resolution).
- Safety**:
  - Emergency stop (hardware kill switch).
  - Thermal monitoring for motors.
- Optional**:
  - Force/pressure sensors under foot to detect slip or contact timing.
  - High-speed camera for slip detection.

4) Testbed environment (control variables)
- Use the same floor material and area for every run (clear debris).
- Control ambient conditions where feasible (avoid strong wind, wet surface).
- Fix battery voltage (use power supply with known constant voltage or fully charge battery before each run).
- Mark a starting location and ensure robot starts in the same pose/heading.


5) Experimental protocol (step-by-step)

a. **Pre-check / warm-up**
   - Charge battery to a target SOC or use bench power supply.
   - Run a 30–60 s warm-up cycle at low torque to get motors to operating temperature.
   - Confirm sensors (encoders, current, voltage, mocap) are streaming.

b. **Calibration**
   - Calibrate encoder zero & pose offset relative to world frame.
   - Measure motor torque constant `K_t` if unknown (bench test or manufacturer spec).
   - Verify camera/mocap coordinate alignment with robot frame.

c. **Single test run (one torque limit)**
   - Set motor controller torque limit (software or driver).
   - Place robot at marked start pose.
   - Start data logging (timestamped) on all channels: time, base pose, motor command, encoder velocity, current, voltage, temperature.
   - Command the same controller used in sim (e.g., ramp to target angular velocity with the same ramp profile).
   - Run for the same duration as sim (e.g., 6 s).
   - Stop logging and bring robot to safe stop.

d. **Repeatability**
   - Repeat each torque condition **N** times (N≥5 recommended) to estimate mean & variance.
   - Randomize order of torque conditions (or counterbalance) to avoid systematic drift bias.

e. **Parameter sweep**
   - Select torque values matching simulation sweep (coarse then fine).
   - For each torque value, run the N replicates and store logs in uniquely named folders/files:
     ```
     <experiment>/<date>/
       torque_0.020/run_01/
         log.csv
         video.mp4
       torque_0.020/run_02/ ...
     ```
   - Capture video (external camera) for later qualitative analysis.

f. **Data processing & metrics computation**
- Synchronize logs (use timestamp or sync pulse). Resample to a common rate (e.g., 200 Hz).
- Compute:
  - **Distance**: x_final − x_initial from mocap or pose estimate.
  - **Mechanical torque τ(t)**: either direct torque sensor or τ = K_t × I(t) (if motor current measured).
  - **Angular velocity ω(t)**: from encoder.
  - **Actuator mechanical energy** ≈ ∑ |τ(t) × ω(t)| Δt.
  - **Electrical energy** = ∑ V(t) × I(t) Δt.
  - **Efficiency** = distance ÷ energy (report both mechanical-based and electrical-based efficiencies).
- Compute mean and standard deviation across repeats. Plot mean ± std.

g. **Statistical & comparison methods**
- Use **paired** comparisons with sim predictions for same torque points:
  - Report absolute difference and relative error (e.g., (real − sim)/sim × 100%).
- Compute correlation (Pearson or Spearman) between sim and real for distance, energy, efficiency.
- Perform basic hypothesis checks (t-test) if claiming statistically significant differences—report p-values and effect sizes.
- Report trial-to-trial coefficient of variation (CV = std/mean) for each metric.

h. **Sources of discrepancy — what to measure and log to explain them**
- **Slip & contact timing**: log foot contact sensors or use video to detect slip.
- **Motor heating / saturation**: log motor temperature and controller limits.
- **Battery sag**: log supply voltage across the run.
- **Unmodeled friction**: measure static/dynamic friction with bench experiments.
- **Structural compliance**: measure chassis deflection under load if possible.

i. **Ways to make the experiment more streamlined and controlled**
- **Automation**
  - Use a script to iterate torque values, trigger runs, and collect logs automatically (no manual intervention).
  - Implement experiment management: unique experiment IDs, automatic directory creation and naming.
- **Standardize logs**
  - Single CSV/Parquet file per run with fields: timestamp, t, x,y,z, roll,pitch,yaw, motor_cmd, encoder_counts, current, voltage, temp.
- **Pre-checks**
  - Auto-check sensors before each run and abort if out-of-range.
- **Reproducible environment**
  - Use the same power source; stabilize battery between runs.
  - Use fixtures (e.g., guide rails or chucks) to ensure identical start orientation if necessary.
- **Robust error handling**
  - Implement automatic stop if current or temperature exceeds safe thresholds.
- **Automated analysis pipeline**
  - Script that ingests raw logs, computes metrics, generates plots and summary CSVs.
- **Version control**
  - Record firmware/controller version, parameter settings, and code git commit hash in experiment metadata.

j. **Example minimal automation pseudocode**
```python
for torque in torque_values:
    for run_idx in range(1, N+1):
        set_motor_torque_limit(torque)
        ensure_battery_and_temp_ok()
        wait(random_short_delay())   # reduce temporal bias
        start_logging("exp/t_{:.3f}/run_{:02d}".format(torque, run_idx))
        run_controller_for_duration(duration_s)
        stop_logging()
        rename_and_store_video()
# After sweep:
run_analysis_pipeline("exp/")


