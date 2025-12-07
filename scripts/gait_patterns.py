#!/usr/bin/env python3
import numpy as np

JOINT_NAMES = [
    "lf1", "lf2",   
    "rf1", "rf2",   
    "lw1", "lw2",   
    "rw1", "rw2",   
]


def trot_pattern(duration: float = 4.0, dt: float = 0.02):

    t = np.arange(0.0, duration, dt)
    n = len(t)

    freq = 0.5           
    omega = 2.0 * np.pi * freq
    hip_amp = 0.4        
    knee_amp = 0.7      
    knee_offset = 0.9    

    traj = {name: np.zeros(n) for name in JOINT_NAMES}

    def hip_wave(phase: float):
        return hip_amp * np.sin(omega * t + phase)

    def knee_wave(hip_angle):
        return knee_offset - knee_amp * np.sin(omega * t)  

    phase_main = 0.0
    phase_opposite = np.pi  

    hip_lf = hip_wave(phase_main)
    hip_rw = hip_wave(phase_main)
    hip_rf = hip_wave(phase_opposite)
    hip_lw = hip_wave(phase_opposite)

    traj["lf1"] = hip_lf
    traj["rw1"] = hip_rw
    traj["rf1"] = hip_rf
    traj["lw1"] = hip_lw

    traj["lf2"] = knee_offset - knee_amp * np.sin(omega * t + phase_main)
    traj["rw2"] = knee_offset - knee_amp * np.sin(omega * t + phase_main)

    traj["rf2"] = knee_offset - knee_amp * np.sin(omega * t + phase_opposite)
    traj["lw2"] = knee_offset - knee_amp * np.sin(omega * t + phase_opposite)
    return t, traj


def pace_pattern(duration: float = 4.0, dt: float = 0.02):
 
    t = np.arange(0.0, duration, dt)
    n = len(t)
    freq = 0.5
    omega = 2.0 * np.pi * freq
    hip_amp = 0.4
    knee_amp = 0.7
    knee_offset = 0.9

    traj = {name: np.zeros(n) for name in JOINT_NAMES}

    phase_left = 0.0
    phase_right = np.pi

    traj["lf1"] = hip_amp * np.sin(omega * t + phase_left)
    traj["lw1"] = hip_amp * np.sin(omega * t + phase_left)
    traj["lf2"] = knee_offset - knee_amp * np.sin(omega * t + phase_left)
    traj["lw2"] = knee_offset - knee_amp * np.sin(omega * t + phase_left)

    traj["rf1"] = hip_amp * np.sin(omega * t + phase_right)
    traj["rw1"] = hip_amp * np.sin(omega * t + phase_right)
    traj["rf2"] = knee_offset - knee_amp * np.sin(omega * t + phase_right)
    traj["rw2"] = knee_offset - knee_amp * np.sin(omega * t + phase_right)
    return t, traj


def bound_pattern(duration: float = 4.0, dt: float = 0.02):

    t = np.arange(0.0, duration, dt)
    n = len(t)
    freq = 0.5
    omega = 2.0 * np.pi * freq
    hip_amp = 0.4
    knee_amp = 0.7
    knee_offset = 0.9

    traj = {name: np.zeros(n) for name in JOINT_NAMES}

    phase_front = 0.0
    phase_rear = np.pi

    traj["lf1"] = hip_amp * np.sin(omega * t + phase_front)
    traj["rf1"] = hip_amp * np.sin(omega * t + phase_front)
    traj["lf2"] = knee_offset - knee_amp * np.sin(omega * t + phase_front)
    traj["rf2"] = knee_offset - knee_amp * np.sin(omega * t + phase_front)

    traj["lw1"] = hip_amp * np.sin(omega * t + phase_rear)
    traj["rw1"] = hip_amp * np.sin(omega * t + phase_rear)
    traj["lw2"] = knee_offset - knee_amp * np.sin(omega * t + phase_rear)
    traj["rw2"] = knee_offset - knee_amp * np.sin(omega * t + phase_rear)

    return t, traj


def get_gait(gait_type: str, duration: float = 4.0, dt: float = 0.02):
    gait_type = gait_type.lower()
    if gait_type == "trot":
        return trot_pattern(duration, dt)
    if gait_type == "pace":
        return pace_pattern(duration, dt)
    if gait_type == "bound":
        return bound_pattern(duration, dt)
    return trot_pattern(duration, dt)
