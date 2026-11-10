#!/usr/bin/env python3
"""Generate burt.urdf: simplified quadruped model (no arm) from Fusion measurements.

All source coordinates below are millimetres in the Fusion "burt v2" world frame
(x forward, y left, z up), taken from the front-left leg and the body.
Edit the numbers in the CONFIG section and re-run to regenerate the URDF.
"""
import sys
import numpy as np

# ----------------------------------------------------------------- CONFIG
# Measured masses (grams)
SERVO_G = 62.0                 # each of the 12 leg servos
BATTERY_G = 336.0
TOP_G = 230.0                  # mounting plate + Pi + Pi battery + hardware
ROBOT_NO_TOP_NO_BATT_G = 1700.0  # body + 4 legs incl. 12 servos, no battery/plate/Pi/arm

# Printed volumes from Fusion (cm^3)
VOL = dict(hull=138.59 + 83.23, servo_holder=29.15, servo_clamp=6.66,
           bearing_connector=3.44, crank=5.63, upper_leg=12.24, rod=1.79,
           lower_leg=16.56, tire_connector=7.36)

# Boxes as (min xyz, max xyz) in Fusion world mm
HULL = ([-245.93, -15.78, 40.36], [48.15, 106.03, 87.38])
BATTERY = ([-199.92, 33.13, 42.87], [-69.92, 57.13, 87.37])   # as placed in Fusion
TOP = ([-178.17, -14.07, 79.37], [-62.92, 104.33, 105.0])     # plate + Pi lump

# Front-left leg parts (Fusion world mm)
ROLL_SERVO = ([0.3, 60.0, 54.2], [47.6, 100.2, 74.4])          # bolted to the body
HIP_PARTS = {  # name: (box, mass source)
    'pitch_servo': (([-53.9, 65.3, 54.2], [-13.7, 111.7, 74.4]), 'servo'),
    'knee_servo': (([-53.7, 132.6, 54.1], [-13.5, 179.9, 74.3]), 'servo'),
    'servo_holder': (([-32.7, 65.0, 49.3], [-1.7, 179.2, 74.4]), 'servo_holder'),
    'servo_clamp': (([-32.7, 94.5, 70.4], [-1.7, 149.8, 78.4]), 'servo_clamp'),
    'bearing_connector': (([-65.9, 81.2, 51.2], [-36.9, 97.9, 77.3]), 'bearing_connector'),
    'crank': (([-61.6, 125.1, 52.2], [-31.6, 131.1, 114.2]), 'crank'),
}
UPPER_PARTS = {
    'upper_leg': (([-152.3, 116.7, 51.3], [-30.8, 123.7, 77.3]), 'upper_leg'),
    'rod': (([-147.8, 119.7, 100.2], [-39.6, 125.6, 108.3]), 'rod'),
}
LOWER_PARTS = {
    'lower_leg': (([-153.8, 122.7, -35.7], [-133.8, 131.7, 114.3]), 'lower_leg'),
}
FOOT_PARTS = {
    'tire_connector': (([-168.8, 116.7, -40.7], [-129.5, 130.7, -9.7]), 'tire_connector'),
}

# Pivot points of the front-left leg (Fusion world mm)
BASE_ORIGIN = np.array([-98.89, 45.127, 64.294])   # body centre, at hip-axis height
HIP = np.array([-43.817, 90.126, 64.294])          # on the roll axis (x), at the pitch-axis x
UPPER = np.array([-43.817, 120.226, 64.294])       # hip pitch axis (y)
KNEE = np.array([-143.817, 127.2, 64.294])         # knee axis (y), 100 mm from hip pitch
FOOT = np.array([-143.817, 123.7, -15.706])        # centre of the foot arc, 80 mm below the knee
FOOT_RADIUS_MM = 25.0                              # printed foot radius, without the tyre

MIRROR_Y = 45.127          # left/right symmetry plane
REAR_SHIFT_X = -180.0      # rear legs are the front legs moved back by this

# Joint limits (rad, URDF sign convention, zero = the Fusion pose).
# hip_pitch and knee are the mechanical stops in the Fusion design, found by rotating the
# parts about the joint axis and checking for interference (first contact, rounded inward):
#   hip_pitch  -81 deg: upper leg meets the floor of the servo holder, 9 deg before it hangs
#                       straight down (-90 deg);  +81 deg: upper leg meets the servo clamp.
#   knee       -67 / +64 deg: the knee rod of the parallelogram comes down onto the upper leg.
# Not expressible as a URDF limit, and never reached by the gait: the knee crank meets the
# servo clamp when hip_pitch + knee (= the knee servo angle) goes above +28 deg.
# hip_roll is still a placeholder: the hip link itself clears the body over +-90 deg, so the
# real limit is the leg touching the body, which depends on the leg pose, and the cables.
LIMITS = dict(hip_roll=(-0.6, 0.6), hip_pitch=(-1.41, 1.41), knee=(-1.17, 1.12))
# Placeholders until the servo recordings are in
EFFORT, VELOCITY = 1.9, 4.7     # N*m, rad/s
FOOT_FRICTION = 1.0
# ------------------------------------------------------------- END CONFIG

PLASTIC_G = ROBOT_NO_TOP_NO_BATT_G - 12 * SERVO_G
PRINT_CM3 = VOL['hull'] + 4 * sum(v for k, v in VOL.items() if k != 'hull')
DENSITY = PLASTIC_G / PRINT_CM3   # g/cm^3, includes screws/bearings/inserts/brackets

LEGS = {'fl': (0.0, +1), 'fr': (0.0, -1), 'rl': (REAR_SHIFT_X, +1), 'rr': (REAR_SHIFT_X, -1)}


def part_mass_g(src):
    return SERVO_G if src == 'servo' else VOL[src] * DENSITY


def xf_point(p, sx, m):
    p = np.asarray(p, float)
    return np.array([p[0] + sx, p[1] if m > 0 else 2 * MIRROR_Y - p[1], p[2]])


def xf_box(box, sx, m):
    a, b = xf_point(box[0], sx, m), xf_point(box[1], sx, m)
    return np.minimum(a, b), np.maximum(a, b)


def box_inertia(mass, size):
    sx, sy, sz = size
    return mass / 12.0 * np.diag([sy**2 + sz**2, sx**2 + sz**2, sx**2 + sy**2])


def compose(items, frame_origin_mm):
    """items: list of (mass_g, (min,max) world mm). Returns mass kg, com m (link frame), inertia about com."""
    ms, cs, Is = [], [], []
    for g, (lo, hi) in items:
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        m = g / 1000.0
        c = ((lo + hi) / 2 - frame_origin_mm) / 1000.0
        ms.append(m); cs.append(c); Is.append(box_inertia(m, (hi - lo) / 1000.0))
    M = sum(ms)
    com = sum(m * c for m, c in zip(ms, cs)) / M
    I = np.zeros((3, 3))
    for m, c, Ic in zip(ms, cs, Is):
        d = c - com
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, com, I


def f(v):
    return ' '.join(f'{x:.6g}' for x in np.atleast_1d(v))


def inertial_xml(M, com, I):
    return (f'    <inertial>\n      <origin xyz="{f(com)}" rpy="0 0 0"/>\n      <mass value="{M:.6g}"/>\n'
            f'      <inertia ixx="{I[0,0]:.6e}" ixy="{I[0,1]:.6e}" ixz="{I[0,2]:.6e}" '
            f'iyy="{I[1,1]:.6e}" iyz="{I[1,2]:.6e}" izz="{I[2,2]:.6e}"/>\n    </inertial>\n')


def box_geom_xml(tag, box, frame_origin_mm, material=None):
    lo, hi = np.asarray(box[0], float), np.asarray(box[1], float)
    c = ((lo + hi) / 2 - frame_origin_mm) / 1000.0
    s = (hi - lo) / 1000.0
    mat = f'\n      <material name="{material}"/>' if (material and tag == 'visual') else ''
    return (f'    <{tag}>\n      <origin xyz="{f(c)}" rpy="0 0 0"/>\n'
            f'      <geometry><box size="{f(s)}"/></geometry>{mat}\n    </{tag}>\n')


def link_xml(name, inertial, geoms):
    return f'  <link name="{name}">\n{inertial}{"".join(geoms)}  </link>\n'


def joint_xml(name, jtype, parent, child, origin_m, axis=None, limit=None):
    s = f'  <joint name="{name}" type="{jtype}">\n    <parent link="{parent}"/>\n    <child link="{child}"/>\n'
    s += f'    <origin xyz="{f(origin_m)}" rpy="0 0 0"/>\n'
    if axis is not None:
        s += f'    <axis xyz="{f(axis)}"/>\n'
        s += (f'    <limit lower="{limit[0]}" upper="{limit[1]}" effort="{EFFORT}" velocity="{VELOCITY}"/>\n'
              f'    <dynamics damping="0.02" friction="0.0"/>\n')
    return s + '  </joint>\n'


def build():
    out = []
    out.append('<?xml version="1.0"?>\n')
    out.append(f'''<!--
  burt: simplified quadruped model, generated by make_burt_urdf.py from the Fusion design "burt v2".
  Frame: x forward, y left, z up. base_link origin is the body centre at hip-axis height.
  Zero pose = the pose modelled in Fusion: upper legs horizontal pointing backward, lower legs straight down.
  Joints per leg (fl, fr, rl, rr): <leg>_hip_roll (about +x), <leg>_hip_pitch (about +y), <leg>_knee (about +y).
  The same axis directions are used on all four legs.
  Knee linkage: the real knee servo drives the lower leg through a parallelogram, so
      knee joint angle = knee servo angle - hip pitch servo angle.
  The linkage itself is not modelled; its mass is lumped into the hip and upper-leg links.
  Masses: total {(ROBOT_NO_TOP_NO_BATT_G + BATTERY_G + TOP_G)/1000:.3f} kg. Servos {SERVO_G:.0f} g each, battery {BATTERY_G:.0f} g, plate+Pi {TOP_G:.0f} g.
  The remaining {PLASTIC_G:.0f} g (printed parts plus screws, bearings, inserts, brackets) is spread over the
  printed parts by volume ({DENSITY:.2f} g/cm3 effective).
  Joint limits: hip_pitch and knee are the mechanical stops of the Fusion design (interference check).
  PLACEHOLDERS to replace with measurements: hip_roll limit, effort, velocity, damping, foot radius with tyre.
-->
''')
    out.append('<robot name="burt">\n')
    out.append('  <material name="body"><color rgba="0.25 0.25 0.28 1"/></material>\n')
    out.append('  <material name="leg"><color rgba="0.85 0.45 0.1 1"/></material>\n')
    out.append('  <material name="foot"><color rgba="0.1 0.1 0.1 1"/></material>\n\n')

    # ---- base
    base_items = [(VOL['hull'] * DENSITY, HULL), (BATTERY_G, BATTERY), (TOP_G, TOP)]
    for sx, m in LEGS.values():
        base_items.append((SERVO_G, xf_box(ROLL_SERVO, sx, m)))
    M, com, I = compose(base_items, BASE_ORIGIN)
    geoms = [box_geom_xml('visual', HULL, BASE_ORIGIN, 'body'), box_geom_xml('collision', HULL, BASE_ORIGIN),
             box_geom_xml('visual', TOP, BASE_ORIGIN, 'body'), box_geom_xml('collision', TOP, BASE_ORIGIN)]
    out.append(link_xml('base_link', inertial_xml(M, com, I), geoms))
    masses = {'base_link': M}

    for leg, (sx, m) in LEGS.items():
        hip_o, up_o, knee_o, foot_o = (xf_point(p, sx, m) for p in (HIP, UPPER, KNEE, FOOT))

        def items(parts):
            return [(part_mass_g(src), xf_box(box, sx, m)) for box, src in parts.values()]

        # hip link
        M, com, I = compose(items(HIP_PARTS), hip_o)
        vis = xf_box(([-54.0, 65.0, 49.3], [-1.7, 180.0, 78.4]), sx, m)
        col = xf_box(([-54.0, 108.0, 49.3], [-1.7, 180.0, 78.4]), sx, m)   # part outside the body only
        out.append(link_xml(f'{leg}_hip', inertial_xml(M, com, I),
                            [box_geom_xml('visual', vis, hip_o, 'leg'), box_geom_xml('collision', col, hip_o)]))
        masses[f'{leg}_hip'] = M
        out.append(joint_xml(f'{leg}_hip_roll', 'revolute', 'base_link', f'{leg}_hip',
                             (hip_o - BASE_ORIGIN) / 1000.0, (1, 0, 0), LIMITS['hip_roll']))
        # upper leg
        M, com, I = compose(items(UPPER_PARTS), up_o)
        ub = xf_box(UPPER_PARTS['upper_leg'][0], sx, m)
        out.append(link_xml(f'{leg}_upper_leg', inertial_xml(M, com, I),
                            [box_geom_xml('visual', ub, up_o, 'leg'), box_geom_xml('collision', ub, up_o)]))
        masses[f'{leg}_upper_leg'] = M
        out.append(joint_xml(f'{leg}_hip_pitch', 'revolute', f'{leg}_hip', f'{leg}_upper_leg',
                             (up_o - hip_o) / 1000.0, (0, 1, 0), LIMITS['hip_pitch']))
        # lower leg
        M, com, I = compose(items(LOWER_PARTS), knee_o)
        lb = xf_box(([-153.8, 122.7, -15.7], [-133.8, 131.7, 114.3]), sx, m)
        out.append(link_xml(f'{leg}_lower_leg', inertial_xml(M, com, I),
                            [box_geom_xml('visual', lb, knee_o, 'leg'), box_geom_xml('collision', lb, knee_o)]))
        masses[f'{leg}_lower_leg'] = M
        out.append(joint_xml(f'{leg}_knee', 'revolute', f'{leg}_upper_leg', f'{leg}_lower_leg',
                             (knee_o - up_o) / 1000.0, (0, 1, 0), LIMITS['knee']))
        # foot (fixed), sphere contact
        M, com, I = compose(items(FOOT_PARTS), foot_o)
        r = FOOT_RADIUS_MM / 1000.0
        sphere = (f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n      <geometry><sphere radius="{r:.6g}"/></geometry>\n')
        geoms = [f'    <visual>\n{sphere}      <material name="foot"/>\n    </visual>\n',
                 f'    <collision>\n{sphere}    </collision>\n']
        out.append(link_xml(f'{leg}_foot', inertial_xml(M, com, I), geoms))
        masses[f'{leg}_foot'] = M
        out.append(joint_xml(f'{leg}_foot_fixed', 'fixed', f'{leg}_lower_leg', f'{leg}_foot',
                             (foot_o - knee_o) / 1000.0))
        out.append('\n')
    out.append('</robot>\n')
    return ''.join(out), masses


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else 'burt.urdf'
    xml, masses = build()
    with open(path, 'w') as fh:
        fh.write(xml)
    print(f'wrote {path}')
    print(f'effective print density {DENSITY:.3f} g/cm3 over {PRINT_CM3:.1f} cm3')
    for k, v in masses.items():
        print(f'  {k:16s} {v*1000:8.1f} g')
    print(f'  total            {sum(masses.values())*1000:8.1f} g')
