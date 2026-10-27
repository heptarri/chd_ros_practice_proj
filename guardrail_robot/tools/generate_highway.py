#!/usr/bin/env python3
"""Generate worlds/highway.sdf for guardrail_robot.

The road is a circular left-hand arc of constant radius R.  Everything
(asphalt chords, edge lines, centre dashes, guardrail, posts and the damaged
section) is written as explicit SDF geometry, so Gazebo Fortress can load the
world without any runtime mesh generation.

Change the CONFIG block and re-run:

    python3 tools/generate_highway.py
"""

from pathlib import Path
import math


# --------------------------------------------------------------------- config
R = 120.0            # curve radius [m] -- keeps the bend unchanged
S_START = -10.0      # asphalt starts this far behind the spawn point
S_END = 170.0        # asphalt ends here -> 180 m of road

DAMAGE_START = 30.0  # guardrail missing from here ...
DAMAGE_END = 38.0    # ... to here (bent rail in the second half)
BENT_START = 34.0

LANE_EDGE = 3.65     # solid edge lines at +/- this offset
ASPHALT_D = -1.25    # carriageway + shoulder centre offset
RAIL_D = -6.15
POST_D = -6.25

STEP = 2.0           # chord length used for road furniture

MAT_ROAD = ('<material><ambient>0.055 0.058 0.062 1</ambient>'
            '<diffuse>0.16 0.17 0.18 1</diffuse></material>')
MAT_LINE = ('<material><ambient>0.8 0.8 0.76 1</ambient>'
            '<diffuse>0.95 0.95 0.9 1</diffuse></material>')
MAT_RAIL = ('<material><ambient>0.44 0.47 0.5 1</ambient>'
            '<diffuse>0.68 0.72 0.75 1</diffuse></material>')
MAT_POST = ('<material><ambient>0.25 0.27 0.29 1</ambient>'
            '<diffuse>0.42 0.44 0.46 1</diffuse></material>')

FRICTION_ROAD = ('<surface><friction><ode><mu>0.95</mu><mu2>0.95</mu2>'
                 '</ode></friction></surface>')


def pose(s, d=0.0, z=0.0, roll=0.0, pitch=0.0, yaw_off=0.0):
    """Pose on the arc at arc length s, lateral offset d (positive = left)."""
    th = s / R
    x = (R - d) * math.sin(th)
    y = R - (R - d) * math.cos(th)
    return (f'{x:.6f} {y:.6f} {z:.6f} '
            f'{roll:.6f} {pitch:.6f} {th + yaw_off:.6f}')


def box_visual(name, p, size, material):
    return (f'        <visual name="{name}"><pose>{p}</pose>'
            f'<geometry><box><size>{size}</size></box></geometry>'
            f'{material}</visual>\n')


def box_collision(name, p, size, surface=''):
    return (f'        <collision name="{name}"><pose>{p}</pose>'
            f'<geometry><box><size>{size}</size></box></geometry>'
            f'{surface}</collision>\n')


def build_surface():
    out = []

    # Ground plane: big enough for the whole arc.
    out.append(
        '        <visual name="grass">\n'
        '          <pose>0 0 -0.03 0 0 0</pose>\n'
        '          <geometry><plane><normal>0 0 1</normal>'
        '<size>320 260</size></plane></geometry>\n'
        '          <material>\n'
        '            <ambient>0.06 0.12 0.04 1</ambient>\n'
        '            <diffuse>0.12 0.25 0.08 1</diffuse>\n'
        '          </material>\n'
        '        </visual>\n'
        '        <collision name="grass_collision">\n'
        '          <pose>0 0 -0.03 0 0 0</pose>\n'
        '          <geometry><plane><normal>0 0 1</normal>'
        '<size>320 260</size></plane></geometry>\n'
        '          <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2>'
        '</ode></friction></surface>\n'
        '        </collision>\n\n'
    )

    # Paved carriageway (2 x 3.75 m) plus the 2.50 m right-hand hard
    # shoulder, split into short chords along the bend.
    out.append('        <!-- Paved carriageway plus the right-hand hard '
               'shoulder. -->\n')
    n_asphalt = int(round((S_END - S_START) / STEP))
    for i in range(n_asphalt):
        s = S_START + STEP * (i + 0.5)
        z = 0.0 if i % 2 == 0 else 0.0001
        p = pose(s, ASPHALT_D, z)
        out.append(box_visual(f'asphalt_{i}', p, '2.180 10.02 0.04',
                              MAT_ROAD))
        out.append(box_collision(f'asphalt_{i}_collision', p,
                                 '2.180 10.02 0.04', FRICTION_ROAD))

    # Solid edge lines at the lane boundaries (0.15 m wide).
    out.append('\n        <!-- Solid edge lines at the lane boundaries. -->\n')
    i = 0
    s = -8.0
    while s <= S_END - STEP:
        z = 0.025 if i % 2 == 0 else 0.02505
        out.append(box_visual(f'edge_0_{i}', pose(s, LANE_EDGE, z),
                              '2.120 0.15 0.006', MAT_LINE))
        out.append(box_visual(f'edge_1_{i}', pose(s, -LANE_EDGE, z),
                              '2.120 0.15 0.006', MAT_LINE))
        i += 1
        s += STEP

    # Broken centre line: 6 m dash, 9 m gap (15 m cycle).
    out.append('\n        <!-- Broken centre line: 6 m dash, 9 m gap '
               '(15 m cycle). -->\n')
    k = 0
    group_start = -9.0
    while group_start + 5.0 <= S_END - STEP:
        for j in range(3):
            s = group_start + 1.0 + 2.0 * j
            z = 0.025 if j % 2 == 0 else 0.02505
            out.append(box_visual(f'dash_{k}_{j}', pose(s, 0.0, z),
                                  '2.120 0.15 0.006', MAT_LINE))
        k += 1
        group_start += 15.0

    return ''.join(out)


def build_guardrail():
    out = []

    # Continuous W-beam, with the damaged span left out.
    out.append('        <!-- Continuous W-beam, damaged span left out. -->\n')
    idx = 0
    s = -7.0
    while s <= S_END - 1.0:
        if not (DAMAGE_START <= s < DAMAGE_END):
            z = 0.6 if idx % 2 == 0 else 0.6001
            zr = 0.53 if idx % 2 == 0 else 0.5301
            p = pose(s, RAIL_D, z)
            out.append(box_visual(f'rail_{idx}', p, '2.180 0.07 0.31',
                                  MAT_RAIL))
            out.append(box_collision(f'rail_{idx}_collision', p,
                                     '2.180 0.07 0.31',
                                     '<surface><friction><ode><mu>0.7</mu>'
                                     '<mu2>0.7</mu2></ode></friction>'
                                     '</surface>'))
            for ridge in range(2):
                out.append(box_visual(
                    f'rail_{idx}_ridge_{ridge}',
                    pose(s, RAIL_D + 0.045, zr + 0.14 * ridge),
                    '2.180 0.04 0.06', MAT_RAIL))
            idx += 1
        s += STEP

    # Bent W-beam in the damaged span, displaced towards the road.
    out.append('\n        <!-- Bent W-beam, displaced towards the road. -->\n')
    bent_centre = (BENT_START + DAMAGE_END) / 2.0
    p = pose(bent_centre, -5.92, 0.58, roll=0.12, yaw_off=0.10)
    out.append(box_visual('rail_bent', p, '3.9 0.07 0.31', MAT_RAIL))
    out.append(box_collision('rail_bent_collision', p, '3.9 0.07 0.31',
                             '<surface><friction><ode><mu>0.7</mu>'
                             '<mu2>0.7</mu2></ode></friction></surface>'))
    for ridge in range(2):
        out.append(box_visual(
            f'rail_bent_ridge_{ridge}',
            pose(bent_centre, -5.871, 0.51 + 0.14 * ridge,
                 roll=0.12, yaw_off=0.10),
            '3.8 0.04 0.06', MAT_RAIL))

    # Posts every 4 m; s=32 is missing, s=36 is the bent post.
    out.append('\n        <!-- Posts every 4 m; s=32 missing, s=36 bent. '
               '-->\n')
    idx = 0
    s = -8.0
    while s <= S_END:
        if s not in (32.0, 36.0):
            p = pose(s, POST_D, 0.375)
            out.append(box_visual(f'post_{idx}', p, '0.12 0.12 0.75',
                                  MAT_POST))
            out.append(box_collision(f'post_{idx}_collision', p,
                                     '0.12 0.12 0.75'))
            idx += 1
        s += 4.0

    p = pose(bent_centre, -6.05, 0.34, roll=0.18, yaw_off=0.12)
    out.append(box_visual('post_bent', p, '0.12 0.12 0.75', MAT_POST))
    out.append(box_collision('post_bent_collision', p, '0.12 0.12 0.75'))

    p = pose(33.05, -5.64, 0.08, pitch=1.45, yaw_off=0.6)
    out.append(box_visual('post_fallen', p, '0.12 0.12 0.75', MAT_POST))
    out.append(box_collision('post_fallen_collision', p, '0.12 0.12 0.75'))

    return ''.join(out)


def build_work_zone():
    # Only the broken rail is left here; the traffic drums were removed so
    # the right lane stays clear for the line follower.
    p = pose(31.6, -5.6, 0.05, yaw_off=0.9)
    out = [box_visual('broken_rail', p, '1.6 0.30 0.06', MAT_RAIL)]
    out.append(box_collision('broken_rail_collision', p, '1.6 0.30 0.06'))
    return ''.join(out)


def main():
    angle_deg = math.degrees((S_END - S_START) / R)
    length = S_END - S_START

    header = f'''<?xml version="1.0"?>
<!--
  highway.sdf — {length:.0f} m two-lane highway with a left-hand bend.

  Generated by tools/generate_highway.py (edit the CONFIG block there and
  re-run the script instead of editing this file by hand).

  The centreline is a circular arc of radius {R:.0f} m over about
  {angle_deg:.0f} degrees ({length:.0f} m arc length).  Cross-section follows
  a realistic highway:
    * two 3.75 m lanes (7.50 m carriageway)
    * 2.50 m right-hand hard shoulder
    * 0.15 m edge lines, 6 m dash / 9 m gap centre line
    * W-beam guardrail at the outer edge of the shoulder:
      0.31 m beam, top at 0.755 m, posts every 4 m
    * damage zone around 30-38 m: missing / bent rail and a fallen post

  Axes: +x forward, +y left, +z up.
  Arc: centre at (0, {R:.0f}); the spawn point is the origin heading +x, with
  {abs(S_START):.0f} m of asphalt behind it and {S_END:.0f} m ahead.
  Right-hand lane centre at the spawn point: (0, -1.875), heading +x.
  Guardrail normal offset: y=-6.15.
-->
<sdf version="1.8">
  <world name="highway">
'''

    middle = f'''
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.55 0.58 0.62 1</ambient>
      <background>0.60 0.72 0.86 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0.55 0.35</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.45 0.35 -1.0</direction>
    </light>

    <plugin filename="libignition-gazebo-physics-system.so"
        name="ignition::gazebo::systems::Physics"/>

    <plugin filename="libignition-gazebo-user-commands-system.so"
        name="ignition::gazebo::systems::UserCommands"/>

    <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
        name="ignition::gazebo::systems::SceneBroadcaster"/>

    <plugin filename="libignition-gazebo-sensors-system.so"
        name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- ==================== Road surface ==================== -->
    <model name="highway_surface">
      <static>true</static>
      <link name="surface_link">

'''

    world = (
        header + middle + build_surface() +
        '\n      </link>\n    </model>\n'
        '\n    <!-- ==================== Guardrail ==================== -->\n'
        '    <model name="guardrail">\n'
        '      <static>true</static>\n'
        '      <link name="guardrail_link">\n\n' +
        build_guardrail() +
        '\n      </link>\n    </model>\n'
        '\n    <!-- ==================== Damage debris ==================== -->\n'
        '    <model name="work_zone">\n'
        '      <static>true</static>\n'
        '      <link name="work_zone_link">\n\n' +
        build_work_zone() +
        '\n      </link>\n    </model>\n'
        '\n  </world>\n</sdf>\n'
    )

    out = Path(__file__).resolve().parent.parent / 'worlds' / 'highway.sdf'
    out.write_text(world, encoding='utf-8')
    print(f'wrote {out} ({len(world.splitlines())} lines, {length:.0f} m road, '
          f'{angle_deg:.1f} deg, R={R:.0f} m)')


if __name__ == '__main__':
    main()
