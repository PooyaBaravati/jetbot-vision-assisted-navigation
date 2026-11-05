#!/usr/bin/env python2
from __future__ import print_function
import os
import subprocess
import xml.etree.ElementTree as ET

def package(name):
    return subprocess.check_output(['rospack', 'find', name]).decode('utf-8').strip()

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
pro = package('jetbot_pro')
nav_path = os.path.join(pro, 'launch', 'nav_camera_costmap.launch')
mb_path = os.path.join(pro, 'launch', 'move_base_camera_costmap.launch')
local_path = os.path.join(pro, 'config', 'diff', 'local_costmap_camera_params.yaml')
import yaml
with open(local_path) as f:
    settings = yaml.safe_load(f)
plugins = settings['local_costmap']['plugins']
if len([p for p in plugins if p['name'] == 'camera_obstacle_layer']) != 1:
    raise SystemExit('Expected exactly one camera_obstacle_layer; no files written.')
plugins = [dict(p) for p in plugins]
for p in plugins:
    if p['name'] == 'camera_obstacle_layer':
        p['type'] = 'cl_camera_plugins/ExpiringCameraLayer'
cam_index = next(i for i,p in enumerate(plugins) if p['name']=='camera_obstacle_layer')
if not any(p['type']=='costmap_2d::InflationLayer' for p in plugins[cam_index+1:]):
    raise SystemExit('Camera layer must precede inflation; no files written.')
global_plugins = [
    {'name':'static_layer','type':'costmap_2d::StaticLayer'},
    {'name':'obstacle_layer','type':'costmap_2d::ObstacleLayer'},
    {'name':'camera_global_layer','type':'cl_camera_plugins/ExpiringCameraLayer'},
    {'name':'inflation_layer','type':'costmap_2d::InflationLayer'}]
mb = ET.parse(mb_path)
nodes = mb.getroot().findall('node')
if len(nodes)!=1 or nodes[0].get('pkg')!='move_base':
    raise SystemExit('Unexpected move_base launch structure; no files written.')
node=nodes[0]
if node.findall('remap'):
    raise SystemExit('Unexpected remapping. This package requires direct move_base /cmd_vel output.')
ET.SubElement(node,'rosparam',{'param':'local_costmap/plugins'}).text=yaml.safe_dump(plugins, default_flow_style=False)
ET.SubElement(node,'rosparam',{'param':'global_costmap/plugins'}).text=yaml.safe_dump(global_plugins, default_flow_style=False)
for name,value in [
    ('local_costmap/camera_obstacle_layer/obstacle_ttl','5.0'),
    ('local_costmap/camera_obstacle_layer/camera_timeout','1.0'),
    ('local_costmap/camera_obstacle_layer/scan_timeout','0.75'),
    ('local_costmap/camera_obstacle_layer/transition_only','false'),
    ('local_costmap/camera_obstacle_layer/mark_radius','0.03'),
    ('local_costmap/camera_obstacle_layer/preserve_on_reset','false'),
    ('global_costmap/camera_global_layer/obstacle_ttl','8.0'),
    ('global_costmap/camera_global_layer/camera_timeout','1.0'),
    ('global_costmap/camera_global_layer/scan_timeout','0.75'),
    ('global_costmap/camera_global_layer/transition_only','true'),
    ('global_costmap/camera_global_layer/mark_radius','0.05'),
    ('global_costmap/camera_global_layer/preserve_on_reset','true'),
    ('camera_turn/free_frames','5'),('camera_turn/turn_speed','0.6'),
    ('camera_turn/camera_timeout','1.0'),('camera_turn/costmap_timeout','0.75'),
    ('camera_turn/max_duration','30.0'),('camera_turn/turn_margin','0.08'),
    ('camera_turn/free_handoff_delay','2.5'),
    ('camera_turn_retry/free_frames','5'),('camera_turn_retry/turn_speed','0.6'),
    ('camera_turn_retry/camera_timeout','1.0'),('camera_turn_retry/costmap_timeout','0.75'),
    ('camera_turn_retry/max_duration','30.0'),('camera_turn_retry/turn_margin','0.08'),
    ('camera_turn_retry/free_handoff_delay','2.5'),
    ('TebLocalPlannerROS/min_obstacle_dist','0.10'),
    ('TebLocalPlannerROS/max_vel_x','0.40'),
    ('TebLocalPlannerROS/max_vel_x_backwards','0.20')]:
    ET.SubElement(node,'param',{'name':name,'value':value})
ET.SubElement(node,'rosparam',{'param':'recovery_behaviors'}).text=(
    '\n- {name: camera_turn, type: "cl_camera_plugins/CameraTurnRecovery"}'
    '\n- {name: conservative_reset, type: "clear_costmap_recovery/ClearCostmapRecovery"}'
    '\n- {name: camera_turn_retry, type: "cl_camera_plugins/CameraTurnRecovery"}\n')
nav=ET.parse(nav_path)
replaced=0
for item in nav.getroot().iter('include'):
    if item.get('file','').endswith('/launch/move_base_camera_costmap.launch'):
        item.set('file','$(find cl_camera_plugins)/launch/move_base_cl_v2.launch'); replaced+=1
if replaced!=1:
    raise SystemExit('Expected one camera move_base include; no files written.')
launch=os.path.join(root,'launch')
if not os.path.isdir(launch): os.makedirs(launch)
outputs=[(mb,os.path.join(launch,'move_base_cl_v2.launch')),
         (nav,os.path.join(launch,'nav_cl_v2.launch'))]
for tree,path in outputs:
    if os.path.exists(path): raise SystemExit('Output already exists: '+path+'; no files written.')
for tree,path in outputs:
    tree.write(path,encoding='UTF-8',xml_declaration=True)
    ET.parse(path)
    print('Created: '+path)
print('Original jetbot_pro files unchanged. Current CL planner settings inherited.')
