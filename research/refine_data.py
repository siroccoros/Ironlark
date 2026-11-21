# Evidence corrections applied after primary project drafting.
import json
la=json.loads((EV/'license-audit.json').read_text())
for rec in la:
 if rec['id'] not in by or 'url' not in rec:continue
 x=by[rec['id']]; src(x['id'],'License file inspected',rec['url'])
 old=x['license']; kind=rec['kind']
 if kind in ['MIT','Apache-2.0']:
  if x['id'] not in ['isaacvio','sym','realsense','oak','prometheus','traj']:
   x['license']=kind+' at root; dependencies separate'
 elif kind=='BSD-like' and ('3-Clause' in rec['excerpt'] or '3. Neither' in rec['excerpt'] or 'Neither the name' in rec['excerpt']):
  if x['id'] not in ['airstack']:
   x['license']='BSD-3-Clause'+('; Gurobi / solver terms separate' if x['id'] in ['faster','mader','rmader','panther','deeppan','dynus','sando'] else ' at root')
 elif kind.startswith('GPL-') and x['license'].startswith('Check'):x['license']=kind.replace('GPL-','GPL-')+'.0 at root'
by['msckf']['license']='Restricted: nonprofit research (Penn agreement)';by['msckf']['status']='Restricted research';by['msckf']['limit']='Current Penn agreement restricts use to nonprofit research and internal academic modifications. Estimator only; older ROS dependencies.'
by['raven']['license']='BSD-3-Clause wrapper; dependencies separate'
by['avoidmpc']['license']='MIT; CasADi/IPOPT dependencies separate'
by['gco']['license']='MIT'
by['pegasus']['license']='BSD-3-Clause extension; simulator separate'
by['nvblox']['license']='Apache-2.0 library; CUDA/toolchain terms separate';by['nvblox']['status']='Research';by['nvblox']['limit']='GPU-specific acceleration. Current LICENSE.md is Apache-2.0 despite ambiguous API metadata; CUDA, ROS integration and assets have separate dependencies.'
by['uxas']['license']='USAF Open Source Agreement 1.0; registration terms'
by['dlio']['license']='MIT';by['rpgctrl']['license']='MIT';by['ual']['license']='MIT'
edges[:]=[z for z in edges if (z['a'],z['b']) not in [('gco','decomp'),('super','fastlivo'),('kumar','jps')]]
for z in edges:
 if (z['a'],z['b'])==('super','fastlio'):
  z['type']='Uses';z['note']='FAST-LIO2 is the state estimator in the published SUPER system.';z['source']=by['super']['sources'][1]['url']
# Correct a link’s label, not the evidence distinction.
by['ntnu']['sources'].append({'label':'Workspace manifests','url':by['ntnu']['repo']+'/tree/'+by['ntnu']['commit']+'/repos'})
for cl in claims:
 if cl['id']=='C38':cl['sources'][-1]['url']=by['ntnu']['repo']+'/tree/'+by['ntnu']['commit']+'/repos'

for z in edges:
 if z['source'].endswith('/README.md'):
  if z['a']=='fastdrone':z['source']=z['source'][:-len('README.md')]+'readme_en.md'
  if z['a'] in ['rmader','deeppan']:z['source']=z['source'][:-len('README.md')]+'Readme.md'
 if (z['a'],z['b'])==('agile','agilicious'):z['source']=pinned('agilicious','README.md')
 if (z['a'],z['b'])==('ntnu','px4'):z['source']='https://arxiv.org/abs/2605.12735'
 if (z['a'],z['b'])==('as2','px4'):z['source']='https://github.com/aerostack2/as2_platform_pixhawk'
 if (z['a'],z['b'])==('gb','voxblox'):z['source']=pinned('ntnu','repos/ws_gbplanner.repos')
 if (z['a'],z['b'])==('px4ctrl','mavros'):z['source']=pinned('fastdrone','src/realflight_modules/px4ctrl/package.xml')
for z in edges:
 if (z['a'],z['b'])==('kumar','msckf'):z['source']=pinned('kumar','external_all.yaml')
 if z['a']=='alt' and z['b'] in ['px4','ardu']:z['source']='https://docs.altnautica.com/drone-agent/overview'
# The upstream controller stack is not MAVROS-based on evidence from the root README.
# Retain only edges whose exact supporting implementation was established.
edges[:]=[z for z in edges if (z['a'],z['b'])!=('kumar','mavros')]
for cl in claims:
 if cl['id']=='C11':
  cl['claim']='Navigation can run without continuous manual piloting, while oversight remains a separate requirement.'
  cl['verdict']='Supported with limits'
  cl['finding']='Grok explicitly included this distinction in its caveat. Autonomous route execution does not establish unattended arbitrary-environment operation; operator responsibilities and permitted operations are separate questions.'
 if cl['id']=='C16':cl['claim']='PX4 Avoidance is a suitable current companion-planner option.'
 if cl['id']=='C17':
  cl['claim']='Both autopilots provide basic collision prevention.';cl['verdict']='Supported with limits'
  cl['finding']='Directionally correct, but the capabilities differ. PX4 collision prevention is distinct from ArduPilot BendyRuler detours in AUTO/GUIDED/RTL; Dijkstra+BendyRuler combines fence routing and sensed-obstacle avoidance.'
 if cl['id']=='C19':
  cl['claim']='OctoMap/Voxblox and VIO/SLAM packages are useful perception building blocks.';cl['verdict']='Supported with limits'
  cl['finding']='Correct at the architectural level. Occupancy/distance maps and pose estimators solve different problems. OpenVINS/VINS/ORB-SLAM3 do not automatically supply a dense collision map; frames, timing and representations must agree.'
 if cl['id']=='C31':
  cl['claim']='Aerostack2-based setups belong among modular end-to-end autonomy frameworks.';cl['verdict']='Supported with limits'
  cl['finding']='The framework is real, and a chosen setup can combine the required stages. This does not establish default general 3D avoidance: the inspected A* plugin uses a 2D occupancy map and needs an appropriate perception/planning configuration.'
