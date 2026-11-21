"""Project-level identity and problem-first annotations for the canvas atlas."""
from pathlib import Path
import copy,json,re

def enrich(original):
 d=copy.deepcopy(original);ps=d['projects'];es=d['edges'];by={p['id']:p for p in ps}
 # Keep the internal package as an explanation on its parent, not a project node.
 internal=by.pop('px4ctrl');ps[:]=[p for p in ps if p['id']!='px4ctrl']
 by['fastdrone']['internalNotes']=[{'name':'px4ctrl','description':'An internal tracking/control package in this repository. It consumes estimated state and desired motion and sends commands through MAVROS. Other projects may copy this package without using the whole Fast-Drone system.','url':internal['repo']}]
 es[:]=[e for e in es if 'px4ctrl' not in [e['a'],e['b']]]
 for p in ps:p['identity']='project'
 review=json.loads((Path(__file__).resolve().parents[0]/'evidence/canvas-source-review.json').read_text());review={r['repo']:r for r in review}
 def add(id,name,repo,layer,role,problem,summary,needs,limit,platform='ROS 1',license='Module license not separately audited'):
  p=dict(id=id,name=name,repo='https://github.com/'+repo,layer=layer,role=role,problem=problem,summary=summary,needs=needs,limit=limit,platform=platform,license=license,status='Research component',evidence='Repository documentation + parent assembly manifest',identity='project',sources=[dict(label='Project repository',url='https://github.com/'+repo)])
  if repo in review and not review[repo].get('unavailable'):p['sources'].append(dict(label='Inspected project README',url=review[repo]['url']))
  ps.append(p);by[id]=p;return p
 def edge(a,b,typ,note,source=None):
  e=dict(a=a,b=b,type=typ,note=note,source=source or by[a]['repo']+'/blob/HEAD/README.md');es.append(e);return e
 add('cerflight','CERLAB autonomous_flight','Zhefan-Xu/autonomous_flight','Planning','Mission orchestration','Turn perception and motion modules into navigation, exploration and inspection behaviors.','Mission-level integration of the lab’s perception, planning and control modules, released as its own repository.','CERLAB modules and calibrated sensor/state inputs.','The px4 branch targets real flight/PX4 simulation; the simulation branch targets the custom simulator.')
 add('cerremote','CERLAB remote_control','Zhefan-Xu/remote_control','Control','Visualization and commands','Inspect a running mission and send high-level commands.','RViz configurations and command scripts distributed independently as part of the CERLAB assembly.','A running CERLAB stack and ROS environment.','Operator tooling, not the autonomy algorithm.')
 add('certime','ROTA / time_optimizer','Zhefan-Xu/time_optimizer','Planning','Trajectory timing','Adjust how quickly a trajectory is executed under uncertainty.','Optimal time allocation post-processes a trajectory to improve timing and account for uncertainty.','Trajectory, mapping and controller interfaces from the CERLAB ecosystem.','Optimizes time along a trajectory; it does not independently supply perception or an aircraft.')
 add('certrack','CERLAB tracking_controller','Zhefan-Xu/tracking_controller','Control','Trajectory tracking','Make a PX4-based robot follow the planned motion.','PID trajectory-tracking controller distributed as a standalone CERLAB repository.','State estimate, desired trajectory and MAVROS.','Vehicle dynamics, tracking error and tuning remain part of deployment.')
 add('cersim','CERLAB uav_simulator','Zhefan-Xu/uav_simulator','Simulation','Quadrotor simulation','Test navigation around static and moving obstacles before deploying.','Lightweight Gazebo/ROS simulation with an optional PX4 simulation wrapper.','ROS/Gazebo; MAVROS-related packages; optional PX4 setup.','Two simulator paths exist. Their controller and sensing assumptions should not be mixed silently.')
 add('krcontrol','Kumar MAV control','KumarRobotics/kr_mav_control','Control','Quadrotor control bundle','Connect trajectory tracking, vehicle interfaces and flight management.','Reusable control stack containing controllers, trackers, robot interfaces, messages and a small dynamics simulator.','Compatible state/trajectory messages and robot interface.','The current default README describes ROS 2 Humble. The autonomy assembly pins master; follow the assembly branch rather than assuming all default branches match.','ROS 1 / ROS 2 branches')
 add('mrslsim','MRSL quadrotor simulator','KumarRobotics/mrsl_quadrotor','Simulation','Vehicle and sensor simulation','Reproduce a quadrotor platform and its sensors in Gazebo.','Vehicle models, sensor setups, environments and simulator utilities used in Kumar’s flight assembly.','Compatible legacy ROS/Gazebo stack and quadrotor control.','A simulator dependency, not an independent real-flight navigator.')
 add('mrscore','MRS UAV core','ctu-mrs/mrs_uav_core','Control','Flight framework bundle','Assemble the estimation, control and flight-management core.','A separate aggregation repository organizes the core MRS flight packages and their plugin interfaces.','MRS installation and matching ROS branch.','The bundle contains multiple independent repositories. It does not itself establish obstacle-avoiding mission behavior.','ROS 2; legacy ROS 1')
 add('quarter','quarterKalibr','D2SLAM-Fusion/tools-quarterKalibr','Perception','Four-camera calibration','Calibrate an overlapping four-fisheye camera rig in smaller steps.','OmniNxt’s calibration workflow builds on Kalibr and TartanCalib for multi-fisheye camera calibration.','Recorded camera/IMU calibration data and the specified tools.','Calibration tooling, not an online state estimator.','ROS / offline')
 add('vdb','VDB Mapping','castacks/vdb_mapping','Mapping','Sparse 3D mapping','Maintain a sparse volumetric environment map.','The VDB mapping core included in AirStack is an OpenVDB-based mapping library.','OpenVDB and posed sensor observations; optional ROS wrapper.','AirStack points to its castacks fork; upstream development is separately linked in the README.','C++ / ROS wrapper')
 add('vdbros','VDB Mapping ROS 2','castacks/vdb_mapping_ros2','Mapping','Mapping interface','Expose VDB mapping to a ROS 2 autonomy stack.','Separate ROS 2 wrapper around VDB Mapping, included as an AirStack submodule.','VDB Mapping/OpenVDB and ROS 2.','A wrapper dependency, not an independent planner.','ROS 2')
 for b in ['cerflight','cerremote','certime','certrack','cersim']:edge('cerlab',b,'Uses','Separate repository listed in the parent assembly’s submodules.',by['cerlab']['repo']+'/blob/'+by['cerlab']['commit']+'/.gitmodules')
 for b in ['cerplan','cermap','certrack']:edge('cerflight',b,'Uses','Mission-level orchestration combines this lab module; branch-specific setup is documented.')
 edge('certrack','mavros','Uses','Explicit ROS dependency used to send flight-control commands.')
 edge('cersim','px4','Optional','Optional PX4-based simulation wrapper; the custom simulator is another path.')
 for b in ['krcontrol','mrslsim']:edge('kumar',b,'Uses','External repository pinned by the assembly’s external_all.yaml.',by['kumar']['repo']+'/blob/'+by['kumar']['commit']+'/external_all.yaml')
 edge('mrs','mrscore','Uses','MRS core is installed as part of the MRS UAV System.',by['mrscore']['repo'])
 # These subprojects are identified by the core's published repository table.
 for id,name,repo,layer,role,problem in [
  ('mrscontrol','MRS UAV controllers','mrs_uav_controllers','Control','Feedback controllers','Track desired motion with selectable feedback controllers.'),
  ('mrsestimate','MRS state estimators','mrs_uav_state_estimators','Perception','State-estimator integration','Provide state estimates through MRS estimator interfaces.'),
  ('mrsmanager','MRS UAV managers','mrs_uav_managers','Control','Flight management','Manage the controller, tracker and flight-state interfaces.'),
  ('mrshw','MRS hardware API','mrs_uav_hw_api','Control','Hardware abstraction','Connect the flight framework to vehicle hardware through a common interface.'),
  ('mrstracker','MRS UAV trackers','mrs_uav_trackers','Control','Reference tracking','Generate and manage references for the flight controllers.'),
  ('mrssim','MRS multirotor simulator','mrs_multirotor_simulator','Simulation','Flight simulation','Exercise the MRS flight core with a multirotor model.')]:
  # Only create the node when the published source names this actual repository.
  if repo not in review['ctu-mrs/mrs_uav_core']['text']:continue
  q=add(id,name,'ctu-mrs/'+repo,layer,role,problem,'Independently published package named in the MRS core repository table.','Compatible MRS core and ROS branch.','This record establishes its role in the core assembly; the individual package implementation was not re-audited for this rewrite.','ROS 2; legacy ROS 1')
  q['sources'].append(dict(label='Parent repository table / role evidence',url=by['mrscore']['repo']))
  q['evidence']='Parent repository table; individual implementation not re-audited'
  edge('mrscore',id,'Uses','Independent repository in the MRS core submodule table.',by['mrscore']['repo'])
 edge('omni','quarter','Uses','OmniNxt explicitly links this calibration tool.',by['omni']['repo']+'/blob/'+by['omni']['commit']+'/README.md')
 edge('quarter','kalibr','Code reuse','Calibration method is based on Kalibr; the workflow and variants are described in the project README.')
 for b in ['vdb','vdbros']:edge('airstack',b,'Uses','The castacks fork is a submodule in AirStack’s assembly.',by['airstack']['repo']+'/blob/'+by['airstack']['commit']+'/.gitmodules')
 edge('vdbros','vdb','Uses','ROS 2 wrapper around the VDB Mapping core.')
 # A reused package is explained at the project level, without inventing a new project node.
 for e in es:
  if (e['a'],e['b'])==('gary','fastdrone'):
   e['type']='Code reuse';e['note']='Vendors EGO integration and the internal px4ctrl package from a pinned Fast-Drone-250 revision; does not run the whole original assembly unchanged.'
  if (e['a'],e['b']) in [('ego','fast'),('egoswarm','ego'),('egov2','ego'),('mighty','gco'),('pulp','dronet'),('omni','ego')]:e['type']='Code reuse'
 es[:]=[e for e in es if (e['a'],e['b'])!=('omni','kalibr')]
 # Stable identifiers: data can appear in multiple views but is never duplicated.
 for i,e in enumerate(es):
  if e['type']=='Lineage':e['type']='Research lineage'
  e['id']=e['a']+'--'+e['b']+'--'+e['type'].lower().replace(' ','-')
  e['scope']='project relationship'
  if e['a']=='sym':e['scope']='project origin'
 # The named problem is the introduction, never a score against imagined requirements.
 problems={
 'fastdrone':'Build a small quadrotor that localizes itself and replans around sensed obstacles.',
 'cerlab':'Combine navigation, exploration and tracking in a reusable UAV reference stack.',
 'ntnu':'Bring several sensing and navigation approaches into one robot-autonomy blueprint.',
 'omni':'Give a compact aerial robot visual perception around the whole vehicle.',
 'mrs':'Provide a reusable flight core with configurable estimation, control and mission modules.',
 'as2':'Organize aerial-robot behavior across different platforms using ROS 2.',
 'kumar':'Fly through clutter without GPS using an integrated lab flight stack.',
 'agilicious':'Make agile quadrotor experiments practical with co-designed flight hardware and software.',
 'airstack':'Run layered aerial autonomy with simulation and ground control in one environment.',
 'aas':'Move multi-drone perception and control experiments from simulation toward onboard deployment.',
 'prometheus':'Integrate perception, planning and control examples around a PX4 development platform.',
 'gaas':'Assemble visual and LiDAR navigation on a legacy autonomous-flight platform.',
 'px4fast':'Connect Fast-Planner trajectories to a PX4 flight-control setup.',
 'gary':'Reproduce LiDAR localization, waypoint navigation and exploration on a small quadrotor.',
 'alt':'Operate an onboard drone agent with fleet tools and selectable extensions.',
 'uniquad':'Provide reference airframes and component designs for aerial-robotics experiments.',
 'nogps':'A claimed GPS-denied stack whose inspected repository contains no implementation.',
 'lion':'A described LiDAR navigation system whose implementation is private.',
 'super':'Plan fast LiDAR-based flight while preserving a backup trajectory in known free space.',
 'raven':'Use semantic memory and language-guided behaviors to navigate toward meaningful targets.',
 'halo':'Explore from an overhead camera and search for targets described in language.',
 'flightbench':'Compare classical and learned visual navigation under shared test scenarios.',
 'active3d':'Choose flight paths that gather useful information for 3D reconstruction.',
 'agile':'Learn rapid visual navigation from an expert with privileged scene information.',
 'clover':'Make educational autonomous-flight experiments accessible on a small reference kit.',
 'crazyswarm':'Coordinate experiments with multiple small Crazyflie drones.',
 'px4':'Stabilize the aircraft and execute flight modes, missions and external commands.',
 'ardu':'Control the aircraft and execute missions, with configurable built-in obstacle detours.',
 'vins':'Estimate motion by combining cameras and an inertial sensor.',
 'vinsmono':'Estimate motion using one camera and an inertial sensor.',
 'openvins':'Estimate position, orientation and velocity from calibrated camera/IMU measurements.',
 'orb':'Localize and build a visual landmark map, including visual-inertial configurations.',
 'msckf':'Estimate motion from stereo images and IMU measurements using a filter.',
 'fastlio':'Estimate motion and register LiDAR scans using inertial measurements.',
 'fastlivo':'Combine LiDAR, images and inertial measurements for state estimation.',
 'mimosa':'Fuse complementary sensing modalities for robot odometry.',
 'd2':'Share visual-inertial SLAM information between drones and support wide-field camera rigs.',
 'octo':'Represent occupied, free and unknown 3D space with an octree.',
 'voxblox':'Build a distance field that a motion planner can query for obstacle clearance.',
 'rog':'Maintain an efficient occupancy map that moves with the robot.',
 'nvblox':'Use a GPU to reconstruct 3D surfaces and distance information.',
 'ray':'Represent where objects and semantic targets may be found in a scene.',
 'ego':'Generate smooth local trajectories without maintaining a full distance field.',
 'egoswarm':'Replan local trajectories while coordinating with other drones.',
 'gco':'Optimize dynamically feasible polynomial trajectories efficiently.',
 'fast':'Search and optimize local routes for a quadrotor in clutter.',
 'faster':'Explore possible motion into unknown space while retaining a safe backup.',
 'mighty':'Generate efficient trajectories using a Hermite-spline representation.',
 'sando':'Plan through unknown environments containing both moving and static obstacles.',
 'dynus':'Account for uncertainty and moving obstacles during trajectory planning.',
 'la':'Choose motion that keeps localization possible in visually difficult environments.',
 'apace':'Plan motion with the quality of visual perception in mind.',
 'avoidmpc':'Generate obstacle-avoiding control from point clouds and sparse waypoints.',
 'fuel':'Choose frontiers and flight trajectories to explore an unknown environment.',
 'gb':'Use local and global graphs to explore a previously unknown environment.',
 'fc':'Plan coverage of a complex structure whose 3D representation is already available.',
 'racer':'Divide and coordinate exploration among multiple drones.',
 'yopo':'Predict useful local trajectories from depth, state and a goal.',
 'rlnav':'Turn LiDAR and state observations into acceleration and yaw commands with a learned policy.',
 'aerovla':'Translate visual observations and language instructions into UAV actions in a benchmark.',
 'pulp':'Run compact visual steering and collision prediction on a tiny embedded platform.',
 'dronet':'Predict steering and collision probability from a monocular image.',
 'rappids':'Find local collision-free motion quickly from a single depth image.',
 'nanomap':'Query nearby geometry while accounting for uncertainty in the robot’s past poses.',
 'mavros':'Translate between ROS interfaces and a MAVLink autopilot.',
 'px4ros':'Define external flight modes and control interfaces for PX4 through ROS 2.',
 'mavsdk':'Build applications that command and monitor a MAVLink vehicle.',
 'flightmare':'Simulate quadrotor dynamics and rendered observations for learning and evaluation.',
 'aerialgym':'Run many aerial-robot learning and sensing experiments in parallel.',
 'pegasus':'Bring multirotor dynamics and autopilot interfaces into Isaac Sim.',
 'skydio':'Provide commercial autonomous flight on the manufacturer’s own aircraft.',
 'dji':'Provide commercial waypoint flight and model-specific pilot assistance.',
 }
 group_map={
 'position':('Finding position','How does the robot know where it is and how it is moving?'),
 'sensing':('Observing the world','How do sensor streams become useful observations?'),
 'mapping':('Representing obstacles','What representation can a planner query about the surrounding space?'),
 'motion':('Choosing motion','How does the robot choose a route and a trajectory it can execute?'),
 'exploration':('Choosing where to look','How does the robot explore, inspect or search when the next goal is not given?'),
 'control':('Executing flight','How are desired movements turned into reliable vehicle commands?'),
 'learning':('Learning behavior','What parts of perception, navigation or control can be learned?'),
 'evaluation':('Testing and simulation','How can we train, reproduce and compare flight behavior?'),
 'hardware':('Reference platforms','What hardware and deployment infrastructure can support experiments?')}
 assembly_groups={
 'navigation':('Navigation reference systems','Assemblies for moving through environments with onboard perception and planning.'),
 'frameworks':('Reusable flight frameworks','Assemblies that organize flight software and deployment across experiments.'),
 'missions':('Exploration and semantic missions','Systems that combine perception, planning and higher-level task behavior.'),
 'experiments':('Learning and evaluation assemblies','Integrated environments and reference systems for training or comparing behavior.')}
 roots={
 'navigation':['cerlab','fastdrone','kumar','ntnu','omni','super','gary','px4fast','gaas'],
 'frameworks':['mrs','as2','airstack','aas','agilicious','prometheus','crazyswarm','clover'],
 'missions':['raven','halo','active3d'],
 'experiments':['flightbench','agile']}
 rootids=[i for ids in roots.values() for i in ids]
 hybrid={'super','agile','halo','raven','active3d','flightbench'}
 references={'nogps','lion','aeromppi','skydio','dji','nav2','fael','tare'}
 explorer={'fuel','gb','mb','fc','falcon','racer','teach','active3d','airfar','cerglobal','cerflight'}
 sensors={'realsense','oak','oak4','kalibr','quarter','cerdetect'}
 for p in ps:
  p['problem']=problems.get(p['id'],p.get('problem',p['summary'].split('. ')[0].rstrip('.')+'.'))
  p['views']=(['assembled'] if p['id'] in rootids else [])+(['focused'] if p['id'] not in rootids or p['id'] in hybrid else [])
  p['referenceOnly']=p['id'] in references
  if p['referenceOnly']:p['views']=[]
  p['assemblyGroup']=next((g for g,ids in roots.items() if p['id'] in ids),None)
  p['problemGroup']='sensing' if p['id'] in sensors else 'exploration' if p['id'] in explorer else {'Perception':'position','Mapping':'mapping','Planning':'motion','Control':'control','Learning':'learning','Simulation':'evaluation','Systems':'hardware'}[p['layer']]
  p['classificationReason']='An assembled system and a focused research contribution; the same project appears in both contexts.' if p['id'] in hybrid else 'Reference material retained for its narrower scope or release limitation.' if p['referenceOnly'] else 'Independently identifiable project. Its role is explained by the inspected sources.'
  # Avoid a duplicate unstructured list of constituents now represented by graph nodes.
  if p.get('constituents'):
   p['additionalLinks']=[q for q in p['constituents'] if q['url'] not in {x['repo'] for x in ps}]
   p.pop('constituents',None)
 for p in [by['omni']]:
  p['assemblyNote']='The guide links modified planner/controller repositories whose README fetches were unavailable during this rewrite. Those links remain in Sources; the diagram distinguishes upstream code ancestry from verified linked implementations.'
 by['mrs']['assemblyNote']='The core bundle exposes separately published packages. Optional external estimation/map integrations are separate relationships; this graph is not a claim that all module branches are simultaneously compatible.'
 by['fastdrone']['assemblyNote']='External code is included inside the reference repository. px4ctrl is an internal package, explained with the parent rather than plotted as an independent project.'
 by['kumar']['assemblyNote']='Follow the assembly’s manifest branch pins. Dependency default branches can describe a newer ROS generation than the configuration used in the published flight experiments.'
 d['canvas']={'schema':2,'assemblyGroups':[dict(id=k,title=v[0],description=v[1],projects=roots[k]) for k,v in assembly_groups.items()],'problemGroups':[dict(id=k,title=v[0],description=v[1]) for k,v in group_map.items()],'initialProject':'cerlab','identityRule':'Nodes identify independently published projects, including aggregation repositories. Internal packages are described within their parent. Multiple appearances point to the same record.','connectionRule':'Drawn integration graphs include documented Uses, Optional and Code reuse relationships. Research lineage and Evaluation are explicitly separate overlays. Missing edges are not proof of no dependencies.'}
 d['edges']=list({e['id']:e for e in es}.values())
 assert len({p['id'] for p in ps})==len(ps)
 assert all(e['a'] in by and e['b'] in by for e in d['edges'])
 assert all('/tree/' not in p['repo'] and '/blob/' not in p['repo'] for p in ps)
 return d
