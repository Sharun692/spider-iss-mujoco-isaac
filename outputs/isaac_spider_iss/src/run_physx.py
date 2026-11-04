"""Independent GPU PhysX integration of the existing compliant-grasp research model.
No MuJoCo runtime, no reference trajectory input. Assumed coefficients are unchanged.
"""
from pathlib import Path
import sys,argparse,json,time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python_deps'))
import scipy.io
from isaacsim import SimulationApp
args=argparse.ArgumentParser();args.add_argument('--duration',type=float,default=3);args.add_argument('--step',type=float,default=.0005);args.add_argument('--name',default='physx_pilot');args.add_argument('--solver',default='TGS');args.add_argument('--sample-period',type=float,default=.01);opt=args.parse_args()
app=SimulationApp({'headless':True,'width':640,'height':360,'multi_gpu':False,'max_gpu_count':1})
import numpy as np,torch,omni.usd
from scipy.spatial.transform import Rotation
from pxr import UsdPhysics,PhysxSchema,UsdGeom,Gf
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from logic import Logic

def cpu(t):return t.detach().cpu().numpy().copy()
def gpu(a):return torch.as_tensor(a,dtype=torch.float32,device='cuda:0').contiguous()
raw=json.loads((ROOT/'assets/parameters.json').read_text());p={k:np.asarray(v,float) if isinstance(v,list) else v for k,v in raw.items()};logic=Logic(p)
omni.usd.get_context().open_stage(str(ROOT/'imported_spider.usda'));stage=omni.usd.get_context().get_stage()
# Give each modal slider a fixed-base articulation; a bare slider to world is not a PhysX articulation DOF.
for i in range(16):
 body=stage.GetPrimAtPath(f'/World/Spider/fem_mode_{i}/fem_mode_{i}');body.RemoveAPI(UsdPhysics.ArticulationRootAPI)
 base=UsdGeom.Xform.Define(stage,f'/World/ModeBase_{i}').GetPrim();UsdPhysics.RigidBodyAPI.Apply(base);UsdPhysics.MassAPI.Apply(base).CreateMassAttr(1.)
 UsdPhysics.ArticulationRootAPI.Apply(base)
 fixed=UsdPhysics.FixedJoint.Define(stage,f'/World/ModeFixed_{i}');fixed.CreateBody1Rel().SetTargets([base.GetPath()])
 joint=UsdPhysics.PrismaticJoint.Get(stage,f'/World/Spider/joints/modal_eta_{i}');joint.CreateBody0Rel().SetTargets([base.GetPath()])
# Zero implicit motor gains. Modal stiffness is an implicit PhysX prismatic drive.
for prim in stage.Traverse():
 if prim.IsA(UsdPhysics.Joint):
  for attr in prim.GetAttributes():
   if attr.GetName().endswith('physics:maxForce'):attr.Set(1e10)
 if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
  api=PhysxSchema.PhysxArticulationAPI.Apply(prim);api.CreateSolverPositionIterationCountAttr(32);api.CreateSolverVelocityIterationCountAttr(4);api.CreateSleepThresholdAttr(0.);api.CreateStabilizationThresholdAttr(0.)
world=World(physics_dt=opt.step,rendering_dt=1/30,stage_units_in_meters=1,backend='torch',device='cuda:0',set_defaults=False)
pc=world.get_physics_context();pc.set_gravity(0);pc.set_solver_type(opt.solver);pc.enable_gpu_dynamics(True);pc.set_broadphase_type('GPU')
robot=world.scene.add(Articulation('/World/Spider/robot_body/robot_body',name='robot'));world.reset()
view=robot._physics_view;simview=SimulationManager.get_physics_sim_view();modes=simview.create_articulation_view('/World/ModeBase_*')
mi=np.array([int(s.split('ModeBase_')[-1]) for s in modes.prim_paths]);invmi=np.argsort(mi)
ids=torch.arange(modes.count,device='cuda:0',dtype=torch.int32);rid=torch.tensor([0],device='cuda:0',dtype=torch.int32)
names=[f'{name}_{j}' for j in range(4) for name in ['yaw','shoulder','knee']];ji=np.array([robot.dof_names.index(n) for n in names]);fi=np.array([robot.body_names.index(f'grip_{j}') for j in range(4)])
initial=np.zeros((1,12));initial[0,ji]=p['initialQ'][6:18];robot.set_joint_positions(gpu(initial));robot.set_joint_velocities(gpu(np.zeros((1,12))));robot.set_world_poses(gpu(p['initialQ'][:3][None]),gpu([[1,0,0,0]]));robot.set_velocities(gpu(np.zeros((1,6))));robot.set_gains(gpu(np.zeros((1,12))),gpu(np.zeros((1,12))))
modes.set_dof_positions(gpu(np.zeros((16,1))),ids);modes.set_dof_velocities(gpu(np.zeros((16,1))),ids)
# The original MJCF import keeps modal drive constants, mapped by explicit prim name.
print('GPU_SETUP',pc.is_gpu_dynamics_enabled(),robot._device,'MODES',modes.count,'DOFS',robot.dof_names,flush=True)
world.step(render=False)
robot.set_joint_velocities(gpu(np.zeros((1,12))));robot.set_velocities(gpu(np.zeros((1,6))))
mass=cpu(robot.get_body_masses())[0];rows=[];pose_rows=[];velocity_rows=[];native_kinetic=[];motor_work_rows=[];held_motor_work=0.;previous_tau=None;previous_v=None;started=time.perf_counter();samples=max(1,round(opt.sample_period/opt.step));last_flags=logic.latched.copy();events=[]
try:
 for k in range(round(opt.duration/opt.step)+1):
  t=k*opt.step
  transforms=cpu(view.get_link_transforms())[0];velocities=cpu(view.get_link_velocities())[0];q=cpu(robot.get_joint_positions())[0,ji];v=cpu(robot.get_joint_velocities())[0,ji]
  euler=Rotation.from_quat(transforms[0,3:7]).as_euler('xyz');roll,pitch,yaw=euler;cp=np.cos(pitch);sp=np.sin(pitch);cy=np.cos(yaw);sy=np.sin(yaw)
  rate_matrix=np.array([[cy*cp,-sy,0],[sy*cp,cy,0],[-sp,0,1]])
  qr=np.r_[transforms[0,:3],euler,q];vr=np.r_[velocities[0,:3],np.linalg.solve(rate_matrix,velocities[0,3:6]),v]
  if previous_tau is not None:held_motor_work+=float(.5*opt.step*previous_tau@(previous_v+v))
  eta=cpu(modes.get_dof_positions())[:,0][invmi];ve=cpu(modes.get_dof_velocities())[:,0][invmi]
  feet=transforms[fi,:3];vf=velocities[fi,:3]
  logic.update(t,qr,vr,eta,ve,feet,vf)
  if not np.array_equal(logic.latched,last_flags):events.append([t,*logic.latched.tolist()]);last_flags=logic.latched.copy()
  s,ages=logic.context(t);xref,xvel,_,_,weights,_,_=logic.gait(s,ages)
  ref,refv=logic.target(s,ages,qr,vr,eta,ve,xref,xvel*logic.rate)
  tau=np.clip(p['motorK']*np.arctan2(np.sin(ref-q),np.cos(ref-q))+p['motorD']*(refv-v),-p['torqueLimit'],p['torqueLimit'])
  a,Ja=logic.anchor(logic.anchors,eta);va=Ja@ve
  F=(p['gripK']*(a-feet)+p['gripD']*(va-vf))*weights[:,None]
  applied=np.zeros((1,len(mass),3));n=p['orbitRate'];pos=transforms[:,:3];vv=velocities[:,:3]
  applied[0]=mass[:,None]*np.stack([-2*n*vv[:,2],-n*n*pos[:,1],2*n*vv[:,0]+3*n*n*pos[:,2]],axis=1)
  applied[0,fi]+=F
  view.apply_forces_and_torques_at_position(gpu(applied),None,None,rid,True)
  modal=-np.einsum('jik,ji->k',Ja,F)-p['Cm']@ve+p['Korb']@eta+p['Corb']@ve+p['Forb']
  modes.set_dof_actuation_forces(gpu(modal[mi,None]),ids)
  effort=np.zeros((1,12));effort[0,ji]=tau;robot.set_joint_efforts(gpu(effort))
  if k%samples==0:
   rows.append(np.r_[t,qr,eta,vr,ve,logic.pack(t),tau,F.ravel()]);pose_rows.append(transforms.copy());velocity_rows.append(velocities.copy());mm=cpu(robot.get_mass_matrices())[0];genvel=np.r_[velocities[0],cpu(robot.get_joint_velocities())[0]];native_kinetic.append(float(.5*genvel@mm@genvel));motor_work_rows.append(held_motor_work)
  if k%max(1,round(1/opt.step))==0:print(f'PHYSX t={t:.3f} x={qr[0]:.6f} grips={int(logic.latched.sum())} maxforce={np.linalg.norm(F,axis=1).max():.2f}',flush=True)
  if not np.isfinite(qr).all() or np.linalg.norm(qr[:3])>20 or np.linalg.norm(ve)>1e5:raise RuntimeError('Unstable state')
  previous_tau=tau.copy();previous_v=v.copy()
  if k<round(opt.duration/opt.step):world.step(render=False)
 data=np.asarray(rows);np.savez_compressed(ROOT/f'results/{opt.name}.npz',data=data,link_poses=np.asarray(pose_rows),body_names=np.asarray(robot.body_names),link_velocities=np.asarray(velocity_rows),body_masses=mass,body_coms=cpu(view.get_coms())[0],body_inertias=cpu(view.get_inertias())[0],native_kinetic=np.asarray(native_kinetic),held_motor_work=np.asarray(motor_work_rows))
 report={'engine':'Isaac Sim 5.1 / GPU PhysX','gpu_dynamics_enabled':bool(pc.is_gpu_dynamics_enabled()),'device':robot._device,'duration_s':opt.duration,'step_s':opt.step,'elapsed_s':time.perf_counter()-started,'body_advance_m':float(data[-1,1]-data[0,1]),'minimum_grips':int(data[:,74:78].sum(1).min()),'peak_motor_torque_Nm':float(abs(data[:,87:99]).max()),'grip_events':events,'scope':'Existing assumed compliant bonds and 16-mode FEM; no resolved jaw friction; TGS integration differs from MuJoCo RK4.'}
 (ROOT/f'results/{opt.name}.json').write_text(json.dumps(report,indent=2));print('PHYSX_RUN_COMPLETE',json.dumps(report),flush=True)
except Exception:
 import traceback;traceback.print_exc();(ROOT/f'results/{opt.name}_FAILED.txt').write_text(traceback.format_exc())
finally:world.stop();app.close()
