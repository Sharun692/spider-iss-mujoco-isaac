"""MuJoCo owns every continuous robot and FEM-modal state and integration step."""
from pathlib import Path
import sys,json,argparse,time
import numpy as np,mujoco as mj
from logic import Logic
ROOT=Path(__file__).resolve().parents[1];PERM=np.array([0,1,2,5,4,3]+list(range(6,18)))
class Simulation:
 def __init__(self,step):
  raw=json.loads((ROOT/'assets/parameters.json').read_text());self.p={k:np.array(v,dtype=float) if isinstance(v,list) else v for k,v in raw.items()};self.logic=Logic(self.p);self.m=mj.MjModel.from_xml_path(str(ROOT/'assets/spider_modal_plant.xml'));self.m.opt.timestep=step;self.d=mj.MjData(self.m);assert self.m.nq==34 and self.m.nv==34 and self.m.nu==12
  self.d.qpos[:18]=self.p['initialQ'][:18][PERM];self.d.qpos[18:]=self.p['initialQ'][18:];self.sites=np.array([mj.mj_name2id(self.m,mj.mjtObj.mjOBJ_SITE,f'foot_{j}') for j in range(4)]);self.robot_bodies=range(1,14);self.jac=np.zeros((3,34));self.J=np.zeros((4,3,34));self.last=None;self.calls=0;self.callback_time=0.;self.integrating=False;self.stage_power=[];mj.mj_forward(self.m,self.d)
 def geometry(self,d):
  for j,site in enumerate(self.sites):mj.mj_jacSite(self.m,d,self.J[j],None,int(site))
  return d.site_xpos[self.sites].copy(),self.J@d.qvel
 def control(self,m,d):
  p=self.p;c=self.logic;s,ages=c.context(d.time);qr=d.qpos[:18][PERM];vr=d.qvel[:18][PERM];eta=d.qpos[18:];ve=d.qvel[18:];x,v,_,_,weights,_,_=c.gait(s,ages);ref,refv=c.target(s,ages,qr,vr,eta,ve,x,v*c.rate);err=ref-qr[6:];tau=np.clip(p['motorK']*np.arctan2(np.sin(err),np.cos(err))+p['motorD']*(refv-vr[6:]),-p['torqueLimit'],p['torqueLimit']);d.ctrl[:]=tau
  feet,vf=self.geometry(d);a,Ja=c.anchor(c.anchors,eta);va=Ja@ve;e=a-feet;rel=va-vf;F=(p['gripK']*e+p['gripD']*rel)*weights[:,None];d.qfrc_applied[:]=np.einsum('jik,ji->k',self.J,F);Fmodal=-np.einsum('jik,ji->k',Ja,F);orbitmodal=p['Korb']@eta+p['Corb']@ve+p['Forb'];d.qfrc_applied[18:]+=Fmodal-p['Cm']@ve+orbitmodal
  orbit=np.zeros(34);zero=np.zeros(3);n=p['orbitRate']
  for body in self.robot_bodies:
   mass=m.body_mass[body];mj.mj_jacBodyCom(m,d,self.jac,None,body);velocity=self.jac@d.qvel;r=d.xipos[body];force=mass*np.array([-2*n*velocity[2],-n*n*r[1],2*n*velocity[0]+3*n*n*r[2]]);mj.mj_applyFT(m,d,force,zero,r,body,orbit)
  d.qfrc_applied[:]+=orbit;pc=.5*p['gripK']*np.sum(np.sum(e*e,axis=1)*c.weight_rate(s,ages));U=.5*p['gripK']*np.sum(np.sum(e*e,axis=1)*weights);diss=p['gripD']*np.sum(np.sum(rel*rel,axis=1)*weights)+ve@p['Cm']@ve
  self.last=(tau.copy(),F.copy(),U,diss,float(orbit@d.qvel+orbitmodal@ve),pc);self.calls+=1
  if self.integrating:self.stage_power.append(float(tau@vr[6:]+orbit@d.qvel+orbitmodal@ve+pc-diss))
 def row(self):
  d=self.d;p=self.p;tau,F,U,diss,po,pc=self.last;mj.mj_energyPos(self.m,d);mj.mj_energyVel(self.m,d);qr=d.qpos[:18][PERM];vr=d.qvel[:18][PERM];eta=d.qpos[18:];ve=d.qvel[18:];metrics=[d.energy.sum()+U,tau@vr[6:],diss,po,p['tipModal']@eta,pc];return np.r_[d.time,qr,eta,vr,ve,self.logic.pack(d.time),tau,F.ravel(),metrics]
 def run(self,duration):
  steps=round(duration/self.m.opt.timestep);rows=np.empty((steps+1,117));work=np.zeros(steps+1);started=time.perf_counter();mj.set_mjcb_control(self.control)
  try:
   for k in range(steps+1):
    self.d.time=k*self.m.opt.timestep;mj.mj_forward(self.m,self.d);rows[k]=self.row();feet,vf=self.geometry(self.d);self.logic.update(self.d.time,self.d.qpos[:18][PERM],self.d.qvel[:18][PERM],self.d.qpos[18:],self.d.qvel[18:],feet,vf);mj.mj_forward(self.m,self.d)
    if k%max(1,round(5/self.m.opt.timestep))==0:print(f'MUJOCO_ONLY t={self.d.time:.3f} x={self.d.qpos[0]:.5f} gait={self.logic.s:.3f} grips={int(self.logic.latched.sum())}',flush=True)
    if k<steps:
     self.stage_power=[];self.integrating=True;mj.mj_step(self.m,self.d);self.integrating=False
     if len(self.stage_power)!=4:raise RuntimeError(f'Expected four RK4 control evaluations, got {len(self.stage_power)}')
     work[k+1]=work[k]+self.m.opt.timestep*np.dot(self.stage_power,[1,2,2,1])/6
    if not np.isfinite(self.d.qpos).all() or not np.isfinite(self.d.qvel).all():raise RuntimeError('Nonfinite MuJoCo state')
  finally:mj.set_mjcb_control(None)
  return rows,time.perf_counter()-started,work

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=120);parser.add_argument('--step',type=float,default=.001);parser.add_argument('--name',default='mission');args=parser.parse_args();sim=Simulation(args.step);rows,elapsed,work=sim.run(args.duration);np.savez_compressed(ROOT/f'results/{args.name}.npz',data=rows,rk4_work=work);print('MUJOCO_ONLY_COMPLETE',elapsed,flush=True)
 report={'duration_s':args.duration,'step_s':args.step,'runtime_s':elapsed,'integrator':'MuJoCo mj_step RK4 for robot and 16 FEM coordinates','control_callbacks':sim.calls,'MATLAB_runtime_used':False,'Simulink_runtime_used':False,'ROS2_used':False,'attachment_scope':'same captured compliant bonds; geometry contact disabled'};(ROOT/f'results/{args.name}_run.json').write_text(json.dumps(report,indent=2))
if __name__=='__main__':main()
