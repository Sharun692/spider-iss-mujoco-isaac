"""Independent source-geometry kinematics and sampled work/energy audit of the PhysX port."""
from pathlib import Path
import json,sys,xml.etree.ElementTree as E
import numpy as np
from logic import Logic,rotation
ROOT=Path(__file__).resolve().parents[1]
name=sys.argv[1] if len(sys.argv)>1 else 'physx_full';z=np.load(ROOT/'results'/f'{name}.npz');data=z['data'];saved_poses=z['link_poses'] if 'link_poses' in z else None;saved_vel=z['link_velocities'] if 'link_velocities' in z else None;body_names=z['body_names'] if 'body_names' in z else None;native_kin=z['native_kinetic'] if 'native_kinetic' in z else None;raw=json.loads((ROOT/'assets/parameters.json').read_text());p={k:np.asarray(v,float) if isinstance(v,list) else v for k,v in raw.items()};c=Logic(p)
source={b.get('name'):b for b in E.parse(ROOT/'bridge_mjcf.xml').iter('body')}
def rz(a):return np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
def ry(a):return np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
def fk(q,v):
 R=rotation(q[3:6]);roll,pitch,yaw=q[3:6];M=np.array([[np.cos(yaw)*np.cos(pitch),-np.sin(yaw),0],[np.sin(yaw)*np.cos(pitch),np.cos(yaw),0],[-np.sin(pitch),0,1]]);omega=M@v[3:6];root=q[:3];rootv=v[:3];b={'robot_body':(root,rootv,omega,R)}
 for j in range(4):
  gamma,shoulder,knee=q[6+3*j:9+3*j];gv,sv,kv=v[6+3*j:9+3*j];hip=root+R@p['hips'][:,j];hipv=rootv+np.cross(omega,R@p['hips'][:,j]);Y=R@rz(gamma);yw=omega+R@np.array([0,0,gv]);A=Y@ry(-shoulder);aw=yw+Y@np.array([0,-sv,0]);kp=hip+A@np.array([0,0,-p['l1']]);kpv=hipv+np.cross(aw,A@np.array([0,0,-p['l1']]));B=A@ry(-knee);bw=aw+A@np.array([0,-kv,0]);fp=kp+B@np.array([0,0,-p['l2']]);fpv=kpv+np.cross(bw,B@np.array([0,0,-p['l2']]));b[f'yaw_carrier_{j}']=(hip,hipv,yw,Y);b[f'link_{j}_0']=(hip,hipv,aw,A);b[f'link_{j}_1']=(kp,kpv,bw,B);b[f'grip_{j}']=(fp,fpv,bw,B)
 return b
Kdiag=[]
for i in range(16):Kdiag.append(float(source[f'fem_mode_{i}'].find('joint').get('stiffness')))
energies=[];powers=[];robot_kin=[];maxkin=0;maxvel=0;flags=data[:,74:78]
for index,row in enumerate(data):
 q=row[1:19];eta=row[19:35];v=row[35:53];ve=row[53:69];tau=row[87:99];links=fk(q,v);kin=0.;orbitpower=0.
 if saved_poses is not None:
  for j,(body,pose) in enumerate(zip(body_names,saved_poses[index])):
   maxkin=max(maxkin,float(np.linalg.norm(links[str(body)][0]-pose[:3])))
   if saved_vel is not None and str(body).startswith('grip_'):maxvel=max(maxvel,float(np.linalg.norm(links[str(body)][1]-saved_vel[index,j,:3])))
 for body,(pos,vel,omega,R) in links.items():
  inert=source[body].find('inertial');mass=float(inert.get('mass'));local_com=np.fromstring(inert.get('pos','0 0 0'),sep=' ');comv=vel+np.cross(omega,R@local_com);iw=np.fromstring(inert.get('diaginertia'),sep=' ');wl=R.T@omega;kin+=.5*(mass*(comv@comv)+(iw*wl)@wl);n=p['orbitRate'];force=mass*np.array([-2*n*vel[2],-n*n*pos[1],2*n*vel[0]+3*n*n*pos[2]]);orbitpower+=force@vel
 c.s=row[69];c.anchors=row[70:74].copy();c.latched=row[74:78].copy();c.ages=row[78:82].copy();c.last_error=row[82:86].copy();c.rate=row[86];c.t0=row[0];s,ages=c.context(row[0]);weights=c.gait(s,ages)[4];a,Ja=c.anchor(c.anchors,eta);feet=np.array([links[f'grip_{j}'][0] for j in range(4)]);vf=np.array([links[f'grip_{j}'][1] for j in range(4)]);error=a-feet;relative=Ja@ve-vf;U=.5*p['gripK']*np.sum(weights*np.sum(error**2,axis=1));energy=kin+.5*(ve@ve+(eta*np.asarray(Kdiag))@eta)+U;diss=p['gripD']*np.sum(weights*np.sum(relative**2,axis=1))+ve@p['Cm']@ve;changing=.5*p['gripK']*np.sum(c.weight_rate(s,ages)*np.sum(error**2,axis=1));orbitpower+=(p['Korb']@eta+p['Corb']@ve+p['Forb'])@ve;power=tau@v[6:]+orbitpower+changing-diss;energies.append(energy);powers.append(power);robot_kin.append(kin)
energies=np.asarray(energies);powers=np.asarray(powers);work=np.r_[0,np.cumsum(.5*(powers[1:]+powers[:-1])*np.diff(data[:,0]))];residual=energies-energies[0]-work
continuous_residual=residual.copy();meta_path=ROOT/'results'/f'{name}.json';meta=json.loads(meta_path.read_text()) if meta_path.exists() else {};step=meta.get('step_s');fullrate=bool(step and np.max(np.diff(data[:,0]))<=step*1.01);correction=np.zeros(len(data))
if fullrate:
 tau=data[:,87:99];vel=data[:,41:53];h=np.diff(data[:,0]);held=.5*np.sum(tau[:-1]*(vel[:-1]+vel[1:]),axis=1)*h;trap=.5*np.sum(tau[:-1]*vel[:-1]+tau[1:]*vel[1:],axis=1)*h;correction[1:]=np.cumsum(held-trap);residual=residual-correction;work=work+correction
report={'trajectory':name,'duration_s':float(data[-1,0]),'finite':bool(np.isfinite(data).all()),'source_geometry_fk_max_position_error_m':maxkin,'max_foot_velocity_difference_m_per_s':maxvel if saved_vel is not None else None,'minimum_attached_grippers':int(flags.sum(1).min()),'released_each_leg':(np.diff(flags,axis=0)<0).sum(0).tolist(),'recaptured_each_leg':(np.diff(flags,axis=0)>0).sum(0).tolist(),'peak_motor_torque_Nm':float(abs(data[:,87:99]).max()),'body_advance_m':float(data[-1,1]-data[0,1]),'energy_audit_at_solver_rate':fullrate,'naive_continuous_torque_relative_residual':float(abs(continuous_residual).max()/max(np.ptp(energies),1e-9)),'sampled_energy_balance_max_residual_J':float(abs(residual).max()),'sampled_energy_balance_relative_residual':float(abs(residual).max()/max(np.ptp(energies),1e-9)),'native_mass_matrix_kinetic_energy_max_difference_J':float(np.max(abs(np.asarray(robot_kin)-native_kin))) if native_kin is not None else None,'energy_audit_note':'Held-input midpoint motor work when logging at solver rate; other powers trapezoidal. Coarse mission logs are not sufficient for a full step-rate energy validation. Numerical diagnostic, not experimental validation.'}
(ROOT/'results'/f'{name}_checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));np.savez_compressed(ROOT/'results'/f'{name}_energy.npz',time=data[:,0],energy=energies,work=work,residual=residual,continuous_torque_residual=continuous_residual,motor_work_correction=correction,robot_kinetic=np.asarray(robot_kin))
assert report['finite'] and maxkin<.001 and report['minimum_attached_grippers']>=3 and report['peak_motor_torque_Nm']<=40.0001

if data[-1,0]>=96:
 assert abs(report['body_advance_m']-4.7)<.02 and int(flags[-1].sum())==4 and min(report['recaptured_each_leg'])>=32, 'Full traversal checks failed'
