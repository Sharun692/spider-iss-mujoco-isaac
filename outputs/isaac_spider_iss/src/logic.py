"""Direct port of the established gait, capture and spatial IK logic."""
import numpy as np

def smooth(u):
 u=np.clip(u,0.,1.);return 10*u**3-15*u**4+6*u**5

def smooth_derivative(u):
 u=np.clip(u,0.,1.);return 30*u*u-60*u**3+30*u**4

def rotation(q):
 f,t,s=q;cf,sf,ct,st,cs,ss=np.cos(f),np.sin(f),np.cos(t),np.sin(t),np.cos(s),np.sin(s)
 return np.array([[cs*ct,cs*st*sf-ss*cf,cs*st*cf+ss*sf],[ss*ct,ss*st*sf+cs*cf,ss*st*cf-cs*sf],[-st,ct*sf,ct*cf]])

class Logic:
 def __init__(self,p):
  self.p=p;self.phi=p['fullModes'].reshape(96,3,16);self.rails=np.array([-1,-1,1,1])*p['width']/2
  self.s=0.;self.anchors=p['initialAnchors'].copy();self.latched=np.ones(4);self.ages=np.ones(4);self.last_error=np.zeros(4);self.rate=1.;self.t0=0.
 def context(self,t):
  dt=t-self.t0;return min(self.s+self.rate*dt,self.p['walkTime']),self.ages+dt
 def gait(self,s,ages):
  p=self.p;u=np.clip(s/p['walkTime'],0.,1.);x=p['x0']+(p['xf']-p['x0'])*smooth(u);vel=(p['xf']-p['x0'])*(30*u*u-60*u**3+30*u**4)/p['walkTime'];cycle=np.floor(s/p['period']);j=int(cycle)%4;phase=(s-cycle*p['period'])/p['period'];weights=self.latched*smooth(ages/.2);goal=self.anchors.copy();foot=np.stack([self.anchors.copy(),np.full(4,p['rail'])]);pattern=np.array([-1,-1,1,1]);foot[1]+=pattern*p['prestrain']*weights*smooth(s/2)
  if s<p['walkTime']:
   uu=min((cycle+1)*p['period']/p['walkTime'],1.);goal[j]=p['x0']+(p['xf']-p['x0'])*smooth(uu)+p['offset'][j]
   if phase<.2:
    weights[j]*=1-smooth(phase/.1)
    if self.latched[j]:foot[1,j]=p['rail']+pattern[j]*p['prestrain']*weights[j]*smooth(s/2)
   if not self.latched[j]:
    a=np.clip((phase-.1)/.65,0.,1.);foot[0,j]=self.anchors[j]+(goal[j]-self.anchors[j])*smooth(a);foot[1,j]=p['rail']+p['clearance']*16*a*a*(1-a)**2
  return x,vel,j,phase,weights,goal,foot
 def weight_rate(self,s,ages):
  base=self.latched*smooth(ages/.2);rate=self.latched*smooth_derivative(ages/.2)/.2
  if s<self.p['walkTime']:
   cycle=np.floor(s/self.p['period']);j=int(cycle)%4;phase=(s-cycle*self.p['period'])/self.p['period']
   if phase<.2:rate[j]=rate[j]*(1-smooth(phase/.1))-base[j]*smooth_derivative(phase/.1)*self.rate/(.1*self.p['period'])
  return rate
 def anchor(self,positions,eta):
  p=self.p;a=np.clip(positions,0,p['L']);modules=np.minimum(np.floor(a/p['bay']).astype(int),int(p['nb'])-1);u=(a-modules*p['bay'])/p['bay'];corners=np.array([0,0,1,1]);n1=8*modules+corners;n2=n1+4;J=(1-u[:,None,None])*self.phi[n1]+u[:,None,None]*self.phi[n2];r=np.stack([a,self.rails,np.full(4,p['rail'])],axis=1)+J@eta;return r,J
 def angles(self,s,ages,qr,eta,xref):
  p=self.p;_,_,_,_,_,_,footref=self.gait(s,ages);a,_=self.anchor(self.anchors,eta);undeformed=np.stack([self.anchors,self.rails,np.full(4,p['rail'])],axis=1);on=self.latched.astype(bool);shift=(a-undeformed)[on if np.any(on) else np.ones(4,dtype=bool)].mean(axis=0);desired=np.array([xref,0,p['zref']])+shift;r=a-desired-p['hips'].T
  swing=~self.latched.astype(bool)
  if np.any(swing):
   targets,_=self.anchor(footref[0],eta);targets[:,2]+=footref[1]-p['rail'];r[swing]=(targets[swing]-qr[:3])@rotation(qr[3:6])-p['hips'].T[swing]
  rho=np.hypot(r[:,0],r[:,1]);d=np.linalg.norm(r,axis=1)
  if np.any(d>p['l1']+p['l2']+1e-7) or np.any(d<=abs(p['l1']-p['l2'])):raise RuntimeError('Spider foot target outside limb reach')
  gamma=np.arctan2(r[:,1],r[:,0]);shoulder=np.arctan2(rho,-r[:,2])+np.arccos(np.clip((p['l1']**2+d*d-p['l2']**2)/(2*p['l1']*d),-1,1));knee=-np.arccos(np.clip((d*d-p['l1']**2-p['l2']**2)/(2*p['l1']*p['l2']),-1,1));return np.stack([gamma,shoulder,knee],axis=1).ravel()
 def target(self,s,ages,qr,vr,eta,ve,x,vel):
  ep=1e-5;ref=self.angles(s,ages,qr,eta,x);rp=self.angles(min(s+self.rate*ep,self.p['walkTime']),ages+ep,qr+ep*vr,eta+ep*ve,x+ep*vel);rn=self.angles(max(s-self.rate*ep,0.),ages-ep,qr-ep*vr,eta-ep*ve,x-ep*vel);delta=rp-rn;return ref,np.arctan2(np.sin(delta),np.cos(delta))/(2*ep)
 def update(self,t,qr,vr,eta,ve,feet,footvel):
  self.s,self.ages=self.context(t);self.t0=t
  if self.s>=self.p['walkTime']:self.rate=0.;return
  _,_,j,phase,_,goal,_=self.gait(self.s,self.ages)
  if .1<=phase<.2 and self.latched[j] and self.latched.sum()==4:self.latched[j]=0.;self.ages[j]=0.
  if phase>=.75 and not self.latched[j]:
   a,J=self.anchor(goal,eta);gap=np.linalg.norm(a[j]-feet[j]);speed=np.linalg.norm(J[j]@ve-footvel[j])
   if gap<=self.p['captureDistance'] and speed<=self.p['captureSpeed']:self.latched[j]=1.;self.anchors[j]=goal[j];self.ages[j]=0.;self.last_error[j]=gap
  self.rate=float(not(phase>=.9 and not self.latched[j]))
 def pack(self,t):
  s,ages=self.context(t);return np.r_[s,self.anchors,self.latched,ages,self.last_error,self.rate]
