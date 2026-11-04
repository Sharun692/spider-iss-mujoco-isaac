"""Replay integrated spider/FEM dynamics in an ISS setting; no physics is prescribed by the renderer."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import sys,json,subprocess,copy,xml.etree.ElementTree as E
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from earth_background import Earth
OUT=ROOT/'spider_truss_iss'; ASSETS=OUT/'physics/assets'; OFFSET=np.array([-29.,8.,-1.15])
PERM=np.array([0,1,2,5,4,3]+list(range(6,18)))
def zquat(v):
 v=v/np.linalg.norm(v);q=np.r_[1+v[2],-v[1],v[0],0.];return q/np.linalg.norm(q) if np.linalg.norm(q)>1e-10 else np.array([0,1,0,0])
def build():
 tree=E.parse(ASSETS/'spider_render.xml');root=tree.getroot();w=root.find('worldbody');asset=root.find('asset');asset.clear()
 # The research robot and truss are preserved; station meshes are visual context.
 for name,color in [('station_body','.70 .73 .77 1'),('station_arrays','.13 .19 .36 1')]:
  E.SubElement(asset,'mesh',name=name,file=str(ROOT/'assets/processed'/f'{name}.stl'))
  E.SubElement(w,'geom',name='iss_'+name,type='mesh',mesh=name,rgba=color,contype='0',conaffinity='0')
 w.find("body[@name='robot_body']").set('pos',' '.join(map(str,OFFSET)))
 visual=root.find('visual');visual.find('global').set('offheight','720')
 E.SubElement(visual,'map',znear='.0001',zfar='50')
 E.SubElement(visual,'headlight',ambient='.25 .25 .28',diffuse='.45 .45 .45',specular='.1 .1 .1')
 # Root flange intersects the existing ISS beam (surface z=-1.559 m here).
 E.SubElement(w,'geom',name='root_flange',type='box',pos='-29 8 -1.48',size='.20 .25 .18',rgba='.65 .69 .73 1',contype='0',conaffinity='0')
 for j,y in enumerate([7.2,8.8]):
  E.SubElement(w,'geom',name=f'root_brace_{j}',type='capsule',fromto=f'-29 {y} -1.58 -28.84 8 -1.15',size='.045',rgba='.65 .69 .73 1',contype='0',conaffinity='0')
  E.SubElement(w,'geom',name=f'root_boltplate_{j}',type='box',pos=f'-29 {y} -1.57',size='.18 .18 .07',rgba='.50 .54 .58 1',contype='0',conaffinity='0')
 tree.write(OUT/'iss_integrated_truss_scene.xml');m=mujoco.MjModel.from_xml_path(str(OUT/'iss_integrated_truss_scene.xml'));d=mujoco.MjData(m)
 return m,d

def main():
 m,d=build();g=json.loads((ASSETS/'geometry.json').read_text());p=json.loads((ASSETS/'parameters.json').read_text())
 trajectory=OUT/'physics/results/full_mission.npz'
 # Preview can use the project's previously integrated trajectory while the fresh run completes.
 if '--preview-source' in sys.argv:trajectory=OUT/'physics/results/full_mission.npz'
 rows=np.load(trajectory)['data'];edges=np.array(g['edges'],int)-1;pairs=np.array(g['interfacePairs'],int)-1
 bars=[m.body(f'bar_{e}').mocapid[0] for e in range(len(edges))];interfaces=[m.body(f'interface_{j}').mocapid[0] for j in range(len(pairs))]
 renderer=mujoco.Renderer(m,height=720,width=1280);detail=mujoco.Renderer(m,height=216,width=360);earth=Earth(ROOT/'spider_iss');cam=mujoco.MjvCamera()
 font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
 def state(physical):
  row=rows[min(round(physical/.001),len(rows)-1)];d.qpos[:]=row[1:19][PERM]
  nodes=np.array(p['nodes'])+(np.array(p['fullModes'])@row[19:35]).reshape(96,3)
  for e,(a,b) in enumerate(edges):
   v=nodes[b]-nodes[a];d.mocap_pos[bars[e]]=(nodes[a]+nodes[b])/2+OFFSET;d.mocap_quat[bars[e]]=zquat(v);m.geom(f'bar_geom_{e}').size[1]=np.linalg.norm(v)/2
  for j,(a,b) in enumerate(pairs):d.mocap_pos[interfaces[j]]=(nodes[a]+nodes[b])/2+OFFSET
  for j in range(4):
   for side in [-1,1]:
    geom=m.geom(f'jaw_{j}_{side}');geom.pos[0]=side*(.024 if row[74+j] else .038);geom.rgba[:]=[.1,.7,.38,1] if row[74+j] else [.95,.48,.1,1]
  d.time=row[0];mujoco.mj_forward(m,d);return row
 def camera(t,row):
  # Fixed pose from 12 seconds onwards: no tracking, zoom, orbit, or cuts during walking.
  side=OFFSET+np.array([3,0,.28])
  keys=[(0,150,112,-28,[-9,0,0]),(3,150,95,-25,[-9,0,0]),
        (7,11,120,-28,OFFSET+[2.5,0,.1]),(10,7,105,-18,side),
        (12,5.6,90,-12,side),(136,5.6,90,-12,side)]
  for a,b in zip(keys,keys[1:]):
   if t<=b[0]:
    u=np.clip((t-a[0])/(b[0]-a[0]),0,1);u=u*u*(3-2*u)
    cam.distance=np.exp(np.log(a[1])*(1-u)+np.log(b[1])*u);cam.azimuth=a[2]*(1-u)+b[2]*u;cam.elevation=a[3]*(1-u)+b[3]*u;cam.lookat[:]=np.array(a[4])*(1-u)+np.array(b[4])*u;break
 earth_cache=None
 def frame(t):
  nonlocal earth_cache
  physical=min(120,max(0,t-12));row=state(physical);camera(t,row);renderer.update_scene(d,camera=cam);rgb=renderer.render().copy()
  if t<=12 or earth_cache is None:earth_cache=earth.render(renderer.scene.camera[0])
  renderer.enable_segmentation_rendering();mask=renderer.render()[...,0]<0;renderer.disable_segmentation_rendering();rgb[mask]=earth_cache[mask]
  image=Image.fromarray(rgb);draw=ImageDraw.Draw(image);draw.rounded_rectangle((20,648,650,704),radius=8,fill=(12,19,30))
  count=int(row[74:78].sum());labels='  '.join(f'{j+1}: '+('GRIP' if row[74+j] else 'SWING') for j in range(4))
  draw.text((32,654),f'MuJoCo dynamics   t = {physical:05.2f} s   |   {count}/4 attached',font=font,fill=(240,244,250));draw.text((32,679),labels+'   |   Compliant gripper model',font=font,fill=(142,221,187))
  if False:
   macro=mujoco.MjvCamera();macro.lookat[:]=d.site('foot_0').xpos;macro.distance=.28;macro.azimuth=110;macro.elevation=-12
   detail.update_scene(d,camera=macro);inset=Image.fromarray(detail.render());image.paste(inset,(900,36));draw=ImageDraw.Draw(image)
   draw.rectangle((898,12,1262,36),fill=(12,19,30));force=np.linalg.norm(row[99:102]);draw.text((908,14),f'Gripper 1 | {force:.1f} N | '+('ATTACHED' if row[74] else 'RELEASED'),font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14),fill=(210,235,225))
  return np.asarray(image)
 if '--preview-source' in sys.argv or '--preview' in sys.argv:
  for t in [0,7,12,60,132]:Image.fromarray(frame(t)).save(OUT/f'full_preview_{t}.png')
  renderer.close();detail.close();return
 path=OUT/'spider_iss_full_sideview.mp4';cmd=['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','30','-i','-','-an','-c:v','libx264','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
 with subprocess.Popen(cmd,stdin=subprocess.PIPE) as encoder:
  for i in range(4080):
   encoder.stdin.write(frame(i/30).tobytes())
   if i%300==0:print(f'RENDER {i}/4080',flush=True)
  encoder.stdin.close();encoder.wait()
  if encoder.returncode:raise RuntimeError('Encoding failed')
 renderer.close();detail.close();print(path,flush=True)
if __name__=='__main__':main()
