"""Render the newly integrated PhysX trajectory with Isaac RTX, then keep the side camera fixed."""
import sys,argparse,json,subprocess,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python_deps'))
import scipy.io
parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true');parser.add_argument('--trajectory',default='physx_full');parser.add_argument('--gui',action='store_true');args=parser.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':not args.gui,'renderer':'RaytracedLighting','width':1280,'height':720,'multi_gpu':False,'max_gpu_count':1,'disable_viewport_updates':True})
import numpy as np,omni.usd,carb
import omni.replicator.core as rep
from PIL import Image,ImageDraw,ImageFont
from scene_rtx import build,update,camera
try:
 z=np.load(ROOT/'results'/f'{args.trajectory}.npz');rows=z['data'];poses=z['link_poses'];body_names=z['body_names']
 omni.usd.get_context().open_stage(str(ROOT/'imported_spider.usda'));stage=omni.usd.get_context().get_stage();handles=build(stage,body_names)
 from omni.kit.viewport.utility import get_active_viewport
 viewport=get_active_viewport()
 if viewport:viewport.camera_path='/World/Camera'
 settings=carb.settings.get_settings();settings.set('/exts/omni.replicator.core/maxAssetLoadingTime',25.);settings.set('/exts/omni.replicator.core/initMinLoadTime',2.);settings.set('/rtx/indirectDiffuse/enabled',True);settings.set('/rtx/ambientOcclusion/enabled',True);settings.set('/rtx/post/dlss/execMode',2)
 product=rep.create.render_product('/World/Camera',(1280,720));annotator=rep.AnnotatorRegistry.get_annotator('rgb');annotator.attach([product])
 font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
 def frame(t):
  physical=min(float(rows[-1,0]),max(0,t-12));i=int(np.argmin(abs(rows[:,0]-physical)));row=rows[i]
  update(handles,row,poses[i]);camera(handles,t)
  rep.orchestrator.step(rt_subframes=4 if args.preview else 2,pause_timeline=True)
  image=Image.fromarray(np.asarray(annotator.get_data())[...,:3].astype(np.uint8));draw=ImageDraw.Draw(image);draw.rounded_rectangle((20,648,720,704),radius=8,fill=(12,19,30));draw.text((32,654),f'Isaac Sim | GPU PhysX | t = {physical:06.2f} s | {int(row[74:78].sum())}/4 attached',font=font,fill=(240,244,250));draw.text((32,679),'Assumed compliant grippers + 16-mode truss | RTX visuals',font=font,fill=(142,221,187));return np.asarray(image)
 for t in [0,7,12]:
  Image.fromarray(frame(t)).save(ROOT/f'results/isaac_preview_{int(t)}.png');print('RTX_PREVIEW',t,flush=True)
 stage.GetRootLayer().Export(str(ROOT/'iss_spider_scene.usda'))
 if not args.preview:
  path=ROOT/'results/isaac_iss_full_sideview.mp4';cmd=['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r','30','-i','-','-an','-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
  with subprocess.Popen(cmd,stdin=subprocess.PIPE) as encoder:
   for i in range(4080):
    encoder.stdin.write(frame(i/30).tobytes())
    if i%300==0:print(f'RTX_RENDER {i}/4080',flush=True)
   encoder.stdin.close();encoder.wait()
   if encoder.returncode:raise RuntimeError('Encoding failed')
  print('RTX_VIDEO_COMPLETE',path,flush=True)
except Exception:
 traceback.print_exc();(ROOT/'results/render_FAILED.txt').write_text(traceback.format_exc())
finally:app.close()
