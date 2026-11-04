"""Camera-aligned Earth imagery, NASA Blue Marble, illustrative 420 km altitude."""
import numpy as np
from PIL import Image
from scipy.ndimage import map_coordinates
class Earth:
 def __init__(self,folder):
  self.texture=np.asarray(Image.open(folder/'earth_nasa_8192.png').convert('RGB'),dtype=np.float32)/255.
  cloud=np.asarray(Image.open(folder/'earth_clouds_nasa.jpg').convert('L').resize((self.texture.shape[1],self.texture.shape[0])),dtype=np.float32)/255.
  self.texture=self.texture*(1-cloud[...,None]*.85)+cloud[...,None]*.85
  lat,lon=np.radians([35,-5]); self.up=np.array([np.cos(lat)*np.cos(lon),np.cos(lat)*np.sin(lon),np.sin(lat)])
  self.east=np.array([-np.sin(lon),np.cos(lon),0]); self.north=np.cross(self.up,self.east)
  x,y=np.meshgrid((np.arange(960)+.5)/960*2-1,1-(np.arange(540)+.5)/540*2)
  self.x=x*np.tan(np.radians(45)/2)*16/9; self.y=y*np.tan(np.radians(45)/2)
 def render(self,camera):
  forward=np.array(camera.forward); up=np.array(camera.up); right=np.cross(forward,up)
  ray=forward+self.x[...,None]*right+self.y[...,None]*up
  ray/=np.linalg.norm(ray,axis=-1,keepdims=True)
  b=6791*ray[...,2]; disc=b*b-(6791**2-6371**2); hit=(disc>=0)&(b<0)
  dist=-b-np.sqrt(np.maximum(disc,0)); local=ray*dist[...,None]+[0,0,6791]
  globe=local[...,0,None]*self.east+local[...,1,None]*self.north+local[...,2,None]*self.up
  globe/=np.linalg.norm(globe,axis=-1,keepdims=True)
  u=(np.arctan2(globe[...,1],globe[...,0])/(2*np.pi)+.5)*(self.texture.shape[1]-1)
  v=(.5-np.arcsin(np.clip(globe[...,2],-1,1))/np.pi)*(self.texture.shape[0]-1)
  rgb=np.stack([map_coordinates(self.texture[...,c],[v,u],order=1,mode='nearest') for c in range(3)],axis=-1)
  haze=.25*np.exp(-np.sqrt(np.maximum(disc,0))/6371*7)
  rgb=rgb*(1-haze[...,None])+haze[...,None]*[.22,.52,.88]; rgb[~hit]=0
  rim=(~hit)&(b<0); glow=np.exp(-((np.sqrt(np.maximum(-disc,0))/6371)/.025)**2)*.3
  rgb[rim]=glow[rim,None]*[.15,.4,.9]
  return np.asarray(Image.fromarray((np.clip(rgb,0,1)*255).astype('uint8')).resize((1280,720),Image.Resampling.BILINEAR))
