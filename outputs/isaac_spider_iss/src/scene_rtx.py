"""RTX scene builder using original NASA meshes and actual saved PhysX link poses."""
from pathlib import Path
import json,numpy as np,xml.etree.ElementTree as ET
from pxr import Usd,UsdGeom,UsdShade,UsdLux,UsdPhysics,Gf,Sdf,Vt
ROOT=Path(__file__).resolve().parents[1];OFFSET=np.array([-29.,8.,-1.15])
PREVIOUS=ROOT.parent/'canadarm-capture';EARTH_ASSETS=PREVIOUS/'spider_iss'
def material(stage,name,color,metal=0.,rough=.5,texture=None,emissive=False,opacity=None):
 m=UsdShade.Material.Define(stage,'/World/Looks/'+name);s=UsdShade.Shader.Define(stage,m.GetPath().AppendChild('Surface'));s.CreateIdAttr('UsdPreviewSurface')
 s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));s.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal);s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(rough)
 if texture or opacity:
  reader=UsdShade.Shader.Define(stage,m.GetPath().AppendChild('UV'));reader.CreateIdAttr('UsdPrimvarReader_float2');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');reader.CreateOutput('result',Sdf.ValueTypeNames.Float2)
  for key,file in [('Color',texture),('Opacity',opacity)]:
   if file:
    tex=UsdShade.Shader.Define(stage,m.GetPath().AppendChild(key));tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(file));tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result');tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3);tex.CreateOutput('r',Sdf.ValueTypeNames.Float)
    if key=='Color':s.CreateInput('emissiveColor' if emissive else 'diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
    else:s.CreateInput('opacity',Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(),'r')
 m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m

def bind(prim,mat):UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
def matrix(prim):
 x=UsdGeom.Xformable(prim);x.ClearXformOpOrder();return x.AddTransformOp()
def posemat(pos,quat):
 q=Gf.Quatd(float(quat[3]),Gf.Vec3d(*map(float,quat[:3])));m=Gf.Matrix4d(1);m.SetRotate(q);m.SetTranslateOnly(Gf.Vec3d(*map(float,pos)));return m

def build(stage,body_names):
 # Use clean USD visual primitives matching the source XML; no physics/Fabric transforms in offline rendering.
 for path in ['/World/Spider','/visuals','/collisions','/meshes','/Tmp']:
  stage.RemovePrim(path)
 stage.SetDefaultPrim(stage.GetPrimAtPath('/World'));UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
 metals=material(stage,'StationAluminium',(.65,.69,.74),.6,.32);arrays=material(stage,'SolarCells',(.025,.045,.13),.3,.24)
 steel=material(stage,'TrussAlloy',(.42,.58,.68),.7,.29);node_mat=material(stage,'Couplings',(.60,.46,.20),.65,.32)
 green=material(stage,'Attached',(.04,.55,.22),.15,.45);orange=material(stage,'Released',(.9,.25,.035),.15,.45);dark=material(stage,'RobotBody',(.22,.30,.38),.4,.35);leg=material(stage,'RobotLinks',(.48,.57,.65),.65,.3)
 for name,mat in [('station_body',metals),('station_arrays',arrays)]:
  z=np.load(ROOT/'assets'/f'{name}.npz');mesh=UsdGeom.Mesh.Define(stage,'/World/ISS/'+name);mesh.CreatePointsAttr(z['vertices']);mesh.CreateFaceVertexCountsAttr(np.full(len(z['faces']),3));mesh.CreateFaceVertexIndicesAttr(z['faces'].ravel());mesh.CreateSubdivisionSchemeAttr('none');bind(mesh.GetPrim(),mat)
 body_ops=[];jaw_ops=[]
 source=ET.parse(ROOT/'bridge_mjcf.xml').getroot();source_bodies={b.get('name'):b for b in source.iter('body')}
 for name in body_names:
  name=str(name);parent=UsdGeom.Xform.Define(stage,'/World/Robot/'+name).GetPrim();body_ops.append(matrix(parent))
  b=source_bodies[name]
  for k,g in enumerate(b.findall('geom')):
   if (g.get('name') or '').startswith('jaw_'):continue
   typ=g.get('type','sphere');path=str(parent.GetPath())+f'/Geometry_{k}';size=list(map(float,g.get('size','.01').split()))
   if typ=='box':
    obj=UsdGeom.Cube.Define(stage,path);obj.CreateSizeAttr(2.);x=UsdGeom.Xformable(obj);x.AddTranslateOp().Set(Gf.Vec3d(*map(float,g.get('pos','0 0 0').split())));x.AddScaleOp().Set(Gf.Vec3f(*size))
   elif typ=='sphere':
    obj=UsdGeom.Sphere.Define(stage,path);obj.CreateRadiusAttr(size[0]);UsdGeom.Xformable(obj).AddTranslateOp().Set(Gf.Vec3d(*map(float,g.get('pos','0 0 0').split())))
   elif typ=='capsule':
    ft=np.array(list(map(float,g.get('fromto').split()))).reshape(2,3);v=ft[1]-ft[0];obj=UsdGeom.Capsule.Define(stage,path);obj.CreateRadiusAttr(size[0]);obj.CreateHeightAttr(float(np.linalg.norm(v)));matrix(obj.GetPrim()).Set(posemat(ft.mean(0),zquat(v)))
   else:raise ValueError(typ)
   bind(obj.GetPrim(),dark if name=='robot_body' else leg)
  if name.startswith('grip_'):
   j=int(name.split('_')[-1])
   for side in [-1,1]:
    jaw=UsdGeom.Cube.Define(stage,str(parent.GetPath())+f'/Jaw_{"L" if side<0 else "R"}');jaw.CreateSizeAttr(2.);x=UsdGeom.Xformable(jaw);tr=x.AddTranslateOp();x.AddScaleOp().Set((.004,.012,.016));bind(jaw.GetPrim(),green);jaw_ops.append((j,side,tr,jaw.GetPrim()))
 geom=json.loads((ROOT/'assets/geometry.json').read_text());p=json.loads((ROOT/'assets/parameters.json').read_text());edges=np.array(geom['edges'],int)-1
 rest_nodes=np.asarray(p['nodes']);rest_lengths=np.linalg.norm(rest_nodes[edges[:,1]]-rest_nodes[edges[:,0]],axis=1)
 lengths,prototype_indices=np.unique(np.round(rest_lengths,9),return_inverse=True)
 instancer=UsdGeom.PointInstancer.Define(stage,'/World/WalkingTruss');prototype_paths=[]
 for i,length in enumerate(lengths):
  bar=UsdGeom.Capsule.Define(stage,f'/World/WalkingTruss/Prototypes/Member_{i}');bar.CreateAxisAttr('Z');bar.CreateRadiusAttr(.006);bar.CreateHeightAttr(float(length));bind(bar.GetPrim(),steel);prototype_paths.append(bar.GetPath())
 instancer.CreatePrototypesRel().SetTargets(prototype_paths);instancer.CreateProtoIndicesAttr(prototype_indices.astype(np.int32))
 positions=instancer.CreatePositionsAttr();orientations=instancer.CreateOrientationsAttr();scales=instancer.CreateScalesAttr()
 for name,pos,size in [('RootFlange',[-29,8,-1.48],[.20,.25,.18])]:
  cube=UsdGeom.Cube.Define(stage,'/World/ISSConnection/'+name);cube.CreateSizeAttr(2.);x=UsdGeom.Xformable(cube);x.AddTranslateOp().Set(Gf.Vec3d(*pos));x.AddScaleOp().Set(Gf.Vec3f(*size));bind(cube.GetPrim(),metals)
 for j,y in enumerate([7.2,8.8]):
  a=np.array([-29,y,-1.58]);b=np.array([-28.84,8,-1.15]);v=b-a;bar=UsdGeom.Capsule.Define(stage,f'/World/ISSConnection/Brace_{j}');bar.CreateRadiusAttr(.045);bar.CreateHeightAttr(float(np.linalg.norm(v)));matrix(bar.GetPrim()).Set(posemat((a+b)/2,zquat(v)));bind(bar.GetPrim(),metals)
 # Earth is a visual object at true radius/altitude; it has no collision or mass.
 lat,lon=np.radians([35,-5]);up=np.array([np.cos(lat)*np.cos(lon),np.cos(lat)*np.sin(lon),np.sin(lat)]);east=np.array([-np.sin(lon),np.cos(lon),0]);north=np.cross(up,east);basis=np.stack([east,north,up],axis=1)
 nlat,nlon=128,256;u,v=np.meshgrid(np.linspace(0,1,nlon+1),np.linspace(0,1,nlat+1));la=(.5-v)*np.pi;lo=(u-.5)*2*np.pi;norm=np.stack([np.cos(la)*np.cos(lo),np.cos(la)*np.sin(lo),np.sin(la)],-1).reshape(-1,3)@basis
 faces=[]
 for i in range(nlat):
  for j in range(nlon):a=i*(nlon+1)+j;faces.extend([[a,a+nlon+1,a+1],[a+1,a+nlon+1,a+nlon+2]])
 earth=UsdGeom.Mesh.Define(stage,'/World/Earth');earth.CreatePointsAttr((norm*6371000+[0,0,-6791000]).astype(np.float32));earth.CreateFaceVertexCountsAttr(np.full(len(faces),3));earth.CreateFaceVertexIndicesAttr(np.array(faces).ravel());earth.CreateSubdivisionSchemeAttr('none');earth.CreateDoubleSidedAttr(True)
 UsdGeom.PrimvarsAPI(earth).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex).Set(np.c_[u.ravel(),1-v.ravel()].astype(np.float32))
 em=material(stage,'NASAEarth',(0,0,0),0,1,texture=ROOT/'assets/earth_composite.jpg',emissive=True);bind(earth.GetPrim(),em)
 light=UsdLux.DistantLight.Define(stage,'/World/Sun');light.CreateIntensityAttr(1500);light.CreateAngleAttr(.53);UsdGeom.Xformable(light).AddRotateXYZOp().Set((35,-20,-20));dome=UsdLux.DomeLight.Define(stage,'/World/EarthFill');dome.CreateIntensityAttr(5);dome.CreateColorAttr((.35,.45,.65))
 camera=UsdGeom.Camera.Define(stage,'/World/Camera');camera.CreateHorizontalApertureAttr(36);camera.CreateVerticalApertureAttr(20.25);camera.CreateFocalLengthAttr(float(20.25/(2*np.tan(np.radians(22.5)))));camera.CreateClippingRangeAttr((.01,20000000));camera_op=matrix(camera.GetPrim())
 return {'bodies':body_ops,'instances':(positions,orientations,scales),'rest_lengths':rest_lengths,'edges':edges,'last_grips':np.full(4,-1.),'p':p,'jaws':jaw_ops,'green':green,'orange':orange,'camera':camera_op}

def zquat(v):
 v=v/np.linalg.norm(v);q=np.r_[-v[1],v[0],0.,1+v[2]];return q/np.linalg.norm(q) if np.linalg.norm(q)>1e-10 else np.array([1,0,0,0])
def update(handles,row,poses):
 for op,pose in zip(handles['bodies'],poses):op.Set(posemat(pose[:3]+OFFSET,pose[3:7]))
 p=handles['p'];nodes=np.array(p['nodes'])+(np.array(p['fullModes'])@row[19:35]).reshape(96,3)+OFFSET
 edge=handles['edges'];vectors=nodes[edge[:,1]]-nodes[edge[:,0]];lengths=np.linalg.norm(vectors,axis=1);directions=vectors/lengths[:,None]
 quaternions=np.c_[-directions[:,1],directions[:,0],np.zeros(len(edge)),1+directions[:,2]];norms=np.linalg.norm(quaternions,axis=1);special=norms<1e-10;quaternions[~special]/=norms[~special,None];quaternions[special]=[1,0,0,0]
 position_attr,orientation_attr,scale_attr=handles['instances'];position_attr.Set(((nodes[edge[:,0]]+nodes[edge[:,1]])/2).astype(np.float32));orientation_attr.Set(Vt.QuathArray([Gf.Quath(float(q[3]),Gf.Vec3h(*map(float,q[:3]))) for q in quaternions]));scale_attr.Set(np.c_[np.ones((len(edge),2)),lengths/handles['rest_lengths']].astype(np.float32))
 for j,side,op,prim in handles['jaws']:
  op.Set((side*(.024 if row[74+j] else .038),0,-.001))
  if handles['last_grips'][j]!=row[74+j]:bind(prim,handles['green'] if row[74+j] else handles['orange'])
 handles['last_grips'][:]=row[74:78]

def camera(handles,t):
 side=OFFSET+np.array([3,0,.28]);keys=[(0,150,112,-28,[-9,0,0]),(3,150,95,-25,[-9,0,0]),(7,11,120,-28,OFFSET+[2.5,0,.1]),(10,7,105,-18,side),(12,5.6,90,-12,side),(136,5.6,90,-12,side)]
 for a,b in zip(keys,keys[1:]):
  if t<=b[0]:
   u=np.clip((t-a[0])/(b[0]-a[0]),0,1);u=u*u*(3-2*u);distance=np.exp(np.log(a[1])*(1-u)+np.log(b[1])*u);az=np.radians(a[2]*(1-u)+b[2]*u);el=np.radians(a[3]*(1-u)+b[3]*u);target=np.array(a[4])*(1-u)+np.array(b[4])*u;eye=target-distance*np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)]);handles['camera'].Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse());break
