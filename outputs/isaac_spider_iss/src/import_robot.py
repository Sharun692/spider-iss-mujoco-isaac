"""Rebuild the native USD robot using the tested Isaac 5.1 MJCF import configuration."""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'python_deps'))
import scipy.io
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'multi_gpu': False, 'max_gpu_count': 1})
try:
    import omni.kit.commands
    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.asset.importer.mjcf')
    ok, config = omni.kit.commands.execute('MJCFCreateImportConfig')
    if not ok:
        raise RuntimeError('MJCF import configuration failed')
    config.merge_fixed_joints = False
    config.import_inertia_tensor = True
    config.fix_base = False
    config.distance_scale = 1.
    ok, path = omni.kit.commands.execute('MJCFCreateAsset', mjcf_path=str(ROOT/'bridge_mjcf.xml'), import_config=config, prim_path='/World/Spider')
    if not ok:
        raise RuntimeError('MJCF import failed')
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath('/World/Spider/robot_body/robot_body').IsValid():
        raise RuntimeError('Expected articulation root missing')
    stage.GetRootLayer().Export(str(ROOT/'imported_spider.usda'))
    print('IMPORTED', path, flush=True)
finally:
    app.close()
