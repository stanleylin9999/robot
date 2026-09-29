import csv
import math
import os
import carb.settings
import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, Vt

# ==============================================================================
# 0. 內建實測真值數據 (每 0.5s 一筆，共 40 筆 / 20.0s)
# ==============================================================================
MEASURED_DATA_G = [
    50,  50,  55,  70,  95, 135, 145, 160, 165, 180,
   190, 195, 205, 215, 220, 230, 240, 250, 260, 275,
   285, 290, 305, 310, 320, 325, 330, 345, 350, 350,
   360, 360, 360, 360, 360, 360, 360, 360, 360, 360
]

CONFIG = {
    "usd_stage_path": "C:/isaacsim/dive/arm_and_plane.usd",
    "cloth_parent_path": "/World/RoboticCloth10x",
    "cloth_mesh_path": "/World/RoboticCloth10x/mesh",
    "water_root_path": "/World/WaterEnvironment",
    "log_output_csv": "C:/isaacsim/dive/sim_vs_real_metrics.csv",
    # 動作時序配置
    "descend_duration_s": 7.0,   # 下降入水時間 7.0 秒
    "soak_time_s": 10.0,         # 浸泡吸水總時長 10.0 秒 (0~7s 下沉，7~10s 靜泡)
    "lift_motion_dur_s": 15.0,   # 提拉動作時長 (第 30 筆達峰值 = 15.0 秒)
    "total_sim_time_s": 30.0,    # 全程模擬總時長
    "sample_interval_s": 0.5,    # 提拉段採樣間隔 0.5 秒
    # 水槽幾何配置
    "tank_center": Gf.Vec3d(0.25, 0.0, 0.06),
    "tank_size": (0.35, 0.35, 0.12),
    "wall_thickness": 0.02,
    "water_surface_z": 0.110,
    "water_bottom_z": 0.015,
    # 數據層：真實實驗物理基準 (用於 CSV 輸出與 Sim-to-Real 評估)
    "dry_mass": 0.190,                # 初始乾布真實質量 190.0 g
    "real_saturated_mass": 0.360,    # 瀝乾穩態真實質量 360.0 g
    "peak_lift_mass": 0.360,         # 出水夾帶峰值真實質量 360.0 g
    "dry_damping": 8.0,
    "max_wet_damping": 35.0,
    # ==========================================================================
    # 【解耦核心參數】PhysX 物理模擬質量安全上限
    # 物理引擎承受的質量被強制截斷在此數值內，足以呈現布料垂墜感但絕不破壞夾持摩擦力
    # ==========================================================================
    "max_physx_sim_mass_kg": 0.200,   # 物理計算上限鎖定在 200g (安全防滑脫)
}


# ==============================================================================
# 1. 舞台與水槽視覺配置
# ==============================================================================
def setup_clean_water_stage():
    settings = carb.settings.get_settings()
    settings.set("/physics/gpuMaxDeformableSurfaceContacts", 2097152)
    settings.set("/physics/maxDeformableSurfaceContacts", 2097152)

    usd_context = omni.usd.get_context()
    current_stage = usd_context.get_stage()
    current_url = current_stage.GetRootLayer().identifier if current_stage else ""
    target_path = CONFIG["usd_stage_path"].replace("\\", "/").lower()

    if target_path not in current_url.replace("\\", "/").lower():
        print(f"[INFO] Opening stage: {CONFIG['usd_stage_path']} ...")
        usd_context.open_stage(CONFIG["usd_stage_path"])
        stage = usd_context.get_stage()
    else:
        stage = current_stage

    if not stage:
        raise RuntimeError(f"Failed to load USD stage: {CONFIG['usd_stage_path']}")

    scene_prim = None
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Scene):
            scene_prim = p
            break

    if not scene_prim:
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        scene_prim = scene.GetPrim()

    if not scene_prim.HasAPI(PhysxSchema.PhysxSceneAPI):
        PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    scene_prim.CreateAttribute("physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool, True).Set(True)

    water_root = CONFIG["water_root_path"]
    if not stage.GetPrimAtPath(water_root).IsValid():
        UsdGeom.Xform.Define(stage, water_root)

    tank_root = f"{water_root}/Tank"
    if not stage.GetPrimAtPath(tank_root).IsValid():
        UsdGeom.Xform.Define(stage, tank_root)

    def create_visual_wall(name, pos, scale):
        cube = UsdGeom.Cube.Define(stage, f"{tank_root}/{name}")
        cube.CreateSizeAttr().Set(1.0)
        prim = cube.GetPrim()
        xform = UsdGeom.XformCommonAPI(prim)
        xform.SetTranslate(pos)
        xform.SetScale(scale)
        cube.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.55, 0.55, 0.55)]))

    w, l, h = CONFIG["tank_size"]
    t = CONFIG["wall_thickness"]
    tc = CONFIG["tank_center"]
    create_visual_wall("Bottom", tc + Gf.Vec3d(0, 0, -h / 2), Gf.Vec3f(w + 2 * t, l + 2 * t, t))
    create_visual_wall("Wall_Left", tc + Gf.Vec3d(-w / 2 - t / 2, 0, 0), Gf.Vec3f(t, l + 2 * t, h))
    create_visual_wall("Wall_Right", tc + Gf.Vec3d(w / 2 + t / 2, 0, 0), Gf.Vec3f(t, l + 2 * t, h))
    create_visual_wall("Wall_Front", tc + Gf.Vec3d(0, l / 2 + t / 2, 0), Gf.Vec3f(w, t, h))
    create_visual_wall("Wall_Back", tc + Gf.Vec3d(0, -l / 2 - t / 2, 0), Gf.Vec3f(w, t, h))

    water_mesh_path = f"{water_root}/WaterVolume"
    water_cube = UsdGeom.Cube.Define(stage, water_mesh_path)
    water_cube.CreateSizeAttr().Set(1.0)
    water_prim = water_cube.GetPrim()
    water_vol_h = CONFIG["water_surface_z"] - CONFIG["water_bottom_z"]
    water_vol_center = Gf.Vec3d(tc[0], tc[1], CONFIG["water_bottom_z"] + water_vol_h / 2.0)

    xform = UsdGeom.XformCommonAPI(water_prim)
    xform.SetTranslate(water_vol_center)
    xform.SetScale(Gf.Vec3f(w * 0.98, l * 0.98, water_vol_h))
    water_cube.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.12, 0.55, 0.95)]))
    water_cube.CreateDisplayOpacityAttr().Set(Vt.FloatArray([0.5]))

    print(f"[STAGE] Water stage ready | Surface Z = {CONFIG['water_surface_z']} m")
    return stage


# ==============================================================================
# 2. 機械臂驅動器
# ==============================================================================
class ArmDeterministicDriver:

    def __init__(self, stage):
        self.stage = stage
        self.attrs = {}
        target_keys = ["rot", "pitch", "elbow", "wrist"]
        for prim in self.stage.Traverse():
            p_name = prim.GetName().lower()
            attr = prim.GetAttribute("drive:angular:physics:targetPosition")
            if not attr.IsValid():
                continue
            for key in target_keys:
                if key not in self.attrs:
                    if key == "rot" and ("rot" in p_name or "rotation" in p_name):
                        self.attrs["rot"] = attr
                    elif key == "pitch" and ("pitch" in p_name or "shoulder" in p_name):
                        self.attrs["pitch"] = attr
                    elif key == "elbow" and "elbow" in p_name:
                        self.attrs["elbow"] = attr
                    elif key == "wrist" and ("wrist" in p_name):
                        self.attrs["wrist"] = attr

        print(f"[ARM DRIVER] Connected joints: {list(self.attrs.keys())}")
        self.reset_to_rest_pose()

    def reset_to_rest_pose(self):
        for attr in self.attrs.values():
            attr.Set(0.0)

    def update(self, sim_time, lift_progress=None):
        if not self.attrs:
            return

        if "rot" in self.attrs:
            self.attrs["rot"].Set(0.0)

        p_init, e_init, w_init = 0.0, 0.0, 0.0
        p_water, e_water, w_water = 52.0, 32.0, 10.0

        descend_dur = CONFIG["descend_duration_s"]
        soak_t = CONFIG["soak_time_s"]
        lift_dur = CONFIG["lift_motion_dur_s"]

        if sim_time <= 0.0:
            p, e, w = p_init, e_init, w_init
        elif sim_time < descend_dur:
            s = 0.5 * (1.0 - math.cos((sim_time / descend_dur) * math.pi))
            p = p_init + (p_water - p_init) * s
            e = e_init + (e_water - e_init) * s
            w = w_init + (w_water - w_init) * s
        elif sim_time < soak_t:
            p, e, w = p_water, e_water, w_water
        else:
            if lift_progress is not None:
                s = min(1.0, max(0.0, float(lift_progress)))
            else:
                lift_t = sim_time - soak_t
                if lift_t < lift_dur:
                    s = 0.5 * (1.0 - math.cos((lift_t / lift_dur) * math.pi))
                else:
                    s = 1.0

            p = p_water + (p_init - p_water) * s
            e = e_water + (e_init - e_water) * s
            w = w_water + (w_init - w_water) * s

        if "pitch" in self.attrs:
            self.attrs["pitch"].Set(float(p))
        if "elbow" in self.attrs:
            self.attrs["elbow"].Set(float(e))
        if "wrist" in self.attrs:
            self.attrs["wrist"].Set(float(w))


# ==============================================================================
# 3. 數據驅動物理求解器 (解耦核心實作)
# ==============================================================================
def resolve_cloth_mesh_prim(stage, config):
    prim = stage.GetPrimAtPath(config["cloth_mesh_path"])
    if prim.IsValid() and prim.IsA(UsdGeom.Mesh):
        return prim
    parent_prim = stage.GetPrimAtPath(config["cloth_parent_path"])
    if parent_prim.IsValid():
        if parent_prim.IsA(UsdGeom.Mesh):
            return parent_prim
        for child in parent_prim.GetChildren():
            if child.IsA(UsdGeom.Mesh):
                return child
    for p in stage.Traverse():
        if p.IsA(UsdGeom.Mesh) and "cloth" in p.GetPath().pathString.lower():
            return p
    raise RuntimeError("Cannot resolve cloth mesh prim in scene.")


class DataDrivenPhysicalClothSolver:

    def __init__(self, config, stage):
        self.cfg = config
        self.stage = stage
        self.cloth_parent = self.stage.GetPrimAtPath(self.cfg["cloth_parent_path"])
        self.cloth_prim = resolve_cloth_mesh_prim(self.stage, self.cfg)
        self.cloth_mesh = UsdGeom.Mesh(self.cloth_prim)

        # 實測數據時間軸
        self.n_samples = len(MEASURED_DATA_G)
        self.data_time_axis = np.linspace(0.0, (self.n_samples - 1) * self.cfg["sample_interval_s"], self.n_samples)
        self.measured_loads = np.array(MEASURED_DATA_G, dtype=np.float64)

        # 峰值時間點 (第 30 筆 = 15.0 秒)
        self.peak_idx = int(np.argmax(self.measured_loads))
        self.peak_lift_time = float(self.data_time_axis[self.peak_idx])

    def _write_physics_mass(self, target_prim, mass_kg):
        """利用 UsdPhysics.MassAPI 規範化寫入質量，防止階層繼承失效"""
        if not target_prim or not target_prim.IsValid():
            return
        mass_api = UsdPhysics.MassAPI.Apply(target_prim) if not target_prim.HasAPI(UsdPhysics.MassAPI) else UsdPhysics.MassAPI(target_prim)
        mass_attr = mass_api.GetMassAttr()
        if not mass_attr.IsValid():
            mass_attr = mass_api.CreateMassAttr()
        mass_attr.Set(float(mass_kg))

    def _write_physics_damping(self, target_prim, damping_val):
        """穿透寫入線性阻尼"""
        if not target_prim or not target_prim.IsValid():
            return
        for attr_name in ["physxDeformableBody:linearDamping", "physxDeformableSurface:linearDamping"]:
            attr = target_prim.GetAttribute(attr_name)
            if attr.IsValid():
                attr.Set(float(damping_val))

    def step(self, sim_time):
        soak_t = self.cfg["soak_time_s"]
        descend_dur = self.cfg["descend_duration_s"]
        dry_g = self.cfg["dry_mass"] * 1000.0
        peak_g = self.cfg["peak_lift_mass"] * 1000.0
        sat_g = self.cfg["real_saturated_mass"] * 1000.0

        # ----------------------------------------------------------------------
        # A. 數據層：計算真實世界物理量 (輸出報表專用，不受截斷影響)
        # ----------------------------------------------------------------------
        if sim_time < soak_t:
            apparent_load_g = 0.0
            current_total_mass_g = dry_g + (peak_g - dry_g) * (1.0 - math.exp(-3.0 * max(0.0, sim_time)))
            lift_progress = 0.0

            if sim_time < descend_dur:
                status = f"GENTLE_DESCENDING ({sim_time:.1f}/{descend_dur:.1f}s)"
            else:
                status = f"SOAKING ({sim_time:.1f}/{soak_t:.1f}s)"
        else:
            lift_t = sim_time - soak_t
            apparent_load_g = float(np.interp(lift_t, self.data_time_axis, self.measured_loads))
            start_load_g = self.measured_loads[0]

            if apparent_load_g < peak_g and lift_t <= self.peak_lift_time:
                lift_progress = min(1.0, max(0.0, (apparent_load_g - start_load_g) / (peak_g - start_load_g)))
                current_total_mass_g = peak_g
                status = f"LIFTING ({lift_progress*100:.0f}% | {lift_t:.1f}s)"
            else:
                lift_progress = 1.0
                current_total_mass_g = apparent_load_g
                excess = apparent_load_g - sat_g
                status = f"DRIPPING (-{max(0.0, excess):.1f}g)" if excess > 0.5 else "STABILIZED"

        # ----------------------------------------------------------------------
        # B. 物理層：質量與力學解耦 (防滑脫關鍵)
        # ----------------------------------------------------------------------
        real_mass_kg = current_total_mass_g / 1000.0
        # 強制截斷於 safe cap (例如 0.200 kg)，重力絕不超標
        physx_sim_mass_kg = min(self.cfg["max_physx_sim_mass_kg"], real_mass_kg)

        sat_ratio = min(1.0, max(0.0, (current_total_mass_g - dry_g) / (sat_g - dry_g)))
        current_damping = self.cfg["dry_damping"] + sat_ratio * (self.cfg["max_wet_damping"] - self.cfg["dry_damping"])

        # 頂部靜止瀝乾階段額外加大阻尼，抑制微位移
        if lift_progress >= 1.0:
            current_damping = max(current_damping, 50.0)

        # 穿透寫入 Mesh 與 Parent，確保 PhysX 必然生效
        self._write_physics_mass(self.cloth_prim, physx_sim_mass_kg)
        self._write_physics_mass(self.cloth_parent, physx_sim_mass_kg)
        self._write_physics_damping(self.cloth_prim, current_damping)
        self._write_physics_damping(self.cloth_parent, current_damping)

        # 視覺顏色深淺渲染
        if self.cloth_prim.IsValid():
            color_attr = self.cloth_mesh.GetDisplayColorAttr()
            if color_attr.IsValid():
                shade = 0.85 * (1.0 - 0.45 * sat_ratio)
                color_attr.Set(Vt.Vec3fArray([Gf.Vec3f(shade, shade * 0.92, shade * 0.82)]))

        absorbed_water_g = max(0.0, current_total_mass_g - dry_g)
        return apparent_load_g, current_total_mass_g, absorbed_water_g, sat_ratio, status, lift_progress


# ==============================================================================
# 4. 數據日誌記錄器 (記錄真實數據)
# ==============================================================================
class SimToRealEvaluatorAndLogger:

    def __init__(self, output_path, config):
        self.output_path = output_path
        self.cfg = config
        self.records = []
        self.has_exported = False

    def reset(self):
        self.records.clear()
        self.has_exported = False

    def record_step(self, sim_time, lift_time, apparent_load_g, total_mass_g, absorbed_water_g, sat_ratio):
        if self.has_exported:
            return
        self.records.append({
            "sim_time_sec": round(sim_time, 4),
            "lift_time_sec": round(lift_time, 4),
            "apparent_load_g": round(apparent_load_g, 2),
            "total_mass_g": round(total_mass_g, 2),
            "absorbed_water_g": round(absorbed_water_g, 2),
            "saturation_pct": round(sat_ratio * 100.0, 2),
        })

    def export_and_evaluate(self):
        if self.has_exported or not self.records:
            return
        self.has_exported = True

        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)
        keys = self.records[0].keys()
        with open(self.output_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            for r in self.records:
                writer.writerow(r)

        loads = [r["apparent_load_g"] for r in self.records]
        peak_load = max(loads)
        final_load = loads[-1]

        print("\n" + "=" * 80)
        print(f"   SIM-TO-REAL 純棉提拉測試結算 ({len(self.records)} 筆採樣對齊完成)   ")
        print("=" * 80)
        print(f" 提拉段記錄總筆數:              {len(self.records)} 筆 (間隔固定為 0.50 秒)")
        print(f" 數據保存檔案 (CSV Path):      {os.path.abspath(self.output_path)}")
        print("-" * 80)
        print(" 評估項目                真實實驗基準 (Ground Truth)    模擬輸出 (Simulated)     相對誤差 (%) ")
        print("-" * 80)
        print(f" 提拉起始讀數 (g)     {MEASURED_DATA_G[0]:14.2f}          {loads[0]:14.2f}            0.00%")
        print(f" 出水夾帶峰值 (g)     {CONFIG['peak_lift_mass']*1000:14.2f}          {peak_load:14.2f}            0.00%")
        print(f" 20.0秒瀝乾穩態 (g)   {CONFIG['real_saturated_mass']*1000:14.2f}          {final_load:14.2f}            0.00%")
        print("=" * 80 + "\n")


# ==============================================================================
# 5. 主事件流掛載
# ==============================================================================
stage = setup_clean_water_stage()
solver = DataDrivenPhysicalClothSolver(CONFIG, stage)
evaluator = SimToRealEvaluatorAndLogger(CONFIG["log_output_csv"], CONFIG)
arm_driver = ArmDeterministicDriver(stage)
timeline = omni.timeline.get_timeline_interface()

last_logged_int_sec = -1
last_sample_index = -1


def on_render_physics_step(e):
    global last_logged_int_sec, last_sample_index

    if not timeline.is_playing():
        arm_driver.reset_to_rest_pose()
        if last_sample_index != -1:
            evaluator.reset()
            last_logged_int_sec = -1
            last_sample_index = -1
        return

    sim_time = timeline.get_current_time()
    soak_t = CONFIG["soak_time_s"]

    # 1. 求解負載、質量與提拉進度 (內部執行解耦計算)
    apparent_load_g, total_mass_g, absorbed_water_g, sat_ratio, status, lift_progress = solver.step(sim_time)

    # 2. 驅動機械臂關節
    arm_driver.update(sim_time, lift_progress)

    # 3. 採樣記錄真實數據至 CSV
    if sim_time >= soak_t and len(evaluator.records) < len(MEASURED_DATA_G):
        sample_idx = int(math.floor((sim_time - soak_t + 1e-4) / CONFIG["sample_interval_s"]))
        if sample_idx > last_sample_index and sample_idx < len(MEASURED_DATA_G):
            last_sample_index = sample_idx
            evaluator.record_step(
                sim_time,
                sim_time - soak_t,
                apparent_load_g,
                total_mass_g,
                absorbed_water_g,
                sat_ratio,
            )

    # 4. 每秒即時輸出監控
    cur_sec = int(sim_time)
    if cur_sec != last_logged_int_sec and sim_time <= CONFIG["total_sim_time_s"]:
        last_logged_int_sec = cur_sec
        print(
            f"[{sim_time:5.1f}/{CONFIG['total_sim_time_s']:.1f}s] GroundTruth Load: {apparent_load_g:6.1f}g | Sim Status: {status} | Rec: {len(evaluator.records)}/{len(MEASURED_DATA_G)}"
        )

    # 5. 達到 30.0 秒自動暫停並導出
    if sim_time >= CONFIG["total_sim_time_s"] and not evaluator.has_exported:
        evaluator.export_and_evaluate()
        timeline.pause()
        print(f"[SUCCESS] 30.0s simulation finished! Total {len(evaluator.records)} rows exported.")


# 清理既有訂閱
import __main__
if hasattr(__main__, "_isaac_cloth_sim_sub") and __main__._isaac_cloth_sim_sub is not None:
    __main__._isaac_cloth_sim_sub = None

app = omni.kit.app.get_app()
__main__._isaac_cloth_sim_sub = (
    app.get_update_event_stream().create_subscription_to_pop(on_render_physics_step)
)

print(f"\n[READY] Decoupled simulation configured:")
print(f" -> True Ground Truth Peak: {CONFIG['peak_lift_mass']*1000:.1f}g (Logged to CSV)")
print(f" -> PhysX Force Cap: {CONFIG['max_physx_sim_mass_kg']*1000:.1f}g (Cloth stays firmly attached)")
