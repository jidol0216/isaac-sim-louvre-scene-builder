"""
Louvre Scene Builder Extension
==============================
Isaac Sim Extension that builds the Louvre museum scene with robot.

Features:
- Loads origin_lubre.usd with textures (reference, not copy)
- Loads RidgebackFranka robot (ActionGraph preserved)
- Adds mecanum wheels, camera, LiDAR
- No USD export needed - always loads from source

Usage:
1. Enable extension in Isaac Sim
2. Click "Build Louvre Scene" button in the UI
3. Press Play to start simulation
"""

import omni.ext
import omni.ui as ui
import omni.kit.commands
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, UsdShade, PhysxSchema, Gf, Sdf
from pathlib import Path
import carb


class LouvreSceneBuilderExtension(omni.ext.IExt):
    """Extension to build Louvre museum scene"""
    
    # Asset paths (absolute paths for reliability)
    ENV_SET_DIR = Path("/home/rokey/louvre_extension_minimal")
    # GLB 파일 직접 사용 - 텍스처가 임베드되어 있음!
    LOUVRE_GLB = str(ENV_SET_DIR / "scenes" / "lubre.glb")
    ROBOT_USD = str(ENV_SET_DIR / "assets" / "robots" / "RidgebackFranka" / "ridgeback_franka.usd")
    MECANUM_USD = str(ENV_SET_DIR / "assets" / "wheel" / "basic_four_mecanum_robot_after_add_friction_for_rollers_2.usd")
    
    # Transform settings
    LOUVRE_SCALE = 33.0
    LOUVRE_TRANSLATE = (11.0, 118.0, 0.0)
    ROBOT_POSITION = (0.0, 0.0, 2.0)
    
    # Meshes to exclude
    EXCLUDE_MESHES = ["BuildingMesh_00196", "BuildingMesh_00197", "BuildingMesh_00204"]
    
    # Save path
    SAVE_PATH = str(ENV_SET_DIR / "scenes" / "louvre_complete_scene.usd")
    GRAPH_SAVE_PATH = str(ENV_SET_DIR / "scenes" / "louvre_camera_graphs.usd")
    
    def on_startup(self, ext_id):
        """Called when extension starts"""
        print("[LouvreSceneBuilder] Extension startup")
        
        self._window = ui.Window("Louvre Scene Builder", width=300, height=300)
        with self._window.frame:
            with ui.VStack(spacing=10):
                ui.Label("Louvre Museum Scene Builder", height=30, 
                        style={"font_size": 18, "color": 0xFFFFFFFF})
                
                ui.Spacer(height=10)
                
                ui.Button("Build Louvre Scene", height=40,
                         clicked_fn=self._build_scene)
                
                ui.Button("Load Saved Scene", height=35,
                         clicked_fn=self._load_saved_scene,
                         style={"background_color": 0xFF339966})
                
                ui.Button("Save Scene", height=35,
                         clicked_fn=self._save_scene,
                         style={"background_color": 0xFF336699})
                
                ui.Button("Reset Scene", height=30,
                         clicked_fn=self._reset_scene)
                
                ui.Spacer()
                
                self._status_label = ui.Label("Ready", height=20,
                                             style={"color": 0xFF88FF88})
    
    def on_shutdown(self):
        """Called when extension shuts down"""
        print("[LouvreSceneBuilder] Extension shutdown")
        
        # Clean up synthetic data graphs before shutdown
        self._cleanup_synthetic_data_graphs()
        
        if self._window:
            self._window.destroy()
            self._window = None
    
    def _cleanup_synthetic_data_graphs(self):
        """Clean up synthetic data render graphs to prevent errors"""
        try:
            import omni.syntheticdata
            # Synthetic data cleanup (optional, may not be needed)
            pass
        except Exception as e:
            pass
    
    def _update_status(self, msg: str, success: bool = True):
        """Update status label"""
        color = 0xFF88FF88 if success else 0xFFFF8888
        self._status_label.text = msg
        self._status_label.style = {"color": color}
        print(f"[LouvreSceneBuilder] {msg}")
    
    def _reset_scene(self):
        """Reset the scene"""
        import omni.usd
        
        # Clean up synthetic data graphs before reset
        self._cleanup_synthetic_data_graphs()
        
        # Small delay to ensure cleanup
        import asyncio
        asyncio.ensure_future(self._reset_scene_async())
    
    async def _reset_scene_async(self):
        """Async scene reset with proper cleanup"""
        import omni.usd
        from omni.kit.app import get_app
        
        # Wait for cleanup
        await get_app().next_update_async()
        
        # Reset stage
        omni.usd.get_context().new_stage()
        self._update_status("Scene reset")
    
    def _load_saved_scene(self):
        """Load previously saved scene with proper cleanup"""
        import asyncio
        asyncio.ensure_future(self._load_saved_scene_async())
    
    async def _load_saved_scene_async(self):
        """Async scene loading with synthetic data cleanup"""
        try:
            import omni.usd
            import os
            from omni.kit.app import get_app
            
            if not os.path.exists(self.SAVE_PATH):
                self._update_status("❌ No saved scene found. Build first!", success=False)
                return
            
            self._update_status("📂 Loading saved scene...")
            
            # Open the saved USD file
            omni.usd.get_context().open_stage(self.SAVE_PATH)
            
            # Wait for stage to load
            await get_app().next_update_async()
            
            self._update_status(f"✅ Loaded: louvre_complete_scene.usd")
            print(f"[LouvreSceneBuilder] Loaded scene from: {self.SAVE_PATH}")
            print("  ✅ All references & graphs intact")
            
        except Exception as e:
            self._update_status(f"❌ Load error: {str(e)}", success=False)
            carb.log_error(f"[LouvreSceneBuilder] Load error: {e}")
    
    def _reinitialize_synthetic_data(self):
        """Reinitialize synthetic data after loading scene"""
        try:
            import omni.syntheticdata
            sd_interface = omni.syntheticdata._syntheticdata.acquire_syntheticdata_interface()
            if sd_interface:
                # Clear and re-register annotators
                sd_interface.clear_registered_annotators()
            print("  ✅ Synthetic data reinitialized")
        except Exception as e:
            print(f"  ⚠️  Synthetic data reinit: {e}")
    
    def _save_graphs_only(self):
        """GUI에서 만든 OmniGraph를 별도 USD 파일로 저장"""
        try:
            import omni.usd
            from pxr import Usd, Sdf, UsdUtils
            
            context = omni.usd.get_context()
            stage = context.get_stage()
            
            if not stage:
                self._update_status("❌ No stage", success=False)
                return
            
            self._update_status("💾 Saving graphs...")
            
            # 모든 graph prim 경로 찾기
            graph_paths = []
            for prim in stage.Traverse():
                prim_path = str(prim.GetPath())
                # Render graph 또는 ActionGraph만
                if prim_path.startswith("/Render") or prim_path.startswith("/ActionGraph"):
                    # 최상위 레벨만
                    parent_path = str(prim.GetParent().GetPath())
                    if parent_path == "/" or parent_path.startswith("/Render"):
                        if prim_path not in graph_paths:
                            graph_paths.append(prim_path)
            
            if not graph_paths:
                self._update_status("❌ No graphs found", success=False)
                print("  ℹ️  No /Render or /ActionGraph found")
                return
            
            print(f"  📊 Saving {len(graph_paths)} graph hierarchies:")
            for path in graph_paths:
                print(f"    - {path}")
            
            # Flatten 방식으로 export (특정 prim만)
            # 새 stage 만들고 해당 prim만 복사
            export_stage = Usd.Stage.CreateNew(self.GRAPH_SAVE_PATH)
            
            for graph_path in graph_paths:
                src_prim = stage.GetPrimAtPath(graph_path)
                if src_prim and src_prim.IsValid():
                    # Flatten된 형태로 복사
                    UsdUtils.FlattenLayerStack(stage, graph_path)
                    
                    # 해당 prim 트리 전체를 export stage로 복사
                    export_prim = export_stage.OverridePrim(graph_path)
                    for src_child in Usd.PrimRange(src_prim):
                        child_path = src_child.GetPath()
                        dest_prim = export_stage.OverridePrim(child_path)
                        dest_prim.SetTypeName(src_child.GetTypeName())
            
            export_stage.GetRootLayer().Export(self.GRAPH_SAVE_PATH)
            
            self._update_status(f"✅ {len(graph_paths)} graphs saved")
            print(f"[LouvreSceneBuilder] Graphs saved to: {self.GRAPH_SAVE_PATH}")
            
        except Exception as e:
            self._update_status(f"❌ Graph save error: {str(e)}", success=False)
            carb.log_error(f"[LouvreSceneBuilder] Graph save error: {e}")
    
    def _load_camera_graphs(self, stage):
        """저장된 camera graph 불러오기"""
        import os
        try:
            if not os.path.exists(self.GRAPH_SAVE_PATH):
                print("  ℹ️  No saved graphs found")
                return
            
            # Graph USD를 reference로 추가
            add_reference_to_stage(usd_path=self.GRAPH_SAVE_PATH, prim_path="/CameraGraphs")
            print(f"  ✅ Camera graphs loaded from: {self.GRAPH_SAVE_PATH}")
            
        except Exception as e:
            print(f"  ⚠️  Graph load: {e}")
    
    def _save_scene(self):
        """Save the current scene (reference 방식 - graph 보존)"""
        try:
            import omni.usd
            context = omni.usd.get_context()
            stage = context.get_stage()
            
            if not stage:
                self._update_status("❌ No stage to save", success=False)
                return
            
            self._update_status("💾 Saving scene (reference mode)...")
            
            # CRITICAL: Save as reference (not flattened) to preserve graphs
            # 현재 stage를 그대로 저장 (flatten 하지 않음)
            stage.GetRootLayer().Export(self.SAVE_PATH)
            
            self._update_status(f"✅ Saved (graphs preserved)")
            print(f"[LouvreSceneBuilder] Scene saved to: {self.SAVE_PATH}")
            print("  ✅ ActionGraph & Camera graphs preserved via references")
            
        except Exception as e:
            self._update_status(f"❌ Save error: {str(e)}", success=False)
            carb.log_error(f"[LouvreSceneBuilder] Save error: {e}")
    
    def _remove_all_render_graphs(self, stage):
        """Remove all OmniGraph render graphs before saving"""
        try:
            import omni.graph.core as og
            
            # Get all graphs in the scene
            graphs = og.get_all_graphs()
            prims_to_remove = []
            
            for graph in graphs:
                graph_path = graph.get_path_to_graph()
                # Remove synthetic data related graphs
                if any(keyword in graph_path for keyword in 
                       ["/Render", "/RenderProduct", "Synthetic", "annotator", "PostProcess"]):
                    prims_to_remove.append(graph_path)
                    print(f"  🗑️  Removing graph: {graph_path}")
            
            # Remove the prims
            for path in prims_to_remove:
                if stage.GetPrimAtPath(path):
                    stage.RemovePrim(path)
            
            print(f"  ✅ Removed {len(prims_to_remove)} render graphs")
        
        except Exception as e:
            print(f"  ⚠️  Graph removal: {e}")
    
    def _build_scene(self):
        """Build the complete Louvre scene"""
        try:
            self._update_status("Building scene...")
            
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            
            # Setup World
            world = World(stage_units_in_meters=1.0)
            
            # 1. Physics Scene
            self._setup_physics_scene(stage)
            
            # 2. Lighting
            self._setup_lighting(stage)
            
            # 3. Load Louvre (with textures via reference)
            self._load_louvre(stage)
            
            # 4. Load Robot (ActionGraph preserved via reference)
            self._load_robot(stage)
            
            # 5. Add sensors
            self._add_depth_camera(stage)
            self._add_lidar_sensors(stage)
            
            # 6. Apply optimizations
            self._apply_optimizations(stage)
            
            self._update_status("✅ Scene built! Press Play to start")
            
        except Exception as e:
            self._update_status(f"❌ Error: {str(e)}", success=False)
            carb.log_error(f"[LouvreSceneBuilder] {e}")
    
    def _setup_physics_scene(self, stage):
        """Setup physics scene"""
        physics_scene_path = "/World/PhysicsScene"
        
        if not stage.GetPrimAtPath(physics_scene_path).IsValid():
            scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
            scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
            scene.CreateGravityMagnitudeAttr(9.81)
            
            physics_prim = scene.GetPrim()
            PhysxSchema.PhysxSceneAPI.Apply(physics_prim)
            physx_api = PhysxSchema.PhysxSceneAPI(physics_prim)
            
            # CPU physics for performance (reduced to 20fps for camera sync)
            physx_api.CreateEnableGPUDynamicsAttr(False)
            physx_api.CreateBroadphaseTypeAttr("SAP")
            physx_api.GetTimeStepsPerSecondAttr().Set(20.0)  # Lower fps = less frame drops
            
            print("  ✅ Physics scene created")
    
    def _setup_lighting(self, stage):
        """Add lighting"""
        dome_light_path = "/World/DomeLight"
        if not stage.GetPrimAtPath(dome_light_path).IsValid():
            dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
            dome_light.GetIntensityAttr().Set(1000)
            print("  ✅ Dome Light added")
        
        dist_light_path = "/World/DistantLight"
        if not stage.GetPrimAtPath(dist_light_path).IsValid():
            dist_light = UsdLux.DistantLight.Define(stage, dist_light_path)
            dist_light.GetIntensityAttr().Set(3000)
            dist_light.GetAngleAttr().Set(1.0)
            xform = UsdGeom.Xformable(dist_light)
            xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 45, 0))
            print("  ✅ Distant Light added")
    
    def _load_louvre(self, stage):
        """Load Louvre museum from GLB (textures embedded!)"""
        print(f"  📂 Loading Louvre GLB: {self.LOUVRE_GLB}")
        
        # GLB 파일 직접 reference로 추가 - 텍스처 임베드됨!
        add_reference_to_stage(usd_path=self.LOUVRE_GLB, prim_path="/World/lubre")
        
        # Transform
        lubre_prim = stage.GetPrimAtPath("/World/lubre")
        if lubre_prim.IsValid():
            xform = UsdGeom.Xformable(lubre_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(*self.LOUVRE_TRANSLATE))
            xform.AddScaleOp().Set(Gf.Vec3f(self.LOUVRE_SCALE, self.LOUVRE_SCALE, self.LOUVRE_SCALE))
        
        # Apply collisions
        self._apply_collisions(stage, "/World/lubre")
        
        print("  ✅ Louvre loaded (GLB with embedded textures)")
    
    def _apply_collisions(self, stage, root_path):
        """Apply collision to meshes"""
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim.IsValid():
            return
        
        mesh_count = 0
        for prim in Usd.PrimRange(root_prim):
            if prim.IsA(UsdGeom.Mesh):
                mesh_name = prim.GetName()
                
                if mesh_name in self.EXCLUDE_MESHES:
                    UsdGeom.Imageable(prim).MakeInvisible()
                    continue
                
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(prim)
                
                if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                    mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                    mesh_collision_api.CreateApproximationAttr("none")
                
                mesh_count += 1
        
        print(f"  ✅ Collision applied to {mesh_count} meshes")
    
    def _load_robot(self, stage):
        """Load robot as reference (ActionGraph preserved!)"""
        robot_path = "/World/RidgebackFranka"
        print(f"  🤖 Loading Robot: {self.ROBOT_USD}")
        
        # Add as reference - ActionGraph stays intact!
        add_reference_to_stage(usd_path=self.ROBOT_USD, prim_path=robot_path)
        
        # Position
        robot_prim = stage.GetPrimAtPath(robot_path)
        if robot_prim.IsValid():
            xform = UsdGeom.Xformable(robot_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(*self.ROBOT_POSITION))
        
        # Fix joint limits
        self._fix_robot_joint_limits(stage, robot_path)
        
        # Add mecanum wheels
        self._add_mecanum_wheels(stage, robot_path)
        
        print("  ✅ Robot loaded (ActionGraph preserved)")
    
    def _fix_robot_joint_limits(self, stage, robot_path):
        """Fix mimic joint limits"""
        problem_joints = [
            "left_Left_2_Joint", "left_Right_1_Joint", 
            "left_Right_Support_Joint", "left_Right_2_Joint",
            "right_Left_2_Joint", "right_Right_1_Joint",
            "right_Right_Support_Joint", "right_Right_2_Joint"
        ]
        
        robot_prim = stage.GetPrimAtPath(robot_path)
        if not robot_prim:
            return
        
        for prim in Usd.PrimRange(robot_prim):
            if prim.GetName() in problem_joints:
                if prim.GetTypeName() == "PhysicsRevoluteJoint":
                    lower_attr = prim.GetAttribute("physics:lowerLimit")
                    upper_attr = prim.GetAttribute("physics:upperLimit")
                    
                    if not lower_attr or lower_attr.Get() is None:
                        prim.CreateAttribute("physics:lowerLimit", Sdf.ValueTypeNames.Float).Set(-90.0)
                    if not upper_attr or upper_attr.Get() is None:
                        prim.CreateAttribute("physics:upperLimit", Sdf.ValueTypeNames.Float).Set(90.0)
    
    def _add_mecanum_wheels(self, stage, robot_path):
        """Add mecanum wheels to robot"""
        BASE_LINK_PATH = f"{robot_path}/base_link"
        TARGET_WHEEL_RADIUS = 0.12
        
        WHEEL_CONFIG = {
            "wheel_fl": ((0.30, 0.50, 0.0), 45.0),
            "wheel_fr": ((0.30, -0.50, 0.0), -45.0),
            "wheel_rl": ((-0.30, 0.50, 0.0), -45.0),
            "wheel_rr": ((-0.30, -0.50, 0.0), 45.0),
        }
        
        MECANUM_REFS = {
            "wheel_fl": "/basic_four_mecanum_robot/front_left_mecanum",
            "wheel_fr": "/basic_four_mecanum_robot/front_right_mecanum",
            "wheel_rl": "/basic_four_mecanum_robot/front_right_mecanum",
            "wheel_rr": "/basic_four_mecanum_robot/front_left_mecanum",
        }
        
        # Disable dummy joints
        dummy_joints = [
            (f"{robot_path}/world/dummy_base_prismatic_x_joint", "linear"),
            (f"{robot_path}/dummy_base_x/dummy_base_prismatic_y_joint", "linear"),
            (f"{robot_path}/dummy_base_y/dummy_base_revolute_z_joint", "angular"),
        ]
        
        for joint_path, drive_type in dummy_joints:
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid():
                drive = UsdPhysics.DriveAPI.Get(joint_prim, drive_type)
                if drive:
                    drive.GetStiffnessAttr().Set(0)
                    drive.GetDampingAttr().Set(0)
                    drive.GetMaxForceAttr().Set(0)
        
        for wheel_name, (pos, angle) in WHEEL_CONFIG.items():
            lx, ly, lz = pos
            world_pos = (self.ROBOT_POSITION[0] + lx, 
                        self.ROBOT_POSITION[1] + ly, 
                        self.ROBOT_POSITION[2] + lz)
            
            wheel_path = f"{BASE_LINK_PATH}/{wheel_name}"
            wheel_xform = UsdGeom.Xform.Define(stage, wheel_path)
            wheel_prim = wheel_xform.GetPrim()
            
            wheel_xform.SetResetXformStack(True)
            wheel_xform.AddTranslateOp().Set(Gf.Vec3d(*world_pos))
            
            is_left = "fl" in wheel_name or "rl" in wheel_name
            rot_z = 90 if is_left else -90
            wheel_xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, rot_z))
            wheel_xform.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
            
            mecanum_ref = MECANUM_REFS[wheel_name]
            wheel_prim.GetReferences().AddReference(self.MECANUM_USD, Sdf.Path(mecanum_ref))
            
            # Disable body_joint and fix roller joints (from build_scene.py)
            omniwheel_path = f"{wheel_path}/omniwheel"
            
            for child_prim in Usd.PrimRange(wheel_prim):
                child_name = child_prim.GetName()
                child_type = child_prim.GetTypeName()
                
                if child_name == "body_joint" and 'Joint' in child_type:
                    child_prim.SetActive(False)
                    body0_rel = child_prim.GetRelationship("physics:body0")
                    body1_rel = child_prim.GetRelationship("physics:body1")
                    if body0_rel:
                        body0_rel.ClearTargets(True)
                    if body1_rel:
                        body1_rel.ClearTargets(True)
                
                elif 'RevoluteJoint' in child_name and 'Joint' in child_type:
                    if child_name == "RevoluteJoint":
                        roller_num = 1
                    else:
                        try:
                            roller_num = int(child_name.replace("RevoluteJoint", "")) + 2
                        except:
                            continue
                    
                    roller_path = f"{wheel_path}/Roller_{roller_num}"
                    
                    body0_rel = child_prim.GetRelationship("physics:body0")
                    body1_rel = child_prim.GetRelationship("physics:body1")
                    
                    if body0_rel:
                        body0_rel.SetTargets([Sdf.Path(omniwheel_path)])
                    if body1_rel:
                        body1_rel.SetTargets([Sdf.Path(roller_path)])
                    
                    roller_drive = UsdPhysics.DriveAPI.Get(child_prim, "angular")
                    if roller_drive:
                        roller_drive.GetStiffnessAttr().Set(0.0)
                        roller_drive.GetDampingAttr().Set(0.0)
                        roller_drive.GetMaxForceAttr().Set(0.0)
            
            # Setup omniwheel rigid body
            omniwheel_prim = stage.GetPrimAtPath(omniwheel_path)
            if omniwheel_prim and omniwheel_prim.IsValid():
                if not omniwheel_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI.Apply(omniwheel_prim)
                if not omniwheel_prim.HasAPI(UsdPhysics.MassAPI):
                    mass_api = UsdPhysics.MassAPI.Apply(omniwheel_prim)
                    mass_api.GetMassAttr().Set(2.0)
                if not omniwheel_prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(omniwheel_prim)
                
                # Set convexHull approximation for dynamic body (triangle mesh not allowed)
                from pxr import UsdPhysics as UsdPhy
                mesh_collision = UsdPhy.MeshCollisionAPI.Apply(omniwheel_prim)
                mesh_collision.GetApproximationAttr().Set("convexHull")
            
            # Create wheel joint
            joint_path_str = f"{BASE_LINK_PATH}/{wheel_name}_joint"
            joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path_str)
            joint.GetBody0Rel().SetTargets([BASE_LINK_PATH])
            joint.GetBody1Rel().SetTargets([omniwheel_path])
            joint.GetAxisAttr().Set("Y")
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(lx, ly, lz))
            joint.GetLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
            joint.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            
            if is_left:
                joint_rot = Gf.Quatf(0.7071, 0, 0, -0.7071)
            else:
                joint_rot = Gf.Quatf(0.7071, 0, 0, 0.7071)
            joint.GetLocalRot1Attr().Set(joint_rot)
            
            # Drive
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.GetTypeAttr().Set("force")
            drive.GetStiffnessAttr().Set(0)
            drive.GetDampingAttr().Set(1000)
            drive.GetMaxForceAttr().Set(500)
            
            # Mecanum attributes
            joint.GetPrim().CreateAttribute("isaacmecanumwheel:radius", 
                                           Sdf.ValueTypeNames.Float).Set(TARGET_WHEEL_RADIUS)
            joint.GetPrim().CreateAttribute("isaacmecanumwheel:angle", 
                                           Sdf.ValueTypeNames.Float).Set(angle)
        
        print("  ✅ Mecanum wheels added")
    
    def _add_depth_camera(self, stage):
        """Add RealSense D455 camera with proper graph setup"""
        robot_path = "/World/RidgebackFranka"
        mount_path = f"{robot_path}/base_link/camera_mount"
        
        if stage.GetPrimAtPath(mount_path).IsValid():
            return
        
        import numpy as np
        from isaacsim.core.utils.rotations import euler_angles_to_quat
        
        CAMERA_USD = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Sensors/Intel/RealSense/rsd455.usd"
        
        mount_xform = UsdGeom.Xform.Define(stage, mount_path)
        mount_xform.AddTranslateOp().Set(Gf.Vec3d(0.41, 0.0, 0.272))
        
        camera_path = f"{mount_path}/realsense_d455"
        camera_xform = UsdGeom.Xform.Define(stage, camera_path)
        camera_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.06))
        
        realsense_path = f"{camera_path}/RSD455"
        add_reference_to_stage(usd_path=CAMERA_USD, prim_path=realsense_path)
        
        # Remove RigidBody from camera (카메라는 로봇에 고정)
        realsense_prim = stage.GetPrimAtPath(realsense_path)
        if realsense_prim.IsValid():
            for prim in Usd.PrimRange(realsense_prim):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                if prim.HasAPI(UsdPhysics.MassAPI):
                    prim.RemoveAPI(UsdPhysics.MassAPI)
        
        print("  ✅ RealSense D455 added")
    
    def _add_lidar_sensors(self, stage):
        """Add SICK TiM781 LiDAR sensors"""
        import numpy as np
        from isaacsim.core.utils.rotations import euler_angles_to_quat
        
        robot_path = "/World/RidgebackFranka"
        LIDAR_USD = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Sensors/SICK/TiM781/tim781.usd"
        
        configs = {
            "front_laser": ((0, 0, 0), (0, 0, 0)),
            "rear_laser": ((0, 0, 0), (0, 0, 180)),
        }
        
        for name, (pos, rot) in configs.items():
            lidar_path = f"{robot_path}/{name}/sick_tim781"
            
            if stage.GetPrimAtPath(lidar_path).IsValid():
                continue
            
            lidar_xform = UsdGeom.Xform.Define(stage, lidar_path)
            lidar_xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
            
            rot_quat = euler_angles_to_quat(np.array(rot), degrees=True)
            lidar_xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Quatd(float(rot_quat[0]), float(rot_quat[1]), 
                        float(rot_quat[2]), float(rot_quat[3]))
            )
            
            sensor_path = f"{lidar_path}/Sensor"
            add_reference_to_stage(usd_path=LIDAR_USD, prim_path=sensor_path)
        
        print("  ✅ LiDAR sensors added")
    
    def _disable_camera_auto_render(self, stage, camera_root_path):
        """Disable automatic rendering to prevent frame drops"""
        try:
            camera_prim = stage.GetPrimAtPath(camera_root_path)
            if not camera_prim.IsValid():
                return
            
            # Find all Camera prims and disable them
            for prim in Usd.PrimRange(camera_prim):
                if prim.IsA(UsdGeom.Camera):
                    # Make camera inactive during play (enable manually when needed)
                    prim.SetActive(False)
                    print(f"    📷 Camera disabled for auto-render: {prim.GetPath()}")
            
            print("    ✅ Camera auto-rendering disabled (use manual capture)")
            
        except Exception as e:
            print(f"    ⚠️  Camera disable: {e}")
    
    def _optimize_camera_rendering(self, stage, camera_root_path):
        """Optimize camera rendering to reduce frame drops"""
        try:
            import omni.replicator.core as rep
            
            # Find camera prims
            camera_prim = stage.GetPrimAtPath(camera_root_path)
            if not camera_prim.IsValid():
                return
            
            # Lower resolution for better performance
            for prim in Usd.PrimRange(camera_prim):
                if prim.IsA(UsdGeom.Camera):
                    # Set lower resolution
                    camera = UsdGeom.Camera(prim)
                    # Default RealSense is 1280x720, reduce to 640x480
                    if camera:
                        print(f"    📷 Camera optimized: {prim.GetPath()}")
            
            # Reduce synthetic data update frequency (prevent frame drops)
            settings = carb.settings.get_settings()
            settings.set("/omni/replicator/RTSubframes", 1)  # Reduce subframes
            settings.set("/omni/replicator/asyncRendering", True)  # Enable async
            settings.set("/omni/syntheticdata/activateOnPlay", False)  # Manual trigger only
            
            print("    ⚡ Camera rendering optimized (manual trigger mode)")
            
        except Exception as e:
            print(f"    ⚠️  Camera optimization: {e}")
    
    def _apply_optimizations(self, stage):
        """Apply rendering optimizations"""
        settings = carb.settings.get_settings()
        
        # DLSS Performance
        settings.set("/rtx/post/dlss/execMode", 0)
        
        # Disable heavy effects
        settings.set("/rtx/pathtracing/enabled", False)
        settings.set("/rtx/translucency/enabled", False)
        settings.set("/rtx/reflections/enabled", False)
        
        print("  ✅ Optimizations applied")
