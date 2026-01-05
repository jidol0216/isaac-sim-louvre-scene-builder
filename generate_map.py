"""
Louvre Occupancy Map Generator
==============================
Isaac Sim에서 3D 씬의 geometry로부터 직접 2D occupancy map 생성
SLAM 필요 없이 바로 맵 추출!

사용법:
1. Isaac Sim에서 씬 로드
2. Script Editor에서 이 스크립트 실행
3. maps/ 폴더에 맵 저장됨
"""

import omni.usd
from pxr import Usd, UsdGeom, Gf
import numpy as np
from PIL import Image
import yaml
from pathlib import Path


def generate_occupancy_map(
    resolution: float = 0.05,  # meters per pixel
    robot_height: float = 0.5,  # 로봇 높이에서 스캔
    min_height: float = 0.1,   # 장애물 최소 높이
    max_height: float = 2.0,   # 장애물 최대 높이
    padding: float = 5.0,      # 맵 가장자리 여백 (meters)
):
    """3D 씬에서 2D Occupancy Map 생성"""
    
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("❌ No stage loaded!")
        return
    
    print("🗺️ Generating Occupancy Map...")
    print(f"  Resolution: {resolution} m/pixel")
    print(f"  Scan height: {min_height} ~ {max_height} m")
    
    # 1. 씬의 모든 mesh bounding box 수집
    all_points = []
    obstacle_boxes = []
    
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            
            # World transform 적용된 bounding box
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
            bbox = bbox_cache.ComputeWorldBound(prim)
            bbox_range = bbox.ComputeAlignedRange()
            
            if bbox_range.IsEmpty():
                continue
            
            min_pt = bbox_range.GetMin()
            max_pt = bbox_range.GetMax()
            
            # Z 높이 필터 (로봇이 지나갈 수 있는 높이의 장애물만)
            if max_pt[2] < min_height or min_pt[2] > max_height:
                continue
            
            all_points.append((min_pt[0], min_pt[1]))
            all_points.append((max_pt[0], max_pt[1]))
            obstacle_boxes.append((min_pt, max_pt))
    
    if not all_points:
        print("❌ No meshes found!")
        return
    
    # 2. 맵 범위 계산
    all_points = np.array(all_points)
    x_min, y_min = all_points.min(axis=0) - padding
    x_max, y_max = all_points.max(axis=0) + padding
    
    width = int((x_max - x_min) / resolution)
    height = int((y_max - y_min) / resolution)
    
    print(f"  Map size: {width} x {height} pixels")
    print(f"  World bounds: X[{x_min:.1f}, {x_max:.1f}] Y[{y_min:.1f}, {y_max:.1f}]")
    
    # 3. Occupancy map 생성 (255=free, 0=occupied)
    occ_map = np.ones((height, width), dtype=np.uint8) * 255
    
    for min_pt, max_pt in obstacle_boxes:
        # World coords -> pixel coords
        px_min = int((min_pt[0] - x_min) / resolution)
        py_min = int((min_pt[1] - y_min) / resolution)
        px_max = int((max_pt[0] - x_min) / resolution)
        py_max = int((max_pt[1] - y_min) / resolution)
        
        # Clamp to image bounds
        px_min = max(0, min(width - 1, px_min))
        px_max = max(0, min(width - 1, px_max))
        py_min = max(0, min(height - 1, py_min))
        py_max = max(0, min(height - 1, py_max))
        
        # Mark as occupied (flip Y for image coords)
        occ_map[height - 1 - py_max:height - 1 - py_min, px_min:px_max] = 0
    
    # 4. 저장 (스크립트 위치 기준 상대 경로)
    script_dir = Path(__file__).resolve().parent
    save_dir = script_dir / "maps"
    save_dir.mkdir(exist_ok=True)
    
    map_image_path = save_dir / "louvre_map.png"
    map_yaml_path = save_dir / "louvre_map.yaml"
    
    # PNG 저장
    img = Image.fromarray(occ_map)
    img.save(str(map_image_path))
    print(f"  ✅ Saved: {map_image_path}")
    
    # YAML 저장
    yaml_content = {
        "image": "louvre_map.png",
        "resolution": resolution,
        "origin": [float(x_min), float(y_min), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    
    with open(map_yaml_path, 'w') as f:
        yaml.dump(yaml_content, f, default_flow_style=False)
    print(f"  ✅ Saved: {map_yaml_path}")
    
    # ROS2 nav용 맵도 복사 (환경변수 또는 기본 경로 사용)
    import os
    ros_workspace = os.environ.get('ROS_WS', os.path.expanduser('~/IsaacSim-ros_workspaces/humble_ws'))
    ros_map_dir = Path(ros_workspace) / "src/navigation/louvre_navigation/maps"
    if ros_map_dir.exists():
        import shutil
        shutil.copy(map_image_path, ros_map_dir / "louvre_map.png")
        shutil.copy(map_yaml_path, ros_map_dir / "louvre_map.yaml")
        print(f"  ✅ Copied to ROS2 workspace")
    
    print(f"\n🎉 Map generation complete!")
    print(f"   Origin: ({x_min:.2f}, {y_min:.2f})")
    print(f"   Size: {width}x{height} px = {(x_max-x_min):.1f}x{(y_max-y_min):.1f} m")
    
    return occ_map


# 실행
if __name__ == "__main__":
    generate_occupancy_map(
        resolution=0.05,    # 5cm per pixel
        min_height=0.1,     # 10cm 이상 장애물
        max_height=1.5,     # 1.5m 이하 장애물 (로봇 높이)
        padding=5.0,        # 5m 여백
    )
