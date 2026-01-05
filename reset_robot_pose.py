#!/usr/bin/env python3
"""
Isaac Sim에서 로봇을 initial pose (0, 0, 2.0)로 재설정하는 스크립트
Isaac Sim Script Editor에서 실행하세요.
"""

from pxr import Usd, UsdGeom, Gf

# 현재 열려있는 stage 가져오기
stage = omni.usd.get_context().get_stage()

# 로봇 prim 찾기 (경로는 실제 로봇 경로에 맞게 수정)
robot_paths = [
    "/RidgebackFranka",
    "/World/RidgebackFranka",
    "/ridgeback_franka"
]

robot_prim = None
for path in robot_paths:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        robot_prim = prim
        print(f"Found robot at: {path}")
        break

if robot_prim:
    # Xformable로 변환
    xform = UsdGeom.Xformable(robot_prim)
    
    # 기존 xform ops 가져오기
    xform_ops = xform.GetOrderedXformOps()
    
    # 기존 ops 찾아서 수정
    translate_op = None
    orient_op = None
    
    for op in xform_ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            orient_op = op
    
    # translate op이 없으면 새로 생성, 있으면 수정
    if translate_op:
        translate_op.Set(Gf.Vec3d(0.0, 0.0, 2.0))
    else:
        translate_op = xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(0.0, 0.0, 2.0))
    
    # orient op이 없으면 새로 생성, 있으면 수정
    if orient_op:
        orient_op.Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    else:
        orient_op = xform.AddOrientOp()
        orient_op.Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    
    print(f"Robot position reset to (0, 0, 2.0)")
    print(f"Robot orientation reset to identity")
else:
    print("Robot prim not found. Available root prims:")
    for prim in stage.GetPseudoRoot().GetChildren():
        print(f"  - {prim.GetPath()}")
