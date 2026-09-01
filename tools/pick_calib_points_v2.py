#!/usr/bin/env python3
"""
极简标定工具 - 依次点击5个已知地标，自动求解相机外参。

用法:
    python3 tools/pick_calib_points_v2.py /tmp/match_mid.jpg

操作:
    左键  = 确认当前点
    右键  = 撤销上一个点
    ESC   = 退出

需要点的5个点（按顺序，每次左键一次）:
  1. 对方（红方）塔顶 发光LED亮点    世界坐标 (10.822,  3.664, 1.868)
  2. 己方（蓝方）塔顶（不发光，估位） 世界坐标 (17.178, 11.336, 1.868)
  3. 场地中心地面                    世界坐标 (14.0,    7.5,   0.0)
  4. 你能确认的任意地面角点/标记A     (运行前修改 EXTRA_PTS 里的坐标)
  5. 你能确认的任意地面角点/标记B     (运行前修改 EXTRA_PTS 里的坐标)
"""
import sys
import cv2
import numpy as np
import json

K    = np.array([[5033.780199, 0.0,         2829.234535],
                 [0.0,         5036.139955, 1929.489557],
                 [0.0,         0.0,         1.0        ]], dtype=np.float64)
DIST = np.array([-0.061883, 0.104794, 0.000434, -0.000036, 0.0], dtype=np.float64)

# ── 点位来自 config/blue/calibrate_points_blue.yaml，点击顺序固定 ──────────
# Key order: self_fortress, self_tower, enemy_base, enemy_tower, cross_tower
WORLD_PTS = np.array([
    [20.06790,  7.46930, 0.00020],   # 1. self_fortress  蓝方基地近角
    [17.17800, 11.33570, 1.86860],   # 2. self_tower      蓝方前哨最高点
    [ 4.98290, 10.54700, 0.60000],   # 3. enemy_base      红方梯形能量机关
    [ 3.45860,  3.91990, 0.73180],   # 4. enemy_tower     红方基地左90度围栏
    [10.82200,  3.66430, 1.86840],   # 5. cross_tower     红方前哨灯柱底角
], dtype=np.float64)

LABELS = [
    "1. Blue-base near corner",
    "2. Blue outpost highest pt",
    "3. Red trapezoid buff",
    "4. Red base left 90deg fence",
    "5. Red outpost lamp corner",
]
LABELS_ZH = [
    "1. 蓝方基地近角 (self_fortress)",
    "2. 蓝方前哨最高点 (self_tower)",
    "3. 红方梯形能量机关 (enemy_base)",
    "4. 红方基地左90度围栏 (enemy_tower)",
    "5. 红方前哨灯柱底角 (cross_tower)",
]
# ─────────────────────────────────────────────────────────────────────────────

SCALE  = 0.25
clicks = []   # list of (px, py) in original resolution

def draw(img):
    h, w = img.shape[:2]
    disp = cv2.resize(img, (int(w*SCALE), int(h*SCALE)))
    n = len(clicks)
    for i, (px, py) in enumerate(clicks):
        dx, dy = int(px*SCALE), int(py*SCALE)
        cv2.circle(disp, (dx, dy), 10, (0, 255, 0), -1)
        cv2.putText(disp, LABELS[i], (dx+12, dy+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    if n < len(LABELS):
        msg = f"Click: {LABELS[n]}  ({n+1}/{len(LABELS)})"
        color = (0, 200, 255)
    else:
        msg = "All points collected - press Q or Enter to solve"
        color = (0, 255, 128)
    cv2.putText(disp, msg,  (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    cv2.putText(disp, "RightClick=undo  ESC=quit", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180,180,180), 1)
    return disp

def solve():
    if len(clicks) < 4:
        print(f"至少需要4个点，当前 {len(clicks)} 个"); return
    img_pts = np.array(clicks[:len(WORLD_PTS)], dtype=np.float64)
    obj_pts = WORLD_PTS[:len(img_pts)]
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        obj_pts, img_pts, K, DIST,
        iterationsCount=3000, reprojectionError=10.0,
        confidence=0.999, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        print("solvePnP 失败，请检查点位是否正确"); return
    if inl is not None and len(inl) >= 4:
        cv2.solvePnPRefineLM(obj_pts[inl.flatten()],
                             img_pts[inl.flatten()], K, DIST, rvec, tvec)
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, DIST)
    errs = np.linalg.norm(proj.reshape(-1,2) - img_pts, axis=1)
    R, _ = cv2.Rodrigues(rvec)
    cam = -R.T @ tvec.flatten()
    print("\n========== 标定结果 ==========")
    for i, (lbl, e) in enumerate(zip(LABELS[:len(errs)], errs)):
        print(f"  {lbl:20s}  重投影误差={e:.1f} px")
    print(f"平均误差: {errs.mean():.1f} px")
    print(f"相机世界坐标: X={cam[0]:.3f}  Y={cam[1]:.3f}  Z={cam[2]:.3f}")
    rv, tv = rvec.flatten(), tvec.flatten()
    print("\n保存到 /tmp/calib_result.json (rvec/tvec):")
    print(f"RVEC = np.array([{rv[0]:.8f}, {rv[1]:.8f}, {rv[2]:.8f}], dtype=np.float64)")
    print(f"TVEC = np.array([{tv[0]:.8f}, {tv[1]:.8f}, {tv[2]:.8f}], dtype=np.float64)")
    with open("/tmp/calib_result.json","w") as f:
        json.dump({"RVEC":rv.tolist(),"TVEC":tv.tolist(),
                   "cam_world":cam.tolist(),"mean_err_px":float(errs.mean())}, f, indent=2)
    print("已保存 → /tmp/calib_result.json")

def mouse(event, x, y, flags, img):
    global clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(clicks) < len(WORLD_PTS):
            clicks.append((x/SCALE, y/SCALE))
    elif event == cv2.EVENT_RBUTTONDOWN:
        if clicks: clicks.pop()

def main():
    path = sys.argv[1] if len(sys.argv)>1 else "/tmp/match_mid.jpg"
    img = cv2.imread(path)
    if img is None: print(f"无法打开: {path}"); return
    cv2.namedWindow("calib", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("calib", mouse, img)
    print("\n依次左键点击以下地标（右键撤销，Q/Enter求解）:")
    for i, lbl in enumerate(LABELS_ZH):
        print(f"  {lbl}")
    print()
    while True:
        cv2.imshow("calib", draw(img))
        k = cv2.waitKey(30) & 0xFF
        if k == 27: break
        if k in (ord('q'), ord('Q'), 13): solve(); break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
