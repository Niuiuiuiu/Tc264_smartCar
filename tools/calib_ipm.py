#!/usr/bin/env python3
# -*- coding: gbk -*-
"""
calib_ipm.py —— 标定「摄像头像素 (col,row) -> 地面米制 (x,y)」的单应矩阵 H

原理
----
摄像头看的是地面这张平面(z=0)。两个平面之间的射影映射一定是 3x3 单应矩阵，
只有 8 个自由度，所以 4 个点（每点 2 个方程）就够解出来。点多于 4 就是最小
二乘，能把读数误差平均掉。

依赖
----
    pip install numpy              （必需）
    pip install opencv-python      （只有 --pick / --view 需要）

用法
----
    python calib_ipm.py --pick shot.png    # 在照片上点十字架，自动亚像素精化
    python calib_ipm.py                    # 用写死的数据出 H + 残差 + C 数组
    python calib_ipm.py --view shot.png    # 另外存一张鸟瞰图，肉眼验证
"""

import sys

import numpy as np


# ============================================================
# 1) 标定数据 —— 现在全是占位数，真标定时整套换掉
# ============================================================
# 像素坐标 (col,row)：col 是列(向右增大)，row 是行(向下增大)
IMG_PTS = [
    ( 45.0, 112.0),   # 近-左
    (143.0, 112.0),   # 近-右
    ( 62.0,  38.0),   # 远-左
    (126.0,  38.0),   # 远-右
]

# 地面坐标 (x,y)：米。原点 = 车中心，x 向车头为正，y 向车身左侧为正。
# 注意：y 的正负必须和 code/ipm.h 里 ipm_ground_point_t 的约定一致。
#       两边符号反了，车会朝弯道外侧打舵。
GND_PTS = [
    (0.40,  0.30),    # 近-左 -> y 正
    (0.40, -0.30),    # 近-右 -> y 负
    (1.60,  0.60),    # 远-左
    (1.60, -0.60),    # 远-右
]


# ============================================================
# 2) 求 H（纯 numpy，不装 OpenCV 也能跑）
# ============================================================
def solve_h_dlt(img_pts, gnd_pts):
    """最小二乘求 H，固定 h33 = 1，于是只剩 8 个未知数。

    每一对 (col,row)->(x,y) 拆成两个方程：
        col*h11 + row*h12 + h13 - col*x*h31 - row*x*h32 = x
        col*h21 + row*h22 + h23 - col*y*h31 - row*y*h32 = y

    4 个点 = 8 个方程 = 恰好解出（这就是"贴四个十字架"的来历）；
    点多于 4 就是最小二乘。
    """
    rows_a = []
    rows_b = []

    for (col, row), (x, y) in zip(img_pts, gnd_pts):
        rows_a.append([col, row, 1.0, 0.0, 0.0, 0.0, -col * x, -row * x])
        rows_b.append(x)
        rows_a.append([0.0, 0.0, 0.0, col, row, 1.0, -col * y, -row * y])
        rows_b.append(y)

    h = np.linalg.lstsq(np.array(rows_a), np.array(rows_b), rcond=None)[0]

    return np.array([
        [h[0], h[1], h[2]],
        [h[3], h[4], h[5]],
        [h[6], h[7], 1.0],
    ])


def apply_h(H, col, row):
    """按 C 里一模一样的算法算一遍，保证两边不会对不上。"""
    p = H @ np.array([col, row, 1.0])
    return p[0] / p[2], p[1] / p[2]


# ============================================================
# 3) 体检：把标定点代回去看差多少
# ============================================================
def report(H, img_pts, gnd_pts):
    print("\n  (col,row)           期望(x,y)         算出来(x,y)        误差")
    print("  " + "-" * 64)

    worst = 0.0
    for (col, row), (x, y) in zip(img_pts, gnd_pts):
        px, py = apply_h(H, col, row)
        err = float(np.hypot(px - x, py - y)) * 100.0
        worst = max(worst, err)
        print("  (%6.1f,%6.1f)   (%+.3f,%+.3f)    (%+.3f,%+.3f)    %6.2f cm"
              % (col, row, x, y, px, py, err))

    print("\n  最大残差 %.2f cm" % worst)

    if worst > 2.0:
        print("  [警告] 超过 2 cm。先别用，回头查：")
        print("         1. 十字架中心读准了吗（用 --pick 的亚像素精化）")
        print("         2. 卷尺量的米数对吗")
        print("         3. 镜头畸变是不是太大（广角镜头要先去畸变）")

    return worst


def print_c_array(H):
    print("\n/* ---- 把下面整块替换掉 code/ipm.c 里的 IPM_H ---- */")
    print("static const float IPM_H[3][3] =")
    print("{")
    for r in H:
        print("    { %+13.8ff, %+13.8ff, %+13.8ff },"
              % (r[0], r[1], r[2]))
    print("};")


# ============================================================
# 4) 亚像素精化：把手工点的十字中心磨到 0.1 像素
# ============================================================
def refine_center_subpixel(gray, col, row, half=8):
    """在点击位置取个小窗口，用灰度重心把中心磨细。

    前提：十字比地面暗（黑胶带贴白地）。反了就把 (255.0 - patch) 改成 patch。
    """
    c0, r0 = int(round(col)), int(round(row))
    patch = gray[r0 - half: r0 + half + 1,
                 c0 - half: c0 + half + 1].astype(np.float64)

    if patch.size == 0:
        return col, row

    w = 255.0 - patch                        # 越暗权重越大
    w = np.clip(w - w.max() * 0.5, 0.0, None)  # 只留明显暗的，丢掉地面背景
    s = w.sum()

    if s < 1e-6:
        return col, row

    rr, cc = np.mgrid[0:patch.shape[0], 0:patch.shape[1]]
    return (c0 - half + float((w * cc).sum() / s),
            r0 - half + float((w * rr).sum() / s))


def pick_points(img_path, n):
    """开窗口点 n 个十字架，返回亚像素坐标。点完按任意键。"""
    import cv2

    frame = cv2.imread(img_path)
    if frame is None:
        sys.exit("读不到图：%s" % img_path)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    shown = frame.copy()
    pts = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < n:
            c, r = refine_center_subpixel(gray, x, y)
            pts.append((round(c, 2), round(r, 2)))
            cv2.drawMarker(shown, (int(round(c)), int(round(r))),
                           (0, 0, 255), cv2.MARKER_CROSS, 10, 1)
            cv2.putText(shown, str(len(pts)), (x + 6, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("pick", shown)
            print("  #%d  (%7.2f, %7.2f)" % (len(pts), c, r))

    cv2.imshow("pick", shown)
    cv2.setMouseCallback("pick", on_mouse)
    print("在窗口里依次点 %d 个十字架中心，点完按任意键..." % n)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return pts


# ============================================================
# 5) 鸟瞰图：最直观的验证
# ============================================================
def birdview(img_path, H, ppm=200.0,
             x_min=0.0, x_max=3.0, y_min=-1.2, y_max=1.2):
    """把原图掰成俯视图存盘。输出图里 1 m = ppm 像素。"""
    import cv2

    frame = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if frame is None:
        print("（读不到 %s，跳过鸟瞰图）" % img_path)
        return

    width = int(round((y_max - y_min) * ppm))
    height = int(round((x_max - x_min) * ppm))

    # 地面米 -> 俯视图像素：  col = (y_max - y)*ppm,  row = (x_max - x)*ppm
    scale = np.array([[0.0, -ppm, y_max * ppm],
                      [-ppm, 0.0, x_max * ppm],
                      [0.0, 0.0, 1.0]])

    out = cv2.warpPerspective(frame, scale @ H, (width, height))
    out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)

    # 叠一层 0.5 m 的米尺网格，用来读"这里离车多远"
    def to_px(x, y):
        return (int(round(y_max * ppm - y * ppm)),
                int(round(x_max * ppm - x * ppm)))

    x = x_min
    while x <= x_max + 1e-9:
        cv2.line(out, to_px(x, y_min), to_px(x, y_max), (60, 60, 255), 1)
        x += 0.5
    y = y_min
    while y <= y_max + 1e-9:
        cv2.line(out, to_px(x_min, y), to_px(x_max, y), (60, 60, 255), 1)
        y += 0.5

    name = img_path.rsplit(".", 1)[0] + "_birdview.png"
    cv2.imwrite(name, out)

    print("\n  鸟瞰图已存：%s" % name)
    print("  怎么看：地上真实是直的东西（胶带、赛道边线、方格纸的格线）")
    print("          在鸟瞰图里必须变直；方格纸的格子必须变成等大的正方格。")


# ============================================================
def main():
    args = sys.argv[1:]
    img_pts, gnd_pts = IMG_PTS, GND_PTS

    if "--pick" in args:
        img_path = args[args.index("--pick") + 1]
        img_pts = pick_points(img_path, len(gnd_pts))
        print("\n把下面这些贴回脚本的 IMG_PTS：")
        print("IMG_PTS = [")
        for col, row in img_pts:
            print("    (%7.2f, %7.2f)," % (col, row))
        print("]")

    H = solve_h_dlt(img_pts, gnd_pts)
    report(H, img_pts, gnd_pts)
    print_c_array(H)

    if "--view" in args:
        birdview(args[args.index("--view") + 1], H)


if __name__ == "__main__":
    main()
