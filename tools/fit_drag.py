"""阻力测速标定（固件 v12+ 黑匣子）：只用飞机自己的数据，不用摄像头。

原理：空中“加计倾角” ap/ar ≈ μ·v / g（阻力）。v 不知道，但短时间里姿态积分 ∫g·tanθ dt 很准，
差的只是一个起始速度和一条慢慢斜着走的误差（水平校准偏差）：v = v_int + c0 + c1·t。
所以按 g·ap = μ·(v_int + c0 + c1·t) 做最小二乘，每个轴解出 μ。μ 稳定、R2 高 → 阻力测速可用。

用法：python -I tools/fit_drag.py [黑匣子 csv ...]   不写就用 flights/ 里所有带 ap 列的
"""
import csv, glob, os, sys
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
G = 9.81


def load(fn):
    L = open(fn, encoding="utf-8").read().splitlines()
    rows = [l for l in L if l[:1].isdigit() or l.startswith("ms")]
    d = list(csv.DictReader(rows))
    if not d or "ap" not in d[0]:
        return None
    return {k: np.array([float(r[k]) for r in d]) for k in d[0]}


def fit_axis(t, tilt_deg, acc_tan):
    dt = np.diff(t, prepend=t[0])
    v_int = np.cumsum(G * np.tan(np.radians(tilt_deg)) * dt)
    # 加计读数低通 0.25 s，和固件一样；v_int 也同样低通，相位对齐
    def lp(x):
        y = np.zeros_like(x); a = 0.0
        for i in range(len(x)):
            a += (x[i] - a) * dt[i] / 0.25; y[i] = a
        return y
    y = G * lp(acc_tan)
    X = np.column_stack([lp(v_int), np.ones_like(t), t - t[0]])
    # y = μ·v_int + μc0 + μc1·t  → 线性：系数 [μ, μc0, μc1]
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coef
    r2 = 1 - np.sum((y - pred) ** 2) / max(np.sum((y - y.mean()) ** 2), 1e-9)
    return coef[0], r2, np.std(y)


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    fs = sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, "flights", "*.csv")))
    res = []
    for fn in fs:
        A = load(fn)
        if A is None:
            continue
        t = (A["ms"] - A["ms"][0]) / 1000
        air = A["thr"] > 300
        if air.sum() < 400:
            continue
        i0 = np.argmax(air) + 150                       # 离地 0.6 s 后
        i1 = len(air) - 1 - np.argmax(air[::-1])
        stick = (np.abs(A["stR"]) + np.abs(A["stP"])) > 0.01
        seg = np.arange(i0, i1)
        if len(seg) < 250 or stick[seg].any():
            print("%s：空中太短或按了方向键，跳过" % os.path.basename(fn)); continue
        ts = t[seg]
        mp, r2p, sp = fit_axis(ts, A["pitch"][seg], A["ap"][seg])
        mr, r2r, sr = fit_axis(ts, A["roll"][seg], A["ar"][seg])
        res.append((mp, r2p, mr, r2r))
        print("%s  空中 %.1fs  前后轴 μ=%+.2f R2=%.2f（读数波动 %.3f m/s²）  左右轴 μ=%+.2f R2=%.2f（%.3f）" % (
            os.path.basename(fn), ts[-1] - ts[0], mp, r2p, sp, mr, r2r, sr))
    if res:
        R = np.array(res)
        good = R[(R[:, 1] > 0.3) & (R[:, 3] > 0.3)]
        print("\n%d 次可用，其中两轴 R2>0.3 的 %d 次" % (len(R), len(good)))
        if len(good):
            print("建议 μ：前后 %.2f、左右 %.2f（取中位数；两轴接近就用平均）" % (np.median(good[:, 0]), np.median(good[:, 2])))


if __name__ == "__main__":
    main()
