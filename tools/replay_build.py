"""把 flights/*.csv 做成回放数据 flights/replay.json：
1. 电机模型：用所有“芯片转角修好之后”(rot=-90，2026-11-10 02:28 起) 的空中数据，拟合
   角加速度 = b·x − d·ω + c，x = 一阶滞后(τ)·延迟(td) 的差动推力（‰）。滚转/俯仰/偏航各一组。
2. 每次飞行：用模型按记录的电机输出算“预测角速度”（每 0.2 s 用实测值重新对齐，看短期是否一致），给出吻合度。
3. 位置推算（航位推算）：水平加速度 ≈ g·tan(倾角)，按偏航角转到地面坐标，加空气阻力 −k·v，二重积分；
   高度：有 az（v8 以后）用 az·cosθ − 1g 积分，没有就用油门/悬停油门估推力。都是粗估，会漂。
用法：python -I tools/replay_build.py
"""
import csv, glob, json, math, os
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FL = os.path.join(HERE, "flights")
DT = 0.004
G = 9.81

def load(f):
    lines = open(f, encoding="utf-8").read().splitlines()
    head = lines[0] if lines and lines[0].startswith("#") else ""
    R = list(csv.DictReader([l for l in lines if not l.startswith("#")]))
    if not R:
        return None
    cols = {k: np.array([float(r[k]) for r in R]) for k in R[0].keys() if k}
    cols["_head"] = head
    return cols

def lagdelay(u, tau, td):
    d = int(round(td / DT)); x = 0.0; out = np.zeros(len(u))
    for k in range(len(u)):
        uk = u[k - d] if k - d >= 0 else 0.0
        x += DT / tau * (uk - x); out[k] = x
    return out

def smooth(w, fc=10.0):
    a = DT / (1 / (2 * math.pi * fc) + DT); y = np.zeros(len(w)); s = w[0]
    for i, v in enumerate(w): s += a * (v - s); y[i] = s
    # 前后各滤一遍，抵消相位滞后
    z = np.zeros(len(w)); s = y[-1]
    for i in range(len(w) - 1, -1, -1): s += a * (y[i] - s); z[i] = s
    return z

def inputs(c):
    o1, o2, o3, o4 = c["o1"], c["o2"], c["o3"], c["o4"]
    uR = ((o1 + o3) - (o2 + o4)) / 4      # 混控：oR+ → 1、3 号加力
    uP = ((o3 + o4) - (o1 + o2)) / 4      # 混控：oP+ → 3、4 号加力
    uY = ((o1 + o4) - (o2 + o3)) / 4      # 混控：oY+ → 1、4 号加力
    return {"roll": (smooth(uR), smooth(c["gr"])), "pitch": (smooth(uP), smooth(c["gp"])), "yaw": (smooth(uY), smooth(c["gy"]))}

def airborne(c):
    return (c["thr"] >= 330) & (np.abs(c["roll"]) < 20) & (np.abs(c["pitch"]) < 30)

def fit_axis(segs):
    """segs: [(u, w, mask)]；网格搜 τ、td，最小二乘 b、d、c（窗口 12 步 = 48 ms 的角速度变化）"""
    best = None
    for tau in (0.015, 0.025, 0.035, 0.05, 0.07):
        for td in (0.0, 0.008, 0.016, 0.024, 0.032):
            X, Y = [], []
            for u, w, m in segs:
                x = lagdelay(u, tau, td)
                W = 25
                for s in range(0, len(u) - W, 4):
                    if not m[s:s + W + 1].all():
                        continue
                    X.append([DT * x[s:s + W].sum(), -DT * w[s:s + W].sum(), DT * W])
                    Y.append(w[s + W] - w[s])
            if len(Y) < 50:
                continue
            X = np.array(X); Y = np.array(Y)
            coef, *_ = np.linalg.lstsq(X, Y, rcond=None)
            r2 = 1 - np.mean((Y - X @ coef) ** 2) / np.var(Y)
            if best is None or r2 > best["r2"]:
                best = {"tau": tau, "td": td, "b": float(coef[0]), "d": float(coef[1]), "c": float(coef[2]), "r2": float(r2), "n": int(len(Y))}
    return best

def predict(u, w, m, p):
    x = lagdelay(u, p["tau"], p["td"])
    pred = np.copy(w); cur = w[0]
    for k in range(1, len(u)):
        if k % 75 == 0 or not m[k]:
            cur = w[k]                       # 每 0.2 s 用实测对齐一次
        else:
            cur += DT * (p["b"] * x[k - 1] - p["d"] * cur + p["c"])
        pred[k] = cur
    return pred

def dead_reckon(c, hover_thr):
    n = len(c["thr"])
    yaw = np.cumsum(c["gy"]) * DT if "yaw" not in c else c["yaw"]
    m = c["thr"] >= 330
    pos = np.zeros((n, 3)); v = np.zeros(3); p = np.zeros(3); k_drag = 1.2
    for i in range(n):
        if m[i]:
            r, q, y = math.radians(c["roll"][i]), math.radians(c["pitch"][i]), math.radians(yaw[i])
            # 机体：roll+ = 1、3 号那边抬（往 2、4 号那边倒，右滚）；pitch+ = 1、2 号那边往下（往前）
            ab = np.array([G * math.tan(q), G * math.tan(r)])          # 前、右
            ax = ab[0] * math.cos(y) - ab[1] * math.sin(y)
            ay = ab[0] * math.sin(y) + ab[1] * math.cos(y)
            if "az" in c:
                az = (c["az"][i] * math.cos(r) * math.cos(q) - 1.0) * G
            else:
                az = (c["thr"][i] / hover_thr - 1.0) * G
            a = np.array([ax, ay, az]) - k_drag * v
            v = v + a * DT; p = p + v * DT
            if p[2] < 0: p[2] = 0; v[2] = max(0, v[2])
        else:
            v[:] = 0
        pos[i] = p
    return pos

def main():
    files = sorted(glob.glob(os.path.join(FL, "*.csv")))
    data = {os.path.basename(f): load(f) for f in files}
    data = {k: v for k, v in data.items() if v is not None and len(v["thr"]) > 50}
    fixed = [k for k in data if k >= "20261110_022800"]          # 芯片转角改对之后
    model = {}
    for ax in ("roll", "pitch", "yaw"):
        segs = []
        for k in fixed:
            c = data[k]; u, w = inputs(c)[ax]; segs.append((u, w, airborne(c)))
        model[ax] = fit_axis(segs)
    # 悬停油门：修好后各次空中油门的中位数
    hov = [np.median(data[k]["thr"][data[k]["thr"] >= 330]) for k in fixed if (data[k]["thr"] >= 330).sum() > 50]
    hover_thr = float(np.median(hov)) if hov else 450.0
    out = {"model": model, "hover_thr": hover_thr, "flights": []}
    for k, c in data.items():
        m = airborne(c); air = float((c["thr"] >= 330).sum() * DT)
        fl = {"name": k, "head": c["_head"], "air": round(air, 2), "fixed": k in fixed}
        step = 5                                                      # 50 Hz 给页面
        idx = np.arange(0, len(c["thr"]), step)
        t = (c["ms"] - c["ms"][0]) / 1000.0
        fl["t"] = np.round(t[idx], 3).tolist()
        for key in ("thr", "roll", "pitch", "gr", "gp", "gy", "o1", "o2", "o3", "o4"):
            fl[key] = np.round(c[key][idx], 1).tolist()
        yaw = c["yaw"] if "yaw" in c else np.cumsum(c["gy"]) * DT
        fl["yaw"] = np.round(((yaw + 180) % 360 - 180)[idx], 1).tolist()
        fit = {}
        for ax in ("roll", "pitch", "yaw"):
            p = model[ax]
            if not p: continue
            u, w = inputs(c)[ax]
            pr = predict(u, w, m, p)
            fl["pred_" + ax] = np.round(pr[idx], 1).tolist()
            if m.sum() > 50:
                e = (pr - w)[m]; fit[ax] = round(float(1 - np.var(e) / max(np.var(w[m]), 1e-6)), 2)
        fl["fit"] = fit
        pos = dead_reckon(c, hover_thr)
        fl["pos"] = np.round(pos[idx], 3).tolist()
        # 阻力读数轨迹（固件 v12+ 才有 vdx/vdy）：控制器没用它，不会被“拉回原点”，比较接近独立测量；
        # 但阻力系数 μ 还没标定（固件按 0.5 算），所以只能看方向和形状，距离不准。
        if "vdx" in c:
            pd = np.zeros((len(c["thr"]), 2)); p2 = np.zeros(2)
            for i in range(len(c["thr"])):
                if c["thr"][i] >= 330:
                    y = math.radians(yaw[i])
                    vx, vy = c["vdx"][i], c["vdy"][i]
                    p2 = p2 + np.array([vx * math.cos(y) - vy * math.sin(y), vx * math.sin(y) + vy * math.cos(y)]) * DT
                pd[i] = p2
            fl["pos_drag"] = np.round(pd[idx], 3).tolist()
        fl["selfhold"] = "vx" in c          # v11+：控制器在把推算位置拉回原点，推算轨迹天然好看
        # 操作：黑匣子里的油门是输出值（‰ of 700 满）；方向杆 stR/stP（v4 以后有）
        fl["cmd"] = np.round(c["thr"][idx] / 7.0, 1).tolist()
        if "stR" in c:
            fl["sr"] = np.round(c["stR"][idx], 2).tolist(); fl["sp"] = np.round(c["stP"][idx], 2).tolist()
        out["flights"].append(fl)
    # 整次飞行记录（电脑端 25 Hz，解锁→上锁）：有电压和操作
    for f in sorted(glob.glob(os.path.join(FL, "full", "*.csv"))):
        lines = open(f, encoding="utf-8").read().splitlines()
        R = list(csv.DictReader(lines))
        if len(R) < 10: continue
        g = lambda k: np.array([float(r[k]) for r in R]) if k in R[0] else None
        t = g("t"); n = len(t)
        fl = {"name": "整次_" + os.path.basename(f), "head": "整次飞行（电脑端 25 Hz）", "full": True, "fixed": True,
              "selfhold": os.path.basename(f) >= "20261111_1400"}   # v11（11-11 14 点前后刷入）起飞控在拉回推算位置
        fl["t"] = np.round(t, 3).tolist()
        fl["cmd"] = np.round(g("thr_cmd") / 10.0, 1).tolist()           # 杆量 ‰ → %
        fl["thr"] = np.round(g("thr_cmd") * 0.7, 1).tolist()            # 近似输出 ‰
        for k, s in (("roll", "roll"), ("pitch", "pitch"), ("yaw", "yaw"), ("gy", "gy"), ("o1", "o1"), ("o2", "o2"), ("o3", "o3"), ("o4", "o4")):
            fl[k] = np.round(g(s), 1).tolist()
        fl["gr"] = [0] * n; fl["gp"] = [0] * n
        fl["vb"] = np.round(g("vb_mv") / 1000.0, 3).tolist()
        if g("stick_r") is not None:
            fl["sr"] = g("stick_r").tolist(); fl["sp"] = g("stick_p").tolist()
        air = float(t[-1] - t[0]); fl["air"] = round(air, 1); fl["fit"] = {}
        # 位置推算（25 Hz 版）
        p = np.zeros(3); v = np.zeros(3); pos = []
        for i in range(n):
            dt = (t[i] - t[i - 1]) if i else 0.04
            if fl["cmd"][i] >= 45:
                r_, q_, y_ = math.radians(fl["roll"][i]), math.radians(fl["pitch"][i]), math.radians(fl["yaw"][i])
                af, ar = G * math.tan(q_), G * math.tan(r_)
                a = np.array([af * math.cos(y_) - ar * math.sin(y_), af * math.sin(y_) + ar * math.cos(y_), 0.0]) - 1.2 * v
                v = v + a * dt; p = p + v * dt
            else:
                v[:] = 0
            pos.append(p.copy())
        fl["pos"] = np.round(np.array(pos), 3).tolist()
        # 续航：起飞后电压掉速度
        vb = np.array(fl["vb"]); m = np.array(fl["cmd"]) >= 45
        if m.sum() > 10:
            fl["batt"] = {"v_start": round(float(vb[m][:5].mean()), 3), "v_end": round(float(vb[m][-5:].mean()), 3),
                          "air_s": round(float(m.sum() * 0.04), 1)}
        out["flights"].append(fl)
    json.dump(out, open(os.path.join(FL, "replay.json"), "w", encoding="utf-8"))
    print("hover_thr", hover_thr)
    for ax, p in model.items(): print(ax, p)
    for fl in out["flights"]:
        if fl["fixed"] and fl["air"] > 0.5:
            P = np.array(fl["pos"]); print(fl["name"], "air", fl["air"], "fit", fl["fit"], "终点 x%.2f y%.2f z%.2f m" % tuple(P[-1]))

if __name__ == "__main__":
    main()
