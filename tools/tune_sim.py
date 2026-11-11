"""按黑匣子拟合的单轴模型仿真固件的串级 PID，挑稳健参数。
模型：角加速度 = b * x，x = 一阶滞后(tau) 的 u(t - delay)；u = 混控后的差动量（‰）。
控制器照抄固件：角度外环 P → 角速度 PID（D 对测量、限幅 1500 dps/s、30 Hz 低通、D 限 ±250、输出 ±900），
陀螺 60 Hz 低通、量程 ±512 dps，加噪声。打分：多组模型（b/tau/延迟上下浮动）都要稳且收敛快。
"""
import math, itertools, random
import numpy as np

DT = 0.004

def sim(kp, ki, kd, angP, b, tau, delay, T=3.0, noise=4.0, dist=40.0, seed=1, omax=900, imax=150, ang0=8.0):
    rnd = random.Random(seed)
    n = int(T / DT); dly = int(round(delay / DT))
    ubuf = [0.0] * (dly + 1)
    x = 0.0; w = 0.0; ang = ang0      # 真实角速度 / 角度
    gf = 0.0; it = 0.0; pm = 0.0; df = 0.0
    rc = 1 / (2 * math.pi * 30); ad = DT / (rc + DT)
    rg = 1 / (2 * math.pi * 60); ag = DT / (rg + DT)
    angF = ang
    err_int = 0.0; maxang = 0.0; sat = 0
    for k in range(n):
        g = max(-512, min(512, w)) + rnd.gauss(0, noise)
        gf += (g - gf) * ag
        angF = (angF + gf * DT) * 0.97 + ang * 0.03
        sp = max(-250, min(250, angP * (0 - angF)))
        e = sp - gf
        it = max(-imax, min(imax, it + ki * e * DT))
        dm = max(-1500, min(1500, (gf - pm) / DT)); pm = gf
        df += (dm - df) * ad
        dt_ = max(-250, min(250, kd * df))
        u = max(-omax, min(omax, kp * e + it - dt_))
        if abs(u) >= omax * 0.99: sat += 1
        ubuf.append(u); ud = ubuf.pop(0)
        x += DT / tau * (ud - x)
        w += DT * (b * x + b * dist * (1 if k * DT > 1.0 else 0))   # 1 s 时加一个持续偏载（像一个电机弱）
        ang += w * DT
        if abs(ang) > 60: return 1e9, 60, sat
        if k * DT > 0.3: err_int += abs(ang) * DT
        maxang = max(maxang, abs(ang) if k * DT > 0.5 else 0)
    return err_int, maxang, sat

MODELS = [(b, tau, d) for b in (7, 12, 18) for tau in (0.035, 0.05, 0.075) for d in (0.016, 0.028, 0.040)]

def score(kp, ki, kd, angP):
    worst = 0; tot = 0
    for b, tau, d in MODELS:
        e, mx, sat = sim(kp, ki, kd, angP, b, tau, d)
        if e >= 1e9: return 1e9, 0
        worst = max(worst, mx); tot += e
    return tot / len(MODELS), worst

if __name__ == "__main__":
    print("当前 6/3/0.15/0.078:", score(3.0, 0.15, 0.078, 6.0))
    print("原来 2/1/0.15/0.06:", score(1.0, 0.15, 0.06, 2.0))
    res = []
    for kp, kd, angP, ki in itertools.product((0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.2),
                                              (0.0, 0.01, 0.02, 0.03, 0.05, 0.08),
                                              (1.5, 2.0, 2.5, 3.0, 4.0, 5.0),
                                              (0.1, 0.2, 0.4, 0.8)):
        s, w = score(kp, ki, kd, angP)
        if s < 1e9: res.append((s, w, kp, ki, kd, angP))
    res.sort()
    for r in res[:12]:
        print("误差积分 %.2f 最大角 %.1f°  rp=%.2f ri=%.2f rd=%.3f angP=%.1f" % r)
