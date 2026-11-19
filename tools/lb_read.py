"""v20 长黑匣子（/api/lb）：解析二进制、算振动频谱。台架脚本和电脑服务共用。

记录（每 1 ms 一条，小端，打包 42 字节）：
  us u32 · gr[3] i16（原始陀螺 ×10 dps，陷波前，芯片坐标）· gf[3] i16（滤波后 ×10）· ac[3] i16（mg）
  roll,pitch i16（×10°）· spR,spP i16（°/s）· o[4] u16（‰）· thr i16 · stR,stP i8（×100）
"""
import numpy as np

DT = np.dtype([("us", "<u4"), ("gr", "<i2", 3), ("gf", "<i2", 3), ("ac", "<i2", 3), ("roll", "<i2"), ("pitch", "<i2"),
               ("spR", "<i2"), ("spP", "<i2"), ("o", "<u2", 4), ("thr", "<i2"), ("stR", "i1"), ("stP", "i1")])
assert DT.itemsize == 42


def parse(b):
    """bytes → (头部文字, 结构数组)"""
    i = b.find(b"\n")
    head = b[:i].decode("utf-8", "replace"); body = b[i + 1:]
    n = len(body) // DT.itemsize
    return head, np.frombuffer(body[:n * DT.itemsize], dtype=DT)


def spectrum(r, min_thr=300, fmax=500):
    """电机转着时（thr ≥ min_thr）三轴原始/滤波后陀螺的幅度谱。返回 dict，频率和各轴幅度（dps）"""
    m = r["thr"] >= min_thr
    if m.sum() < 512:
        return None
    seg = r[m]
    t = seg["us"].astype(np.float64) * 1e-6
    dt = float(np.median(np.diff(t))) if len(t) > 2 else 0.001
    n = 1 << int(np.log2(min(len(seg), 8192)))
    win = np.hanning(n)
    out = {"fs": round(1 / dt, 1), "n": n, "sec": round(len(seg) * dt, 2)}
    f = np.fft.rfftfreq(n, dt)
    keep = (f > 2) & (f <= fmax)
    out["f"] = [round(x, 1) for x in f[keep]]
    for key in ("gr", "gf"):
        for ax, name in enumerate(("x", "y", "z")):
            x = seg[key][:, ax].astype(np.float64) / 10.0
            # Welch：分段平均，曲线更平滑
            acc = np.zeros(len(f)); k = 0
            for s in range(0, len(x) - n + 1, n // 2):
                acc += np.abs(np.fft.rfft((x[s:s + n] - x[s:s + n].mean()) * win)); k += 1
            amp = acc / max(k, 1) * 2 / win.sum()
            out[key + name] = [round(v, 3) for v in amp[keep]]
    # 找峰（原始陀螺三轴合起来）：按 5 Hz 以上、去掉相邻
    tot = np.array(out["grx"]) + np.array(out["gry"]) + np.array(out["grz"])
    ff = np.array(out["f"]); peaks = []
    for i in np.argsort(tot)[::-1]:
        if ff[i] < 15: continue
        if all(abs(ff[i] - p[0]) > 15 for p in peaks):
            peaks.append((round(float(ff[i]), 1), round(float(tot[i]), 2)))
        if len(peaks) >= 4: break
    out["peaks"] = peaks
    out["rms_raw"] = round(float(np.sqrt(np.mean((seg["gr"].astype(float) / 10 - (seg["gr"].astype(float) / 10).mean(0)) ** 2))), 2)
    out["rms_filt"] = round(float(np.sqrt(np.mean((seg["gf"].astype(float) / 10 - (seg["gf"].astype(float) / 10).mean(0)) ** 2))), 2)
    return out
