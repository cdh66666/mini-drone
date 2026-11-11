"""逐个点动电机（经 WiFi HTTP），同时用麦克风录音，用频谱峰值比较四个电机转速。
用法：python -I motor_audio_test.py <飞机IP> <pm...>
"""
import sys, time, json, threading, urllib.request
import numpy as np, sounddevice as sd

IP = sys.argv[1]
PMS = [int(x) for x in sys.argv[2:]] or [250]
FS = 48000

def mic():
    for i, d in enumerate(sd.query_devices()):
        if d['max_input_channels'] > 0 and 'USB Audio' in d['name'] and d['hostapi'] == 0:
            return i
    return None

def get(p):
    return urllib.request.urlopen("http://%s%s" % (IP, p), timeout=2).read().decode()

def rec(sec, dev):
    a = sd.rec(int(sec * FS), samplerate=FS, channels=1, device=dev, dtype='float32')
    sd.wait()
    return a[:, 0]

def spectrum(x):
    x = x - x.mean()
    w = np.hanning(len(x))
    s = np.abs(np.fft.rfft(x * w))
    f = np.fft.rfftfreq(len(x), 1 / FS)
    return f, s

dev = mic()
print("mic", dev, sd.query_devices(dev)['name'])
base = rec(1.2, dev)
fb, sb = spectrum(base)
res = {}
for pm in PMS:
    for ch in range(4):
        out = {}
        def jog():
            time.sleep(0.25)
            get("/api/jog?i=%d&pm=%d&ms=1500" % (ch, pm))
            time.sleep(0.9)
            try:
                out['st'] = json.loads(get("/api/state"))
            except Exception:
                pass
        t = threading.Thread(target=jog); t.start()
        x = rec(2.0, dev)
        t.join()
        seg = x[int(0.75 * FS):int(1.65 * FS)]          # 电机稳定转的那段
        f, s = spectrum(seg)
        band = (f > 80) & (f < 2000)
        # 减去背景
        sbi = np.interp(f, fb, sb) * (len(seg) / len(base))
        d = np.where(band, s - sbi, 0)
        k = int(np.argmax(d))
        rms = float(np.sqrt(np.mean(seg ** 2)))
        top = sorted(((float(d[i]), float(f[i])) for i in np.argsort(d)[-6:]), reverse=True)
        res[(pm, ch)] = (float(f[k]), rms)
        print("pm=%d M%d  peak=%.1f Hz  rms=%.4f  out=%s  top=%s" % (
            pm, ch + 1, f[k], rms, out.get('st', {}).get('d'), [round(t[1]) for t in top]))
        time.sleep(1.0)
get("/api/stop")
print(json.dumps({"%d_M%d" % (k[0], k[1] + 1): v for k, v in res.items()}))
