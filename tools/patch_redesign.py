"""起飞页重排 + 顶栏电量/充电/信号 + 配平上页面 + 去掉手感页"""
import sys
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
def rep(a, b):
    global t
    assert t.count(a) == 1, a[:80]; t = t.replace(a, b)

# ---------- 顶栏 ----------
rep('''  <span class="chip"><span class="dot" id="cdot"></span><span id="ctext">未连接</span></span>
  <span class="chip" title="电池">🔋 <span id="vb">--</span></span>
  <span class="chip" id="mode" style="display:none"></span>''', '''  <span class="chip"><span class="dot" id="cdot"></span><span id="ctext">未连接</span></span>
  <span class="chip" id="mode" style="display:none"></span>
  <span class="chip" id="batChip" title="电池"><span class="batt"><i id="batFill"></i></span><b id="batPct">--</b><span class="dim" id="vb"></span><span id="chg" class="dim"></span></span>''')
rep('''  <button data-s="3"><span class="i">🛫</span>起飞</button>
  <button data-s="4"><span class="i">🎚️</span>手感</button>''', '''  <button data-s="3"><span class="i">🛫</span>飞行</button>
  <button data-s="4" style="display:none"><span class="i">🎚️</span>手感</button>''')
rep('''  <button data-s="1"><span class="i">📐</span>放平</button>''', '''  <button data-s="1"><span class="i">📐</span>校准</button>''')

# ---------- 起飞页 ----------
i = t.index('<!-- 4 起飞 -->'); j = t.index('<!-- 5 手感 -->')
t = t[:i] + '''<!-- 4 飞行 -->
<section class="step" data-s="3">
  <div class="fly">
    <div class="thr">
      <div class="pct" id="thrPct">0%</div>
      <input type="range" id="thr" min="0" max="100" step="1" value="0">
      <div class="tk">⏎ <input id="tkoff" type="number" min="20" max="90" step="1">%</div>
    </div>

    <div class="mid">
      <div class="inst">
        <div class="ibox">
          <div class="level sm"><div class="ring"></div><div class="bub" id="bub2"></div></div>
          <div class="cap">姿态</div>
        </div>
        <div class="ibox">
          <svg viewBox="-50 -50 100 100" width="150" height="150"><circle r="47" fill="#1f2833" stroke="#2a333e" stroke-width="2"/>
            <text y="-34" fill="#8a97a6" font-size="9" text-anchor="middle">机头</text>
            <g id="yawNeedle"><path d="M0 -38 L8 2 L0 -6 L-8 2 Z" fill="#3b9eff"/></g></svg>
          <div class="cap" id="yawTxt">偏航 --</div>
        </div>
        <div class="ibox">
          <div style="display:flex;gap:6px;align-items:flex-end">
            <canvas id="liveMap" width="300" height="300"></canvas>
            <div class="zbar"><div id="liveZ"></div></div>
          </div>
          <div class="cap" id="liveTxt">轨迹（估）</div>
        </div>
      </div>
      <div class="mrow">
        <span class="dim">电机</span>
        <div class="bar"><i id="o1"></i><b>1</b></div><div class="bar"><i id="o2"></i><b>2</b></div>
        <div class="bar"><i id="o3"></i><b>3</b></div><div class="bar"><i id="o4"></i><b>4</b></div>
        <span class="grow"></span>
        <a class="link" id="btnHist">▶ 历史回放</a>
      </div>
    </div>

    <div class="side">
      <div class="tog" id="confirm"><span class="box"></span>安全确认</div>
      <button class="okb" id="arm">🔓 解锁</button>
      <button class="ghost" id="btnLand">⬇ 降落</button>
      <div class="trim">
        <div class="cap">配平 <span class="dim" id="trimTxt">--</span></div>
        <div class="tpad">
          <span></span><button data-t="P,1">▲</button><span></span>
          <button data-t="R,-1">◀</button><span class="tc">✥</span><button data-t="R,1">▶</button>
          <span></span><button data-t="P,-1">▼</button><span></span>
        </div>
        <div class="dim" style="font-size:12px;text-align:center">总往哪飘，就点反方向</div>
      </div>
      <div class="keys" style="display:none"><span class="key" id="kU"></span><span class="key" id="kL"></span><span class="key" id="kD"></span><span class="key" id="kR"></span></div>
      <div class="dim" style="font-size:12px;text-align:center">⏎ 起飞 · 空格 急停 · 方向键 扶正</div>
    </div>
  </div>
</section>

''' + t[j:]

# ---------- 样式 ----------
rep('''.fly{display:grid;grid-template-columns:150px 1fr 260px;gap:20px;flex:1;min-height:0}''',
    '''.fly{display:grid;grid-template-columns:110px minmax(0,1fr) 220px;gap:16px;flex:1;min-height:0}
.dim{color:var(--dim)}
.batt{display:inline-block;width:26px;height:12px;border:2px solid #8a97a6;border-radius:3px;position:relative;margin-right:2px}
.batt::after{content:"";position:absolute;right:-5px;top:2px;width:3px;height:4px;background:#8a97a6;border-radius:1px}
.batt i{position:absolute;left:1px;top:1px;bottom:1px;width:0;background:var(--ok);border-radius:1px}
#batChip{gap:8px}
.mid{display:flex;flex-direction:column;gap:12px;min-width:0}
.inst{flex:1;display:flex;flex-wrap:wrap;gap:14px;justify-content:space-around;align-content:center;background:var(--card);border-radius:16px;padding:16px}
.ibox{display:flex;flex-direction:column;align-items:center;gap:6px}
.cap{font-size:13px;color:var(--dim)}
#liveMap{width:150px;height:150px;background:#141a21;border-radius:50%}
.zbar{width:8px;height:150px;background:#141a21;border-radius:4px;position:relative}
.zbar div{position:absolute;left:0;right:0;bottom:0;height:0;background:var(--ok);border-radius:4px}
.mrow{display:flex;align-items:center;gap:10px;background:var(--card);border-radius:12px;padding:10px 14px}
.mrow .bar{width:60px}
.link{color:var(--acc);cursor:pointer;font-size:14px}
.side{display:flex;flex-direction:column;gap:12px}
.side .tog{justify-content:center;padding:10px}
.side #btnLand{padding:12px}
.trim{background:var(--card);border-radius:14px;padding:10px;display:flex;flex-direction:column;gap:8px;align-items:center}
.tpad{display:grid;grid-template-columns:repeat(3,40px);gap:6px}
.tpad button{padding:0;height:36px;border-radius:9px}
.tpad .tc{display:grid;place-items:center;color:var(--dim)}
.tk{font-size:13px;color:var(--dim)}
.tk input{width:44px;font:inherit;text-align:center;border-radius:8px;border:1px solid var(--line);background:#0f1318;color:var(--fg)}''')
rep('''#arm{font-size:24px;padding:22px 0;width:100%}''', '''#arm{font-size:22px;padding:18px 0;width:100%}''')
open(p, 'w', encoding='utf-8').write(t)
print('ok')
