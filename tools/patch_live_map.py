import sys
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
def rep(a, b):
    global t
    assert t.count(a) == 1, a; t = t.replace(a, b)

# 起飞页：实时推算轨迹（俯视）+ 高度条 + 历史回放按钮
rep('''        <div class="level sm"><div class="ring"></div><div class="bub" id="bub2"></div></div>''',
    '''        <div class="level sm"><div class="ring"></div><div class="bub" id="bub2"></div></div>
        <div style="display:flex;flex-direction:column;align-items:center;gap:4px">
          <div style="display:flex;gap:6px;align-items:flex-end">
            <canvas id="liveMap" width="300" height="300" style="width:150px;height:150px;background:#141a21;border-radius:10px"></canvas>
            <div style="width:10px;height:150px;background:#141a21;border-radius:5px;position:relative"><div id="liveZ" style="position:absolute;left:0;right:0;bottom:0;height:0;background:#2fc47a;border-radius:5px"></div></div>
          </div>
          <div class="hint" style="font-size:12px" id="liveTxt">推算轨迹（粗估）</div>
          <button class="ghost" id="btnHist" style="padding:4px 10px;font-size:13px">▶ 历史回放</button>
        </div>''')
rep('''    D.roll=m.roll||0;D.pitch=m.pitch||0;if(m.yaw!=null){D.yaw=m.yaw;D.gy=m.gy;}return;''',
    '''    D.roll=m.roll||0;D.pitch=m.pitch||0;if(m.yaw!=null){D.yaw=m.yaw;D.gy=m.gy;}liveStep();return;''')
rep('''  if(on&&pendingTk){ pendingTk=false; setTimeout(takeoffJump,200); }''',
    '''  if(on&&pendingTk){ pendingTk=false; setTimeout(takeoffJump,200); }
  if(on) liveReset();''')
rep('''/* ---------------- 飞行 ---------------- */''', r'''/* ---------------- 实时推算轨迹（和回放页同一套算法：倾角 → 水平加速度，加阻力，二次积分） ---------------- */
const LV={p:[0,0,0],v:[0,0,0],t:0,path:[]};
function liveReset(){LV.p=[0,0,0];LV.v=[0,0,0];LV.t=0;LV.path=[[0,0]];}
function liveStep(){
  const now=performance.now()/1000;const dt=LV.t?Math.min(0.2,now-LV.t):0;LV.t=now;
  if(!st.armed||!dt)return;
  const d=Math.PI/180,g=9.81,k=1.2,r=D.roll*d,q=D.pitch*d,y=(D.yaw||0)*d;
  const af=g*Math.tan(q),ar=g*Math.tan(r);               // 前、右
  const ax=af*Math.cos(y)-ar*Math.sin(y),ay=af*Math.sin(y)+ar*Math.cos(y);
  const air=(st.cmd||0)>=45;
  let az=0; if(air&&D.az!=null) az=(D.az*Math.cos(r)*Math.cos(q)-1)*g;
  const a=[air?ax:0,air?ay:0,az];
  for(let i=0;i<3;i++){LV.v[i]+=(a[i]-k*LV.v[i])*dt;LV.p[i]+=LV.v[i]*dt;}
  if(LV.p[2]<0){LV.p[2]=0;LV.v[2]=Math.max(0,LV.v[2]);}
  if(!air){LV.v=[0,0,0];}
  LV.path.push([LV.p[0],LV.p[1]]);if(LV.path.length>3000)LV.path.shift();
  liveDraw();
}
function liveDraw(){
  const c=$('#liveMap');if(!c)return;const g=c.getContext('2d'),W=300,H=300;g.clearRect(0,0,W,H);
  let R=1;LV.path.forEach(p=>{R=Math.max(R,Math.abs(p[0])*1.2,Math.abs(p[1])*1.2);});
  const X=p=>W/2+p[1]/R*W/2,Y=p=>H/2-p[0]/R*H/2;
  g.strokeStyle='#2a333e';for(let m=-Math.floor(R);m<=Math.floor(R);m++){g.beginPath();g.moveTo(W/2+m/R*W/2,0);g.lineTo(W/2+m/R*W/2,H);g.stroke();g.beginPath();g.moveTo(0,H/2-m/R*H/2);g.lineTo(W,H/2-m/R*H/2);g.stroke();}
  g.strokeStyle='#2fc47a';g.lineWidth=4;g.beginPath();LV.path.forEach((p,i)=>i?g.lineTo(X(p),Y(p)):g.moveTo(X(p),Y(p)));g.stroke();
  const cur=LV.path[LV.path.length-1]||[0,0];g.fillStyle='#ff4d4f';g.beginPath();g.arc(X(cur),Y(cur),9,0,7);g.fill();
  $('#liveZ').style.height=Math.min(100,LV.p[2]/2*100)+'%';
  $('#liveTxt').textContent='前 '+LV.p[0].toFixed(1)+' · 右 '+LV.p[1].toFixed(1)+' · 高 '+LV.p[2].toFixed(1)+' m';
}
$('#btnHist').addEventListener('click',()=>{ if(st.armed){toast('先降落',1);return;} location.href='/replay.html'; });
liveReset();

/* ---------------- 飞行 ---------------- */''')
open(p, 'w', encoding='utf-8').write(t)
print('ok')
