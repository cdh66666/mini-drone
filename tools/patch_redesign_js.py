import sys
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
def rep(a, b):
    global t
    assert t.count(a) == 1, a[:80]; t = t.replace(a, b)

# WiFi 信号格：信号强度 + 丢包
rep('''  if(l.startsWith('#LQ,')){ const a=l.split(','),loss=+a[1],gap=+a[4],m=$('#mode');
    const bad=loss>30||gap>600; m.textContent=bad?'📶 信号差':'📶 WiFi'; m.style.background=bad?'#5a3a00':''; return; }''',
    '''  if(l.startsWith('#LQ,')){ const a=l.split(','),loss=+a[1],rssi=+a[3],gap=+a[4];
    let q=rssi>-60?4:rssi>-70?3:rssi>-80?2:1; if(loss>10||gap>300)q=Math.min(q,2); if(loss>30||gap>600)q=1;
    wifiQ(q,rssi,loss); return; }''')
rep('''    this.mode=j.mode;$('#mode').textContent=j.mode==='wifi'?'📶 WiFi':'🔌 USB';$('#mode').style.display='';''',
    '''    this.mode=j.mode;$('#mode').innerHTML=j.mode==='wifi'?wifiHTML(4):'🔌 USB';$('#mode').style.display='';''')

# 电量
rep('''    const v=D.vb/1000;
    $('#vb').textContent=v.toFixed(2)+' V';
    $('#vb').style.color=(!D.usb&&v<3.45)?'var(--bad)':'';''', '''    batShow();''')

rep('''function frame(){''', r'''/* ---------- 电量：1S 锂电静置电压 → 百分比；飞行中电压被拉低，只显示飞前的数 ---------- */
const OCV=[[4.20,100],[4.15,95],[4.11,90],[4.08,85],[4.02,80],[3.98,75],[3.95,70],[3.91,65],[3.87,60],[3.85,55],[3.84,50],[3.82,45],[3.80,40],[3.79,35],[3.77,30],[3.75,25],[3.73,20],[3.71,15],[3.69,10],[3.61,5],[3.30,0]];
function pctOf(v){ if(v>=4.2)return 100; for(let i=1;i<OCV.length;i++){const[a,pa]=OCV[i-1],[b,pb]=OCV[i]; if(v>=b) return Math.round(pb+(v-b)/(a-b)*(pa-pb));} return 0; }
const CH={hist:[],lastPct:null};
function batShow(){
  const v=D.vb/1000; if(!(v>2.5))return;
  // 充电时电压虚高约 0.1 V，扣掉再查表
  let pct=pctOf(D.usb?v-0.10:v);
  if(st.armed&&CH.lastPct!=null) pct=CH.lastPct; else CH.lastPct=pct;
  $('#batPct').textContent=pct+'%'; $('#vb').textContent=v.toFixed(2)+' V';
  const f=$('#batFill'); f.style.width=(pct*0.22)+'px'; f.style.background=pct<20?'var(--bad)':pct<40?'var(--warn)':'var(--ok)';
  // 充电预计：记最近 10 分钟的电压，按上升速度推到 4.15 V，再加 10 分钟恒压收尾
  const now=Date.now()/1000;
  if(D.usb){ CH.hist.push([now,v]); while(CH.hist.length&&now-CH.hist[0][0]>600) CH.hist.shift(); }
  else CH.hist=[];
  let s='';
  if(D.usb){
    if(v>=4.17) s='⚡ 快满了';
    else if(CH.hist.length>20&&now-CH.hist[0][0]>90){
      const h=CH.hist,n=h.length;let sx=0,sy=0,sxx=0,sxy=0;h.forEach(([x,y])=>{x-=h[0][0];sx+=x;sy+=y;sxx+=x*x;sxy+=x*y;});
      const k=(n*sxy-sx*sy)/(n*sxx-sx*sx);   // V/s
      if(k>0.000005){ const min=Math.round((4.15-v)/k/60+10); s='⚡ 约 '+(min>90?Math.round(min/6)/10+' 小时':min+' 分钟')+'充满'; }
      else s='⚡ 充电中';
    } else s='⚡ 充电中（估算中）';
  } else if(!st.armed&&v<3.6) s='⚠ 该充电了';
  $('#chg').textContent=s;
}
function wifiHTML(q){ return '<span class="wq">'+[1,2,3,4].map(i=>'<i style="height:'+(3+i*3)+'px;background:'+(i<=q?(q>=3?'var(--ok)':q==2?'var(--warn)':'var(--bad)'):'#3d4a58')+'"></i>').join('')+'</span> WiFi'; }
function wifiQ(q,rssi,loss){ const m=$('#mode'); if(srv.mode!=='wifi')return; m.innerHTML=wifiHTML(q); m.title='信号 '+rssi+' dBm · 丢包 '+loss+'%'; }

/* ---------- 配平：点一下 0.5°，马上生效并存进飞机 ---------- */
function trimShow(){ if(!curP){$('#trimTxt').textContent='--';return;}
  const r=curP.trimR||0,p=curP.trimP||0; $('#trimTxt').textContent='前后 '+p.toFixed(1)+'° · 左右 '+r.toFixed(1)+'°'; }
document.querySelectorAll('.tpad button').forEach(b=>b.addEventListener('click',()=>{
  if(!S.on){toast('先连接',1);return;} if(!curP){send('C');toast('读参数中，再点一次');return;}
  const [ax,dir]=b.dataset.t.split(','),k=ax==='P'?'trimP':'trimR';
  curP[k]=Math.round(((curP[k]||0)+0.5*(+dir))*100)/100;
  send('E,'+k+','+curP[k]); saveIfSafe(); trimShow();
  toast((ax==='P'?(+dir>0?'往前':'往后'):(+dir>0?'往右':'往左'))+' 0.5° ✓');
}));
setInterval(trimShow,1000);

function frame(){''')
# 样式：信号格
rep('''.dim{color:var(--dim)}''', '''.dim{color:var(--dim)}
.wq{display:inline-flex;align-items:flex-end;gap:2px;height:15px;margin-right:4px}.wq i{display:block;width:4px;border-radius:1px}''')
open(p, 'w', encoding='utf-8').write(t)
print('ok')
