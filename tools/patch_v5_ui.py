import sys
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
def rep(a, b):
    global t
    assert t.count(a) == 1, a; t = t.replace(a, b)
rep("    D.vb=+a[2];D.usb=a[3]==='0';D.out=[+a[10],+a[11],+a[12],+a[13]];",
    "    D.vb=+a[2];D.usb=a[3]==='0';D.out=[+a[10],+a[11],+a[12],+a[13]];D.ax=+a[4];D.ay=+a[5];D.az=+a[6];")
i = t.index('/* 方向检查')
j = t.index('/* 电机测试 */')
new = r'''/* 方向检查（v5）：用原始加速度算出传感器芯片实际转了多少度，自动写 rot / sgR / sgP。
   约定（与混控一致）：抬 1、2 号那边 → pitch 为负；抬 1、3 号那边 → roll 为正。 */
let dirStep=0,dirRes={},dirT=0,dirOn=false,dirBase=null;
const DIRS=[{k:'nose',txt:'⬆ 把 1、2 号那边抬高一点'},{k:'left',txt:'⬅ 把 1、3 号那边抬高一点'}];
function dirMsg(s){$('#angTxt').textContent=s;$('#angTxt').style.fontSize='22px';}
$('#btnDir').addEventListener('click',()=>{
  if(!S.on){toast('先连接',1);return;}
  if(D.ax==null){toast('还没收到数据，等一秒再点',1);return;}
  dirBase=[D.ax,D.ay];dirStep=0;dirRes={};dirAsk();
});
function dirAsk(){
  dirOn=true;dirMsg(DIRS[dirStep].txt);
  let hold=0,acc=[0,0],n=0;clearInterval(dirT);
  dirT=setInterval(()=>{
    const dx=D.ax-dirBase[0],dy=D.ay-dirBase[1],m=Math.hypot(dx,dy);
    if(m>0.17){hold++;acc[0]+=dx;acc[1]+=dy;n++;} else {hold=0;acc=[0,0];n=0;}
    if(hold>=5){ clearInterval(dirT);
      dirRes[DIRS[dirStep].k]=[acc[0]/n,acc[1]/n];
      dirMsg('✓ 放下');
      setTimeout(()=>{ if(++dirStep<DIRS.length){ dirBase=[D.ax,D.ay]; dirAsk(); } else dirDone(); },1800);
    }
  },120);
}
function wrap(a){while(a>180)a-=360;while(a<-180)a+=360;return a;}
function dirDone(){
  dirOn=false;$('#angTxt').style.fontSize='';
  const F=dirRes.nose,L=dirRes.left,deg=Math.PI/180;
  const aF=Math.atan2(F[1],F[0])/deg,aL=Math.atan2(L[1],L[0])/deg;
  const sep=Math.abs(wrap(aF-aL));
  if(sep<55||sep>125){ toast('两次抬的方向太接近，再测一次（抬一整条边）',1); return; }
  const tL=wrap(90-aL);
  const opts=[{sgP:1,t:wrap(-aF)},{sgP:-1,t:wrap(180-aF)}].map(o=>({...o,d:Math.abs(wrap(o.t-tL))}));
  opts.sort((x,y)=>x.d-y.d);const o=opts[0];
  if(o.d>35){ toast('没测准，再测一次',1); return; }
  const rot=Math.round(wrap(tL+wrap(o.t-tL)/2));
  send('E,rot,'+rot);send('E,sgR,1');send('E,sgP,'+o.sgP);send('E,trimR,0');send('E,trimP,0');
  saveIfSafe();
  try{ fetch('/api/note?t='+encodeURIComponent('dir F='+F.map(v=>v.toFixed(3))+' L='+L.map(v=>v.toFixed(3))+' rot='+rot+' sgP='+o.sgP)); }catch(e){}
  toast('芯片转了 '+rot+'°，已改好 ✓ 放平后点“校准水平”');
  $('#angTxt').textContent='转 '+rot+'° ✓';
}

'''
t = t[:i] + new + t[j:]
open(p, 'w', encoding='utf-8').write(t)
print('ok')
