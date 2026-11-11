import sys
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
def rep(a, b):
    global t
    assert t.count(a) == 1, a; t = t.replace(a, b)

# 1) 基准参数可由 tune.json（服务 /api/tune）覆盖 —— Claude 调好的参数放这里
rep('''          yp:BASE.yp,idle:BASE.idle,mthr:BASE.mthr};''', '''          yp:BASE.yp,idle:BASE.idle,mthr:BASE.mthr,iThr:BASE.iThr};''')
rep('''const BASE={angP:6.0,rp:3.0,ri:0.15,rd:0.06,pp:3.0,pi:0.15,pd:0.06,yp:0,idle:0,mthr:700};''',
    '''const BASE={angP:6.0,rp:3.0,ri:0.15,rd:0.06,pp:3.0,pi:0.15,pd:0.06,yp:0,idle:0,mthr:700,iThr:250};
if(/^https?:$/.test(location.protocol)) fetch('/api/tune').then(r=>r.json()).then(j=>{Object.assign(BASE,j);}).catch(()=>{});''')
rep('''  ['angP','rp','ri','rd','pp','pi','pd','yp','idle','mthr'].forEach(k=>send('E,'+k+','+p[k]));''',
    '''  ['angP','rp','ri','rd','pp','pi','pd','yp','idle','mthr','iThr'].forEach(k=>{ if(p[k]!=null) send('E,'+k+','+p[k]); });''')
rep('''function gotParams(j){''', '''let curP=null;
function gotParams(j){
  if(j&&typeof j.angP==='number') curP=j;''')

# 2) 方向检查（第 2 步）
rep('''      <button class="big okb" id="next1">下一步 →</button>''', '''      <button class="big ghost" id="btnDir">🖐 方向检查</button>
      <button class="big okb" id="next1">下一步 →</button>''')
rep('''/* 电机测试 */''', r'''/* 方向检查：抬机头 → pitch 应变大；抬左边 → roll 应变大。反了自动改符号 */
let dirStep=0,dirRes={},dirT=0;
const DIRS=[{k:'nose',txt:'⬆ 把机头那边（1、2 号）抬高一点',want:'pitch'},{k:'left',txt:'⬅ 把左边（1、3 号）抬高一点',want:'roll'}];
$('#btnDir').addEventListener('click',()=>{
  if(!S.on){toast('先连接',1);return;}
  if(!curP){ send('C'); }
  dirStep=0;dirRes={};dirAsk();
});
function dirAsk(){
  const d=DIRS[dirStep];$('#angTxt').textContent=d.txt;$('#angTxt').style.fontSize='22px';
  let hold=0;clearInterval(dirT);
  dirT=setInterval(()=>{
    const r=D.roll,p=D.pitch,m=Math.max(Math.abs(r),Math.abs(p));
    if(m>10){ hold++; } else hold=0;
    if(hold>=4){ clearInterval(dirT);
      const main=(Math.abs(p)>=Math.abs(r))?'pitch':'roll', v=main==='pitch'?p:r;
      dirRes[d.k]={main,pos:v>0};
      $('#angTxt').textContent='✓ 放下';
      setTimeout(()=>{ if(++dirStep<DIRS.length) dirAsk(); else dirDone(); },1500);
    }
  },150);
}
function dirDone(){
  $('#angTxt').style.fontSize='';
  const n=dirRes.nose,l=dirRes.left;
  if(n.main!=='pitch'||l.main!=='roll'){ toast('传感器方向转了 90°，把截图发给 Claude',1); return; }
  const fix=[];
  if(!n.pos) fix.push('sgP'); if(!l.pos) fix.push('sgR');
  if(!fix.length){ toast('方向全对 ✓'); markDone(1); return; }
  fix.forEach(k=>{ const cur=(curP&&curP[k])||1; send('E,'+k+','+(-cur)); if(curP) curP[k]=-cur; });
  saveIfSafe(); toast('方向反了，已自动改好 ✓ 再测一次确认');
}

/* 电机测试 */''')

# 3) 黑匣子保存提示
rep('''  if(l.startsWith('#LQ,')){''', '''  if(l.startsWith('#BB,saved')){ toast('📦 这次飞行已记录'); return; }
  if(l.startsWith('#LQ,')){''')
open(p, 'w', encoding='utf-8').write(t)
print('ok')
