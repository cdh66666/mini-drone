import sys
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
def rep(a, b):
    global t
    assert t.count(a) == 1, a; t = t.replace(a, b)

# 1) 防自转检查：4 个电机一起判断，函数化，Enter 首次起飞前自动做
i = t.index("$('#btnYaw').addEventListener('click',async()=>{")
j = t.index("/* 电机测试 */")
t = t[:i] + r'''/* 防自转方向：电机加速时机身被反扭。按混控约定 M1、M4 应扭向“正”，M2、M3 扭向“负”。
   s = m1 - m2 - m3 + m4：明显为正 → 方向对；明显为负 → 翻 sgY；太小 → 没测出来（不开防自转）。 */
async function yawCheck(){
  const s=$('#yawSt');s.textContent='⏳ 防自转检查：1~4 号各轻转 0.5 秒…';toast('🌀 防自转检查中，别碰');
  let j=null;try{j=await (await fetch('/api/yawtest?pm=270&ms=500')).json();}catch(e){}
  lsSet('fly.yawTried',true);
  if(!j||j.err||j.m1==null){s.textContent='✗ 没测成';return false;}
  const sc=j.m1-j.m2-(j.m3||0)+(j.m4||0);
  if(Math.abs(sc)<1.0){s.textContent='✗ 没测出来（'+sc.toFixed(1)+'）';toast('防自转没测出来，先不开',1);return false;}
  if(sc<0){ send('E,sgY,'+(-j.sgY)); saveIfSafe(); }
  lsSet('fly.yawOK',true); pushParams();
  s.textContent='✓ 防自转已打开'; toast('🌀 防自转已打开 ✓'); return true;
}
$('#btnYaw').addEventListener('click',()=>{
  if(!S.on||!SRV){toast('先连接',1);return;}
  if(st.armed){toast('先上锁',1);return;}
  yawCheck();
});

''' + t[j:]

# 2) 4 个勾记住；Enter 在任何一步都能起飞；首次起飞前自动防自转检查
rep('''const checks={};
$$('#checks .tog').forEach(t=>t.addEventListener('click',()=>{t.classList.toggle('on');checks[t.dataset.c]=t.classList.contains('on');}));''',
    '''const checks=lsGet('fly.checks',{});
$$('#checks .tog').forEach(t=>{ if(checks[t.dataset.c]) t.classList.add('on');
  t.addEventListener('click',()=>{t.classList.toggle('on');checks[t.dataset.c]=t.classList.contains('on');lsSet('fly.checks',checks);}); });''')
rep('''function takeoff(){
  if(cur!==3){toast('先到“起飞”这一步',1);return;}
  if(st.armed){ takeoffJump(); return; }
  pendingTk=true; $('#arm').click(); setTimeout(()=>{pendingTk=false;},1500);
}''', '''let tkBusy=false;
async function takeoff(){
  if(!S.on){toast('先连接',1);return;}
  if(cur!==3) go(3);
  if(st.armed){ takeoffJump(); return; }
  if(tkBusy) return; tkBusy=true;
  try{
    if(SRV && srv.mode==='wifi' && !lsGet('fly.yawTried',false)) await yawCheck();
    ['prop','space','people','cable'].forEach(k=>{checks[k]=true;}); lsSet('fly.checks',checks);
    $$('#checks .tog').forEach(t=>t.classList.add('on'));
    pendingTk=true; $('#arm').click(); setTimeout(()=>{pendingTk=false;},1500);
  } finally { tkBusy=false; }
}''')

# 3) 方向键温柔一点：0.35（≈8.7°）→ 0.14（≈3.5°）
rep('''  const T=0.35;''', '''  const T=0.14;   // 杆量 × 最大倾角 25° ≈ 3.5°''')

# 4) 校准过就记住，连上直接到“起飞”
rep('''  toast('连上了 ✓'); go(1);''', '''  toast('连上了 ✓'); go(lsGet('fly.ready',false)?3:1);''')
rep('''  if(ok){saveIfSafe();toast('校准好了 ✓');markDone(1);}''', '''  if(ok){saveIfSafe();lsSet('fly.ready',true);toast('校准好了 ✓ 已存进飞机');markDone(1);}''')
open(p, 'w', encoding='utf-8').write(t)
print('ok')
