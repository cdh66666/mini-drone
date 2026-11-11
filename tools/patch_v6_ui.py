import sys
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
def rep(a, b):
    global t
    assert t.count(a) == 1, a; t = t.replace(a, b)

# ---- 1. 自转方向检查（第 3 步，桨装着） ----
rep('''        <button class="ghost" id="btnAll">四个一起 1.5 秒</button>
        <button class="big okb" id="next2">都对 →</button>
      </div>''', '''        <button class="ghost" id="btnAll">四个一起 1.5 秒</button>
        <button class="big okb" id="next2">都对 →</button>
      </div>
      <div class="card" style="display:flex;flex-direction:column;gap:8px">
        <button class="go" id="btnYaw">🌀 防自转检查</button>
        <div class="hint" style="font-size:13px">桨装着、放桌上、手离开 → 点一下（1、2 号各轻转 0.7 秒，不会飞）</div>
        <div class="hint" id="yawSt"></div>
      </div>''')
rep('''function pushParams(save){
  const p=calcPid(+$('#sA').value,+$('#sB').value);''', '''function pushParams(save){
  const p=calcPid(+$('#sA').value,+$('#sB').value);
  // 偏航（防自转）只有方向检查通过后才开，否则方向反了会越转越快
  if(lsGet('fly.yawOK',false)){ p.yp=BASE.ypOn||0.8; p.yi=BASE.yiOn||0.5; } else { p.yp=0; p.yi=0; }''')
rep('''  ['angP','rp','ri','rd','pp','pi','pd','yp','idle','mthr','iThr'].forEach(k=>{ if(p[k]!=null) send('E,'+k+','+p[k]); });''',
    '''  ['angP','rp','ri','rd','pp','pi','pd','yp','yi','idle','mthr','iThr'].forEach(k=>{ if(p[k]!=null) send('E,'+k+','+p[k]); });''')
rep('''/* 电机测试 */''', r'''/* 防自转方向：点动 M1 时机身应被扭向“正”，M2 应为“负”（与混控 oY 一致）；反了就翻 sgY */
$('#btnYaw').addEventListener('click',async()=>{
  if(!S.on||!SRV){toast('先连接',1);return;}
  if(st.armed){toast('先上锁',1);return;}
  const b=$('#btnYaw'),s=$('#yawSt');b.disabled=true;s.textContent='⏳ 转 1 号、2 号…';
  let j=null;try{j=await (await fetch('/api/yawtest?pm=260&ms=700')).json();}catch(e){}
  b.disabled=false;
  if(!j||j.err||j.m1==null){s.textContent='✗ 没测成，再点一次';return;}
  const ok=Math.abs(j.m1)>0.3&&Math.abs(j.m2)>0.3&&Math.sign(j.m1)!==Math.sign(j.m2);
  if(!ok){s.textContent='✗ 没测出来（'+j.m1+' / '+j.m2+'），手离开再点一次';return;}
  if(j.m1<0){ send('E,sgY,'+(-j.sgY)); saveIfSafe(); }
  lsSet('fly.yawOK',true); pushParams();
  s.textContent='✓ 防自转已打开'+(j.m1<0?'（方向已自动改）':'');toast('防自转已打开 ✓');
});

/* 电机测试 */''')

# ---- 2. Enter 一键起飞（值可调） ----
rep('''      <div class="hint">W / S</div>''', '''      <div class="hint">W / S</div>
      <div class="hint" style="font-size:13px;text-align:center">⏎ 一键<br><input id="tkoff" type="number" min="20" max="90" step="1" style="width:56px;font:inherit;text-align:center;border-radius:8px;border:1px solid var(--line);background:#0f1318;color:var(--fg)">%</div>''')
rep('''function armedUI(on){
  st.armed=on;const b=$('#arm');''', '''const tk=$('#tkoff');tk.value=lsGet('fly.tkoff',60);
tk.addEventListener('change',()=>{const v=Math.max(20,Math.min(90,+tk.value||60));tk.value=v;lsSet('fly.tkoff',v);tk.blur();});
let pendingTk=false;
function takeoffJump(){ const v=+tk.value||60; st.landing=false; st.target=v; st.cmd=v; thr.value=v; toast('⏎ 起飞 '+v+'%'); }
function takeoff(){
  if(cur!==3){toast('先到“起飞”这一步',1);return;}
  if(st.armed){ takeoffJump(); return; }
  pendingTk=true; $('#arm').click(); setTimeout(()=>{pendingTk=false;},1500);
}
function armedUI(on){
  st.armed=on;const b=$('#arm');
  if(on&&pendingTk){ pendingTk=false; setTimeout(takeoffJump,200); }''')
rep('''  if(cur===3&&(e.key==='w'||e.key==='W')){''', '''  if(e.key==='Enter'){e.preventDefault();takeoff();return;}
  if(cur===3&&(e.key==='w'||e.key==='W')){''')
open(p, 'w', encoding='utf-8').write(t)
print('ok')
