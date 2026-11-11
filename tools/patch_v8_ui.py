import sys, re
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
def rep(a, b):
    global t
    assert t.count(a) == 1, a; t = t.replace(a, b)

# 1) 4 个勾 → 1 个确认按钮（每次打开页面要按一次）
rep('''      <div class="checks" id="checks">
        <div class="tog" data-c="prop"><span class="box"></span>🪶 桨装回</div>
        <div class="tog" data-c="space"><span class="box"></span>🌳 空旷地方</div>
        <div class="tog" data-c="people"><span class="box"></span>🚷 人离开 1 米</div>
        <div class="tog" data-c="cable"><span class="box"></span>➰ USB 线放松</div>
      </div>''', '''      <div class="tog" id="confirm" style="justify-content:center;font-size:18px"><span class="box"></span>✅ 确认：桨装好 · 空旷 · 人离开 · 线放松</div>''')
rep('''const checks=lsGet('fly.checks',{});
$$('#checks .tog').forEach(t=>{ if(checks[t.dataset.c]) t.classList.add('on');
  t.addEventListener('click',()=>{t.classList.toggle('on');checks[t.dataset.c]=t.classList.contains('on');lsSet('fly.checks',checks);}); });''',
    '''let confirmed=false;
$('#confirm').addEventListener('click',()=>{confirmed=!confirmed;$('#confirm').classList.toggle('on',confirmed);});''')
rep('''  const miss=['prop','space','people','cable'].filter(k=>!checks[k]);
  if(miss.length){toast('先把上面 4 个勾上',1);$('#checks').classList.add('flash');setTimeout(()=>$('#checks').classList.remove('flash'),500);return;}''',
    '''  if(!confirmed){toast('先点上面的“确认”',1);$('#confirm').classList.add('flash');setTimeout(()=>$('#confirm').classList.remove('flash'),500);return;}''')

# 2) Enter 只在起飞页、确认过才有效；不再在 Enter 里做地面防自转检查（改由飞控在空中测一次）
i = t.index('let tkBusy=false;')
j = t.index('function armedUI(on){')
t = t[:i] + '''function takeoff(){
  if(!S.on||cur!==3) return;
  if(!confirmed){toast('先点“确认”',1);$('#confirm').classList.add('flash');setTimeout(()=>$('#confirm').classList.remove('flash'),500);return;}
  if(st.armed){ takeoffJump(); return; }
  pendingTk=true; $('#arm').click(); setTimeout(()=>{pendingTk=false;},1500);
}
''' + t[j:]
rep('''  if(e.key==='Enter'){e.preventDefault();takeoff();return;}''', '''  if(e.key==='Enter'){e.preventDefault();if(cur===3)takeoff();return;}''')

# 3) 偏航（防自转）交给飞控自己管：页面不再写 yp / yi
rep('''  // 偏航（防自转）只有方向检查通过后才开，否则方向反了会越转越快
  if(lsGet('fly.yawOK',false)){ p.yp=BASE.ypOn||0.8; p.yi=BASE.yiOn||0.5; } else { p.yp=0; p.yi=0; }''', '''  delete p.yp; delete p.yi;   // 偏航由飞控自己定：空中测一次方向，测准后自己打开并保存''')

# 4) 偏航角 + 自转速度显示
rep('''    D.roll=m.roll||0;D.pitch=m.pitch||0;return;''', '''    D.roll=m.roll||0;D.pitch=m.pitch||0;if(m.yaw!=null){D.yaw=m.yaw;D.gy=m.gy;}return;''')
rep('''        <div class="level sm"><div class="ring"></div><div class="bub" id="bub2"></div></div>''',
    '''        <div class="level sm"><div class="ring"></div><div class="bub" id="bub2"></div></div>
        <div style="display:flex;flex-direction:column;align-items:center;gap:4px">
          <svg viewBox="-50 -50 100 100" width="96" height="96"><circle r="46" fill="#1f2833" stroke="#2a333e" stroke-width="2"/>
            <g id="yawNeedle"><path d="M0 -40 L7 0 L0 -6 L-7 0 Z" fill="#3b9eff"/></g></svg>
          <div class="hint" style="font-size:13px" id="yawTxt">偏航 --</div>
        </div>''')
rep('''    for(let i=0;i<4;i++){const w=Math.min(100,D.out[i]/7);$('#o'+(i+1)).style.width=w+'%';}''',
    '''    for(let i=0;i<4;i++){const w=Math.min(100,D.out[i]/7);$('#o'+(i+1)).style.width=w+'%';}
    if(D.yaw!=null){ $('#yawNeedle').setAttribute('transform','rotate('+(+D.yaw)+')');
      const spin=Math.abs(+D.gy||0); $('#yawTxt').textContent='偏航 '+Math.round(D.yaw)+'° · 自转 '+Math.round(+D.gy||0)+'°/s';
      $('#yawTxt').style.color=spin>60?'var(--bad)':''; }''')
open(p, 'w', encoding='utf-8').write(t)
print('ok')
