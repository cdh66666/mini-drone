import sys
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
def rep(a, b):
    global t
    assert t.count(a) == 1, a; t = t.replace(a, b)
rep('''    <button class="big go" id="btnConn">连接飞机</button>
    <div class="hint">🔌 插 USB　或　📶 电脑 WiFi 连 “DroneBench”</div>
  </div>''', '''    <button class="big go" id="btnConn">连接飞机</button>
    <div class="hint" id="connHint">🔌 插 USB 就行；飞机连过 WiFi 后不用插线</div>
    <div class="row" id="netBox" style="display:none;align-items:flex-start;gap:16px">
      <div class="card" style="width:380px;display:flex;flex-direction:column;gap:10px">
        <div class="row" style="justify-content:space-between"><b>📶 飞机连 WiFi</b><span class="hint" id="netSt">--</span></div>
        <button class="ghost" id="btnScan">🔍 找 WiFi</button>
        <div id="scanList" style="display:flex;flex-direction:column;gap:6px;max-height:220px;overflow:auto"></div>
        <div class="row" id="pwRow" style="display:none;gap:8px">
          <input id="pw" type="password" placeholder="密码" style="flex:1;font:inherit;padding:10px;border-radius:10px;border:1px solid var(--line);background:#0f1318;color:var(--fg)">
          <button class="go" id="btnJoin">连上</button>
        </div>
        <div class="hint" style="font-size:13px">只能连 2.4G。列表里没有你的 WiFi → 电脑开“移动热点”（2.4GHz）再找</div>
      </div>
      <div class="card" style="width:240px;display:flex;flex-direction:column;gap:10px">
        <b>⬆ 固件</b>
        <div class="hint" id="fwInfo">--</div>
        <button class="ghost" id="btnFlash">更新固件</button>
      </div>
    </div>
  </div>''')
rep('''  if(l.startsWith('#VER,')){D.ver=l.split(',')[1];return;}''', '''  if(l.startsWith('#VER,')){D.ver=l.split(',')[1];return;}
  if(l.startsWith('#NET,')){netShow(l);return;}
  if(l.startsWith('#SCAN,')){scanLine(l);return;}
  if(l.startsWith('#ACK,NET')){$('#netSt').textContent='⏳ 连接中…';return;}''')
rep('''function afterConnect(){
  setConn(true); send('V'); markDone(0);''', '''function afterConnect(){
  setConn(true); send('V'); markDone(0); netPanel();''')
rep('''/* ---------------- 飞机状态 ---------------- */''', '''/* ---------------- 飞机连 WiFi / 更新固件 ---------------- */
function hexOf(s){return [...new TextEncoder().encode(s)].map(b=>b.toString(16).padStart(2,'0')).join('').toUpperCase();}
function strOf(h){const b=new Uint8Array((h.match(/../g)||[]).map(x=>parseInt(x,16)));return new TextDecoder().decode(b);}
let pickSsid='';const seen={};
function netPanel(){
  const usb=!SRV||srv.mode!=='wifi';
  $('#netBox').style.display=SRV?'':'none';
  $('#btnScan').disabled=!usb;$('#btnScan').textContent=usb?'🔍 找 WiFi':'🔍 找 WiFi（要插 USB）';
  if(usb) send('N?'); else $('#netSt').textContent='✓ 无线连接中';
  if(SRV) fetch('/api/fwinfo').then(r=>r.json()).then(j=>{$('#fwInfo').textContent=j.fw?(j.fw.replace('.ino.bin','')+' · '+j.time):'没有固件';}).catch(()=>{});
}
function netShow(l){
  const a=l.split(',');const el=$('#netSt');
  if(a[1]==='ok'){el.textContent='✓ 已连 '+a[3];toast('飞机连上 WiFi ✓ 以后不用插线');}
  else if(a[1]==='wait') el.textContent='⏳ 连接中…'; else el.textContent='未设置';
}
function bars(r){return r>-55?'▂▄▆█':r>-67?'▂▄▆':r>-78?'▂▄':'▂';}
function scanLine(l){
  const a=l.split(',');const box=$('#scanList');
  if(a[1]==='end'){$('#btnScan').disabled=false;$('#btnScan').textContent='🔍 再找一次';if(!box.children.length)box.innerHTML='<div class="hint">没找到</div>';return;}
  const name=strOf(a[4]||'');if(!name||seen[name])return;seen[name]=1;
  const b=document.createElement('button');b.className='ghost';b.style.textAlign='left';
  b.textContent=bars(+a[1])+'  '+name+(a[3]==='0'?'':'  🔒');
  b.onclick=()=>{pickSsid=name;$$('#scanList button').forEach(x=>x.style.borderColor='');b.style.borderColor='var(--acc)';$('#pwRow').style.display='';$('#pw').value='';$('#pw').focus();};
  box.appendChild(b);
}
$('#btnScan').addEventListener('click',()=>{for(const k in seen)delete seen[k];$('#scanList').innerHTML='';$('#btnScan').disabled=true;$('#btnScan').textContent='⏳ 找…';send('NS');});
$('#btnJoin').addEventListener('click',()=>{ if(!pickSsid)return; send('N,'+hexOf(pickSsid)+','+hexOf($('#pw').value)); $('#pw').value=''; $('#pwRow').style.display='none'; });
$('#btnFlash').addEventListener('click',async()=>{
  if(st.armed){toast('先上锁',1);return;}
  const b=$('#btnFlash');b.disabled=true;b.textContent='⏳ 更新中，别拔…';
  let j=null;try{j=await (await fetch('/api/flash')).json();}catch(e){}
  b.disabled=false;b.textContent='更新固件';
  S.on=false;setConn(false);armedUI(false);
  if(j&&j.ok){toast('固件更新好了 ✓ 等 5 秒再点连接');}else toast('没更新成功：'+((j&&j.why)||'').slice(0,80),1);
});

/* ---------------- 飞机状态 ---------------- */''')
# 输入密码时，空格/W/S 不能被当成急停/油门
rep('''  if(e.code==='Space'){e.preventDefault();estop();return;}''', '''  if(e.target&&e.target.tagName==='INPUT'&&e.target.type!=='range'){ if(e.key==='Enter'&&e.target.id==='pw') $('#btnJoin').click(); return; }
  if(e.code==='Space'){e.preventDefault();estop();return;}''')
rep('''  if(e.code==='Space'){e.preventDefault();return;}''', '''  if(e.target&&e.target.tagName==='INPUT'&&e.target.type!=='range') return;
  if(e.code==='Space'){e.preventDefault();return;}''')
open(p, 'w', encoding='utf-8').write(t)
print('ok')
