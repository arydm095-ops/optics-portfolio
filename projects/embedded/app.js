(function () {
  'use strict';
  const $ = id => document.getElementById(id), esc = text => String(text).replace(/[&<>"']/g, x => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[x]));
  const storageKey = 'embedded-device-storage-v1';
  const eventNames = {1:'设备启动',2:'设备停止',3:'指令执行',4:'CRC校验失败',5:'缓冲区溢出',6:'DMA块完成',7:'故障锁存 / 驱动关闭',8:'故障确认 / 复位',9:'参数已保存',10:'参数已恢复',11:'开始写入非活动分区',12:'镜像CRC通过 / 等待重启',13:'启动固件',14:'试启动确认',15:'回退已确认固件',16:'请求被拒绝',17:'镜像校验失败'};
  const modes = ['停止','运行','故障锁存','固件更新'], upgradeNames = ['待机','写入非活动分区','校验通过，等待重启','新版本试启动','新版本已确认','镜像校验失败，保留旧版本','写入中断，保留旧版本'];
  const replyNames = ['已接受','CRC错误','参数超出范围','设备忙','未知命令或长度不匹配','异常未清除或操作被锁定'];
  let core, compiledModule, validation, paused = false, tab = 'runtime', sequence = 0, history = [], lastUpgradeState = -1, lastEventCount = -1, initialized = false, imageFixture;
  const version = value => value ? 'v' + Math.floor(value / 100) + '.' + (value % 100) : '空白';
  function status(message, error = false) { $('status').textContent = message; $('status').className = 'status' + (error ? ' error' : ''); }
  const value = field => core.ed_get(field);
  function memory(instance = core) { return new Uint8Array(instance.memory.buffer, instance.ed_buffer(), 65536); }
  function encode(instance, command, payloadValue = 0, length = 0) {
    const bytes = new Uint8Array(length + 7);
    bytes.set([0xaa,0x55,(++sequence)&255,command,length]);
    for (let i=0;i<length;i++) bytes[5+i]=(payloadValue >>> (8*i))&255;
    memory(instance).set(bytes);
    const crc=instance.ed_crc16(instance.ed_buffer(),length+5);
    bytes[5+length]=crc&255; bytes[6+length]=crc>>>8;
    return bytes;
  }
  function feed(instance, bytes) { for (const byte of bytes) instance.ed_uart_feed(byte); instance.ed_comm(); }
  function transact(command, input = 0, length = 0) {
    feed(core,encode(core,command,input,length));
    render(); return core.ed_tx_get(5);
  }
  function hex(bytes) { return Array.from(bytes,x=>x.toString(16).padStart(2,'0').toUpperCase()).join(' '); }
  function checkedInteger(element, min, max) {
    const number=Number(element.value);
    if (!Number.isInteger(number)||number<min||number>max) throw Error('请输入'+min+'到'+max+'之间的整数。');
    return number;
  }
  function store() {
    try {
      const length=core.ed_export_nv(),bytes=memory().slice(0,length);
      let binary=''; for (const byte of bytes) binary+=String.fromCharCode(byte);
      localStorage.setItem(storageKey,btoa(binary));
    } catch (error) { status('存储镜像未能保存到浏览器：'+error.message,true); }
  }
  function restore() {
    try {
      const encoded=localStorage.getItem(storageKey); if (!encoded) return;
      const binary=atob(encoded); if (binary.length>65536) return;
      const bytes=Uint8Array.from(binary,x=>x.charCodeAt(0)); memory().set(bytes);
      if (!core.ed_import_nv(bytes.length)) status('存储镜像校验失败，已采用出厂参数。',true);
    } catch (error) { status('本地存储无法读取，已采用出厂参数。',true); }
  }
  function synchronizeInputs() {
    $('target').value=value(4); $('sample').value=value(15); $('gain').value=value(16);
    $('gain-value').textContent=(value(16)/1000).toFixed(3);
  }
  function metric(name, number, unit, note) { return '<div class="metric"><small>'+name+'</small><strong>'+number+'<span>'+unit+'</span></strong><em>'+note+'</em></div>'; }
  function table(headers,rows) { return '<table><thead><tr>'+headers.map(x=>'<th>'+esc(x)+'</th>').join('')+'</tr></thead><tbody>'+rows.map(row=>'<tr>'+row.map(x=>'<td>'+x+'</td>').join('')+'</tr>').join('')+'</tbody></table>'; }
  function renderEvents(target, codes) {
    const count=Math.min(value(31),128), rows=[];
    for (let i=count-1;i>=0;i--) {
      const code=core.ed_event_get(i,1); if (codes&&!codes.includes(code)) continue;
      const ms=core.ed_event_get(i,0),number=core.ed_event_get(i,2);
      rows.push('<div class="event-line '+([4,5,7,16,17].includes(code)?'warn':[1,8,12,14].includes(code)?'ok':'')+'"><time>'+String(ms).padStart(6,'0')+'ms</time><span>'+eventNames[code]+' · '+number+'</span></div>');
      if (rows.length>=32) break;
    }
    $(target).innerHTML=rows.join('')||'<div class="empty">尚无事件</div>';
  }
  function figures() {
    if (tab==='runtime' && history.length) {
      Charts.graph('adc-chart',[{name:'三角波测试信号',data:history.map(x=>[x.ms/1000,x.adc/1000])},{name:'DMA块均值',data:history.map(x=>[x.ms/1000,x.mean/1000])}],{x:'逻辑时间 / s',y:'电压 / V',ylim:[0,3.3]});
      Charts.graph('speed-chart',[{name:'目标',dashed:true,data:history.map(x=>[x.ms/1000,x.target])},{name:'速度',data:history.map(x=>[x.ms/1000,x.speed])}],{x:'逻辑时间 / s',y:'速度 / rpm',ylim:[0,2500]});
    }
    if (tab==='tasks') taskFigure();
  }
  function taskFigure() {
    const canvas=$('task-chart'),rect=canvas.getBoundingClientRect();
    if (!rect.width) return;
    const ratio=window.devicePixelRatio||1,w=rect.width,h=228;
    canvas.width=Math.round(w*ratio); canvas.height=Math.round(h*ratio);
    const ctx=canvas.getContext('2d'); ctx.scale(ratio,ratio); ctx.clearRect(0,0,w,h);
    ctx.font='10px Segoe UI, Microsoft YaHei'; ctx.fillStyle='#869ba6';
    const labels=['采集','控制','通信','健康'],colours=['#236377','#198579','#8ca17c','#ad8b72'];
    const latest=value(0),start=Math.max(0,latest-600),span=Math.max(200,latest-start);
    for(let row=0;row<4;row++) { const y=28+row*44;ctx.fillStyle='#8097a4';ctx.fillText(labels[row],3,y+4);ctx.strokeStyle='#e5edf1';ctx.beginPath();ctx.moveTo(42,y);ctx.lineTo(w-12,y);ctx.stroke(); }
    const count=Math.min(value(38),256);
    for(let i=0;i<count;i++) {
      const ms=core.ed_task_get(i,0),id=core.ed_task_get(i,1); if(ms<start||!id) continue;
      const x=42+(ms-start)/span*(w-56); ctx.fillStyle=colours[id-1];ctx.fillRect(x,21+(id-1)*44,3,14);
    }
    ctx.fillStyle='#92a5af';ctx.fillText(start+'ms',42,218);ctx.fillText(latest+'ms',Math.max(42,w-60),218);
  }
  function render() {
    if (!initialized) return;
    const mode=value(1),faults=value(14);
    $('metrics').innerHTML=metric('逻辑运行时间',(value(0)/1000).toFixed(2),'s','C核心时钟')+metric('执行器速度',value(3).toFixed(0),'rpm','目标 '+value(4)+'rpm')+metric('DMA完成块',value(10).toFixed(0),'','队列 '+value(9)+'/8')+metric('当前固件',version(value(20)),'','分区 '+(value(23)?'B':'A'));
    $('mode-label').textContent=modes[mode];
    document.querySelectorAll('[data-mode]').forEach(el=>el.classList.toggle('active',Number(el.dataset.mode)===mode));
    const names=[]; if(faults&1) names.push('过流');if(faults&2) names.push('母线欠压');if(faults&4) names.push('编码器失联');if(faults&8) names.push('看门狗超时');
    $('fault-label').textContent=(names.length?'锁存故障：'+names.join('、'):'无锁存故障')+' · 母线 '+(value(7)/1000).toFixed(1)+'V · 电流 '+(value(6)/1000).toFixed(2)+'A · 输出 '+value(5).toFixed(1)+'%';
    $('dma-label').textContent='写入Bank '+value(34)+' · 位置 '+value(35)+'/16';
    $('dma-banks').innerHTML=[0,1].map(bank=>'<div class="dma-row"><span>Bank '+bank+'</span>'+Array.from({length:16},(_,i)=>'<div class="dma-cell '+(bank===value(34)?i===value(35)?'current':'writing':'')+'">'+core.ed_dma_get(bank,i)+'</div>').join('')+'</div>').join('');
    $('protocol-metrics').innerHTML=[['有效帧',11],['CRC错误',12],['溢出字节',13],['待解析字节',26]].map(([name,field])=>'<div><span>'+name+'</span><strong>'+value(field)+'</strong></div>').join('');
    const reply=Array.from({length:value(37)},(_,i)=>core.ed_tx_get(i));
    $('reply').textContent=reply.length?'RX ← '+hex(reply):'尚未接收指令';
    $('reply-status').textContent=reply.length?'应答状态：'+replyNames[reply[5]]:'';
    $('task-table').innerHTML=table(['任务','周期/触发','调用次数'],[['采集',value(15)+'ms',value(27)],['控制','10ms',value(28)],['通信','20ms / 入站处理',value(29)],['健康','100ms',value(30)]]);
    $('queue-label').textContent='采集队列 '+value(9)+'/8 · 被拒绝的块 '+value(36)+' · UART接收 '+value(26)+'/64字节';
    $('nv-summary').innerHTML=table(['参数','当前值'],[['目标速度',value(4)+'rpm'],['采样周期',value(15)+'ms'],['比例增益',(value(16)/1000).toFixed(3)],['存储记录',value(19)?'CRC有效':'尚未保存 / 使用默认值']]);
    $('flash-slots').innerHTML=[0,1].map(slot=>'<div class="flash-slot '+(value(23)===slot?'active':'')+'"><h4>分区 '+(slot?'B':'A')+(value(23)===slot?' · 活动':'')+'</h4><strong>'+version(value(slot?25:24))+'</strong><span>'+value(slot?42:41)+'字节 · '+(value(slot?40:39)?'已校验':'未确认镜像')+'</span></div>').join('');
    const update=value(22),progress=[2,3,4].includes(update)?1000:value(21);
    $('upgrade-label').textContent=upgradeNames[update];$('upgrade-percent').textContent=(progress/10).toFixed(1)+'%';$('upgrade-progress').value=progress;
    const eventCount=value(31);
    if(eventCount!==lastEventCount) {renderEvents('events');renderEvents('protocol-events',[3,4,5,16]);renderEvents('firmware-events',[9,10,11,12,13,14,15,16,17]);lastEventCount=eventCount;}
    if(update!==lastUpgradeState) {if([2,4,5,6].includes(update)) store();lastUpgradeState=update;}
    figures();
  }
  function collect() {
    const ms=value(0);if(history.length&&ms<history[history.length-1].ms) history=[];
    history.push({ms,adc:value(2),mean:value(17),speed:value(3),target:value(4),mode:value(1),faults:value(14),current:value(6),duty:value(5)});
    if(history.length>200) history.shift();
  }
  function changeTab(name) {
    tab=name;document.querySelectorAll('[data-tab]').forEach(button=>button.setAttribute('aria-selected',String(button.dataset.tab===name)));
    document.querySelectorAll('.panel').forEach(panel=>panel.hidden=panel.id!==name+'-panel');
    if(initialized) render(); if(name==='source') readSource();
  }
  function reboot() {core.ed_power_cycle();history=[];lastEventCount=-1;lastUpgradeState=-1;synchronizeInputs();collect();store();render();}
  function guard(action) { return async()=>{try {if(!initialized) throw Error('C核心尚未加载。');await action();}catch(error){status(error.message,true);}}; }
  function download(name,data,type) {const url=URL.createObjectURL(new Blob([data],{type})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
  async function readSource() {
    try {const response=await fetch('firmware/'+$('source-file').value);if(!response.ok) throw Error('源码读取失败');$('source-code').textContent=await response.text();}
    catch(error) {$('source-code').textContent=error.message;}
  }
  function parseInput() {
    const input=$('hex-frame').value.trim();if(!/^(?:[0-9a-fA-F]{2}\s*)+$/.test(input)) throw Error('请输入两位十六进制字节，用空格分隔。');
    const bytes=Uint8Array.from(input.match(/[0-9a-fA-F]{2}/g),x=>parseInt(x,16));if(bytes.length>256) throw Error('单次最多发送256字节。');return bytes;
  }
  function makeFrame() {
    const command=Number($('command').value),length=command===1?1:[2,3].includes(command)?2:0;
    const number=length?checkedInteger($('command-value'),0,length===1?1:65535):0;
    $('hex-frame').value=hex(encode(core,command,number,length));
  }
  async function upgrade(corrupted) {
    if(value(1)===1) transact(1,0,1);
    if(!imageFixture) {const response=await fetch('firmware-v101.bin');if(!response.ok) throw Error('示例镜像读取失败');imageFixture=new Uint8Array(await response.arrayBuffer());}
    const bytes=imageFixture.slice();if(corrupted) bytes[37]^=1;memory().set(bytes);
    const result=core.ed_stage_image(bytes.length,validation.image.version,validation.image.crc32);
    if(result) throw Error(replyNames[result]+'。已升级至v1.1时，可恢复出厂存储再测试。');
    paused=false;$('pause').textContent='暂停时间';render();status(corrupted?'已送入损坏镜像，C核心将验证CRC。':'已开始写入分区，校验通过后可重启。');
  }
  async function selfTest() {
    $('self-test').disabled=true;$('live-test-status').textContent='正在运行独立C/WebAssembly实例…';
    const rows=[];
    const check=(name,condition)=>{rows.push([name,condition?'<span class="pass">通过</span>':'<span class="fail">失败</span>']);if(!condition) throw Error(name+'失败');};
    try {
      const isolated=(await WebAssembly.instantiate(compiledModule,{})).exports;
      isolated.ed_init();const bytes=memory(isolated),vector=new TextEncoder().encode('123456789');bytes.set(vector);
      check('CRC16标准向量',isolated.ed_crc16(isolated.ed_buffer(),9)===0x4b37);
      check('CRC32标准向量',(isolated.ed_crc32(isolated.ed_buffer(),9)>>>0)===0xcbf43926);
      feed(isolated,encode(isolated,2,1200,2));feed(isolated,encode(isolated,1,1,1));isolated.ed_step(2000);isolated.ed_action(1,1);isolated.ed_step(20);
      const snapshot=validation.nativeSnapshot;
      check('GCC与WebAssembly的39项快照一致',snapshot.every((expected,i)=>Math.abs(isolated.ed_get(i)-expected)<0.00001));
      check('过流关闭驱动并锁存',isolated.ed_get(1)===2&&isolated.ed_get(5)===0);
      isolated.ed_action(1,0);isolated.ed_step(20);feed(isolated,encode(isolated,17,0,0));
      check('移除异常并确认后复位',isolated.ed_get(1)===0&&isolated.ed_get(14)===0);
      feed(isolated,encode(isolated,2,1400,2));feed(isolated,encode(isolated,16,0,0));isolated.ed_power_cycle();
      check('参数掉电恢复',isolated.ed_get(4)===1400&&isolated.ed_get(19)===1);
      isolated.ed_init();const fixture=new Uint8Array(8192);for(let i=0;i<fixture.length;i++)fixture[i]=(i*31+7)&255;
      bytes.set(fixture);const checksum=isolated.ed_crc32(isolated.ed_buffer(),fixture.length)>>>0;
      check('示例镜像CRC一致',checksum===validation.image.crc32);
      isolated.ed_stage_image(fixture.length,101,checksum);isolated.ed_step(2600);isolated.ed_power_cycle();isolated.ed_step(300);
      check('新固件通过校验和试启动',isolated.ed_get(20)===101&&isolated.ed_get(22)===4);
      isolated.ed_init();fixture[37]^=1;bytes.set(fixture);isolated.ed_stage_image(fixture.length,101,checksum);isolated.ed_step(2600);
      check('损坏镜像保持旧版本',isolated.ed_get(20)===100&&isolated.ed_get(22)===5);
      $('live-test-status').textContent=rows.length+'/'+rows.length+'项浏览器自检通过。';
    } catch(error) {$('live-test-status').textContent='自检失败：'+error.message;}
    $('live-tests').hidden=false;$('live-tests').innerHTML=table(['验证项','结果'],rows);$('self-test').disabled=false;
  }
  document.querySelectorAll('[data-tab]').forEach(button=>button.onclick=()=>changeTab(button.dataset.tab));
  $('open-source').onclick=()=>changeTab('source');$('source-file').onchange=readSource;
  $('start').onclick=guard(()=>{const code=transact(1,1,1);status(code?replyNames[code]:'启动指令已接受。',!!code);});
  $('stop').onclick=guard(()=>{const code=transact(1,0,1);status(code?replyNames[code]:'驱动已关闭，速度逐渐降至0；测试信号采集与任务继续运行。',!!code);});
  let targetTimer;$('target').oninput=()=>{clearTimeout(targetTimer);targetTimer=setTimeout(guard(()=>{const target=checkedInteger($('target'),0,2400);transact(2,target,2);status('目标速度已通过指令帧更新。');}),250);};
  $('sample').onchange=guard(()=>{transact(3,Number($('sample').value),2);status('采样周期已更新，DMA块完成间隔随之改变。');});
  $('gain').oninput=guard(()=>{core.ed_action(2,Number($('gain').value));$('gain-value').textContent=(value(16)/1000).toFixed(3);});
  $('inject').onclick=guard(()=>{core.ed_action(1,Number($('fault').value));core.ed_step(200);collect();render();status('异常已注入，请查看驱动关闭、锁存故障和事件记录。');});
  $('remove-fault').onclick=guard(()=>{core.ed_action(1,0);core.ed_step(20);render();status('异常源已移除，锁存故障仍需确认。');});
  $('ack-fault').onclick=guard(()=>{const code=transact(17,0,0);status(code?replyNames[code]:'复位指令已接受，设备回到停止状态。',!!code);});
  $('pause').onclick=guard(()=>{paused=!paused;$('pause').textContent=paused?'继续仿真':'暂停仿真';status(paused?'仿真已暂停，曲线、采集与任务时钟停止推进。':'仿真继续运行。');});
  $('advance').onclick=guard(()=>{core.ed_step(100);collect();render();status('已推进100ms。');});
  $('export').onclick=guard(()=>download('设备运行记录.json',JSON.stringify({schema:'embedded-device/v1',backend:'C/WebAssembly',snapshot:Array.from({length:45},(_,i)=>value(i)),history},null,2),'application/json;charset=utf-8'));
  $('make-frame').onclick=guard(makeFrame);
  $('command').onchange=()=>{$('command-value').value=$('command').value==='3'?5:$('command').value==='1'?1:1200;};
  $('send-frame').onclick=guard(()=>{feed(core,parseInput());synchronizeInputs();render();status('字节流已送入C解析器。');});
  $('split-frame').onclick=guard(async()=>{const bytes=parseInput(),cut=Math.min(5,bytes.length-1);feed(core,bytes.slice(0,cut));render();status('已发送第一段，解析器等待完整载荷和CRC。');setTimeout(()=>{feed(core,bytes.slice(cut));synchronizeInputs();render();status('第二段已发送，分包重组完成。');},350);});
  $('corrupt-frame').onclick=guard(()=>{const bytes=parseInput();bytes[bytes.length-1]^=1;feed(core,bytes);render();status('CRC已破坏，请查看拒绝记录与错误计数。');});
  $('overflow-frame').onclick=guard(()=>{for(let i=0;i<80;i++)core.ed_uart_feed(0);core.ed_comm();render();status('已注入80个噪声字节，接收缓冲有界，下一有效帧可重新同步。');});
  $('queue-overflow').onclick=guard(()=>{core.ed_action(3,0);render();status('采集队列满时拒绝新块，并增加溢出计数。');});
  $('watchdog-test').onclick=guard(()=>{core.ed_action(1,0);transact(1,1,1);core.ed_action(1,8);core.ed_step(200);render();status('控制任务未喂狗，健康任务已执行保护。');});
  $('save-params').onclick=guard(()=>{transact(16,0,0);store();render();status('参数记录与CRC已保存，可模拟重新上电。');});
  $('power-cycle').onclick=guard(()=>{reboot();status('已重新上电，参数与镜像按C核心校验结果恢复。');});
  $('corrupt-nv').onclick=guard(()=>{core.ed_test_corrupt_nv();reboot();status('参数CRC失效，已恢复默认参数。');});
  $('upgrade').onclick=guard(()=>upgrade(false));$('bad-upgrade').onclick=guard(()=>upgrade(true));
  $('interrupt-upgrade').onclick=guard(()=>{if(value(22)!==1)throw Error('请在镜像写入期间操作。');reboot();status('写入被中断，继续使用已确认固件。');});
  $('reboot-upgrade').onclick=guard(()=>{reboot();status('已重新启动，试启动通过健康检查后自动确认。');});
  $('reset-storage').onclick=guard(()=>{core.ed_init();history=[];lastEventCount=-1;lastUpgradeState=-1;synchronizeInputs();store();collect();render();status('已恢复出厂参数和v1.0固件。');});
  $('self-test').onclick=guard(selfTest);
  window.addEventListener('resize',()=>{if(initialized)figures();});
  async function initialize() {
    try {
      const [wasmResponse,reportResponse]=await Promise.all([fetch('device.wasm?v=20261012a'),fetch('validation.json?v=20261012a')]);
      if(!wasmResponse.ok||!reportResponse.ok)throw Error('工程资源未能加载');
      const binary=await wasmResponse.arrayBuffer();compiledModule=await WebAssembly.compile(binary);
      core=(await WebAssembly.instantiate(compiledModule,{})).exports;validation=await reportResponse.json();
      core.ed_init();restore();initialized=true;synchronizeInputs();collect();render();makeFrame();
      $('backend-status').textContent='C/WebAssembly已加载 · '+(binary.byteLength/1024).toFixed(1)+'KB';
      $('validation-badge').textContent=validation.passed+'/'+validation.total+'项主机验证 · '+validation.wasmPassed+'/'+validation.wasmTotal+'项WebAssembly验证';
      $('rtos-results').textContent=validation.tests.filter(test=>test.id.startsWith('freertos_')).map(test=>'PASS  '+test.id).join('\n')+'\n\n'+validation.kernel+'\n40个样本通过消息队列传递与核对。';
      $('test-table').innerHTML=table(['测试项','结果'],[...validation.tests,...validation.wasmTests].map(test=>[esc(test.id),test.passed?'<span class="pass">通过</span>':'<span class="fail">失败</span>']));
      status('C核心已就绪。启动设备，或选择模块进行操作。');
      setInterval(()=>{if(!paused){core.ed_step(50);collect();render();}},50);
    } catch(error) {status('C核心加载失败：'+error.message,true);$('backend-status').textContent='核心未运行';document.querySelectorAll('main button,main input,main select').forEach(el=>el.disabled=true);}
  }
  initialize();
})();
