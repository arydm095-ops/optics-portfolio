(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.DriveScenarios=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const fields=['time','enabled','mode','fault','rpm','rpmRef','position','positionRef','id','iq','idRef','iqRef','torque','load','vd','vq','bus','voltageUse','ia','ib','ic','dutyA','dutyB','dutyC','encoder','fastCalls','speedCalls','positionCalls','managementCalls','canFrames','crcErrors','sequenceErrors','rejected','faultTime','torqueRef','mtpaId','fwId','actualId','adcA','adcB','actualIq','currentLimit','inertia','ld','lq','flux','busRated','speedBandwidth','currentBandwidth','rs','trajectoryVelocity','trajectoryAcceleration'];
const cases={
 step:{name:'速度与负载阶跃',profile:0,mode:0,target:3000,load:8,duration:2.4,step:.1,loadAt:.9,description:'0.1s给定转速阶跃，0.9s施加阻力矩；观察动态响应与负载恢复。'},
 position:{name:'伺服定位',profile:1,mode:2,target:360,load:0,duration:1.8,step:.1,loadAt:.9,description:'48V伺服轴执行五次多项式轨迹，位置、速度和电流环按不同周期运行。'},
 weakening:{name:'基速以上升速',profile:0,mode:0,target:6500,load:3,duration:3,step:.1,loadAt:.1,description:'提高目标转速，观察负d轴弱磁、电压利用率与电流圆约束。'},
 reversal:{name:'正反转切换',profile:0,mode:0,target:2000,load:2,duration:2.6,step:.1,reverseAt:1.3,loadAt:.1,description:'从正转切换为反转，观察制动转矩、反向电流和速度环积分状态。'},
 torque:{name:'转矩控制',profile:0,mode:1,target:12,load:10,duration:1.8,step:.1,loadAt:.1,description:'在转矩模式下跟踪转矩给定；速度由净转矩、惯量和阻力共同决定。'},
 fault:{name:'故障关断',profile:0,mode:0,target:2400,load:3,duration:1.7,step:.1,loadAt:.1,faultAt:1.0,description:'运行期间注入异常；记录故障锁存时刻与PWM关闭后的电机状态。'}
};
function snapshot(c){return Object.fromEntries(fields.map((name,i)=>[name,c.sv_get(i)]));}
function targetValue(mode,target){return mode===0?target*Math.PI/30:mode===2?target*Math.PI/180:target;}
function run(c,key,settings={}){
 const scenario={...cases[key],...settings};if(!cases[key])throw Error('未知工况');
 c.sv_init(scenario.profile);c.sv_set(0,scenario.mode);c.sv_set(3,settings.mtpa===false?0:1);c.sv_set(4,settings.weakening===false?0:1);c.sv_set(5,settings.feedforward===false?0:1);
 if(settings.calibration?.[13])c.sv_set(13,Math.min(c.sv_get(49),settings.calibration[13]));
 for(const [id,value] of Object.entries(settings.calibration||{})){if(!c.sv_set(Number(id),value))throw Error('标定参数越界：'+id);}
 for(const [id,value] of Object.entries(settings.gains||{})){if(!c.sv_set(Number(id),value))throw Error('控制增益越界：'+id);}
 c.sv_enable();const trace=[snapshot(c)],events=[];let stepped=false,loaded=false,reversed=false,injected=false;
 const duration=Math.round(scenario.duration*20000),start=performance.now();
 for(let tick=0;tick<duration;){
  const time=tick*.00005;
  if(!stepped&&time>=scenario.step){c.sv_set(1,targetValue(scenario.mode,scenario.target));stepped=true;events.push({time,action:'目标给定',value:scenario.target});}
  if(!loaded&&time>=scenario.loadAt){c.sv_set(2,scenario.load);loaded=true;if(scenario.load)events.push({time,action:'加载阻力矩',value:scenario.load});}
  if(scenario.reverseAt&&!reversed&&time>=scenario.reverseAt){c.sv_set(1,-targetValue(scenario.mode,scenario.target));reversed=true;events.push({time,action:'反向给定',value:-scenario.target});}
  if(scenario.faultAt&&!injected&&time>=scenario.faultAt){c.sv_inject(settings.faultMask||1);injected=true;events.push({time,action:'注入异常',value:settings.faultMask||1});}
  const count=tick>duration-400?1:20;c.sv_step(count);tick+=count;const row=snapshot(c);if(!Object.values(row).every(Number.isFinite))throw Error('当前参数下对象计算不收敛，请复核电机参数与环路带宽。');trace.push(row);
 }
 return{scenario,trace,events,computeMs:performance.now()-start,final:trace.at(-1),metrics:measure(trace,scenario)};
}
function measure(trace,s){
 const target=Math.abs(s.target);
 const value=r=>s.mode===2?r.position:s.mode===1?r.torque:r.rpm;
 const after=trace.filter(r=>r.time>=s.step&&!r.fault),peak=after.length?Math.max(...after.map(r=>value(r))):0;
 let rms=0;const position=trace.filter(r=>r.time>=s.step&&r.time<=s.step+.8);for(const r of position)rms+=(r.position-r.positionRef)**2;
 const settled=trace.filter(r=>r.time>Math.max(s.loadAt||0,s.reverseAt||0,s.step)+.4&&!r.fault);
 const final=trace.at(-1),expected=s.mode===2?final.positionRef:s.mode===1?final.torqueRef:final.rpmRef;
 return{overshoot:Math.max(0,(peak-target)/Math.max(target,1)*100),steadyError:Math.abs(value(final)-expected),trackingRms:Math.sqrt(rms/Math.max(position.length,1)),peakCurrent:Math.max(...trace.map(r=>Math.hypot(r.id,r.iq))),maxVoltageUse:Math.max(...trace.map(r=>r.voltageUse)),faultLatency:final.fault?Math.max(0,final.faultTime-(s.faultAt||0))*1000:null,samples:trace.length,settledSamples:settled.length};
}
function mtpaCurve(p){const points=[],limit=p.currentLimit||80,maxTorque=Math.min(36,6*p.flux*limit);for(let k=1;k<=18;k++){const torque=maxTorque*k/18;let best={magnitude:Infinity,id:0,iq:0};for(let n=0;n<=1600;n++){const id=-n*limit/1600,iq=torque/(6*(p.flux+(p.ld-p.lq)*id)),magnitude=Math.hypot(id,iq);if(magnitude<=limit+1e-9&&magnitude<best.magnitude)best={id,iq,magnitude};}points.push({torque,id:best.id,iq:best.iq,mtpa:best.magnitude,idZero:torque/(6*p.flux)});}return points;}
return{cases,fields,run,snapshot,measure,mtpaCurve,targetValue};
});
