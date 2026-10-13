'use strict';
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const S=require('./scenarios.js'),R=require('./reference.js'),root=__dirname,checks=[];
function check(name,value,tolerance=0){assert.ok(value<=tolerance,name+': '+value);checks.push({name,passed:true,error:value,tolerance});}
(async()=>{
 const module=await WebAssembly.compile(fs.readFileSync(path.join(root,'servo.wasm'))),imports={env:{sin:Math.sin,cos:Math.cos,sqrt:Math.sqrt}};
 const c=(await WebAssembly.instantiate(module,imports)).exports;
 c.sv_init(0);c.sv_set(0,0);c.sv_set(1,2400*Math.PI/30);c.sv_set(2,5);c.sv_enable();c.sv_step(36000);
 const native=JSON.parse(fs.readFileSync(path.join(root,'native-validation.json'),'utf8'));
 const wasmSnapshot=native.snapshot.map((v,i)=>c.sv_get(i)),differences=native.snapshot.map((v,i)=>Math.abs(wasmSnapshot[i]-v));check('GCC / WebAssembly 52字段一致性',Math.max(...differences),.00001);
 const probe=R.solve(8000,1,12,3),actual=probe.map((v,i)=>c.sv_plant_probe(i,8000,1,12,3)),errors=actual.map((v,i)=>Math.abs(v-probe[i]));
 check('dq电机对象与独立RK4参考 / id',errors[0],.001);check('dq电机对象与独立RK4参考 / iq',errors[1],.001);check('dq电机对象与独立RK4参考 / omega',errors[2],.001);check('dq电机对象与独立RK4参考 / theta',errors[3],.001);
 const scenarios={};for(const key of Object.keys(S.cases)){const r=S.run(c,key);scenarios[key]={scenario:r.scenario,metrics:r.metrics,final:r.final,events:r.events};check(key+' / 数值有限',r.trace.every(v=>Object.values(v).every(Number.isFinite))?0:1);check(key+' / 电压约束',Math.max(0,r.metrics.maxVoltageUse-1),1e-12);if(key!=='fault')check(key+' / 无意外故障',r.final.fault);}
 check('弱磁升速跟踪误差 / rpm',scenarios.weakening.metrics.steadyError,20);
 const without=S.run(c,'weakening',{weakening:false}),withFW=scenarios.weakening;check('弱磁开启后转速更高',withFW.final.rpm>without.final.rpm+200?0:1);
 check('伺服定位最终误差 / degree',scenarios.position.metrics.steadyError,.15);
 const withFF=scenarios.position,noFF=S.run(c,'position',{feedforward:false});check('轨迹前馈降低位置跟踪RMS',withFF.metrics.trackingRms<noFF.metrics.trackingRms?0:1);
 const baseline=S.run(c,'step'),gains=Object.fromEntries(Array.from({length:6},(_,i)=>[20+i,c.sv_get(60+i)]));
 for(let key=20;key<=25;key++){
  const tuned=S.run(c,'step',{gains:{...gains,[key]:gains[key]*.5}});
  check('可调控制增益 '+key+' / C读回',Math.abs(c.sv_get(60+key-20)-gains[key]*.5),1e-10);
  const delta=Math.max(...baseline.trace.map((r,i)=>Math.abs(r.rpm-tuned.trace[i].rpm)+Math.abs(r.id-tuned.trace[i].id)+Math.abs(r.iq-tuned.trace[i].iq)));
  check('可调控制增益 '+key+' / 执行波形变化',delta>.0001?0:1);
 }
 const positionTune=S.run(c,'position',{gains:{26:5},feedforward:false});
 check('位置Kp / C读回',Math.abs(c.sv_get(66)-5),1e-10);
 check('位置Kp / 跟踪误差变化',positionTune.metrics.trackingRms>noFF.metrics.trackingRms?0:1);
 const report={generatedAt:new Date().toISOString(),native,checks,passed:checks.length,total:checks.length,numericCrosscheck:{reference:probe,c:actual,error:errors,method:'RK4 reference vs C midpoint integration',duration:.1,dt:.0000125},crossRuntime:{native:native.snapshot,wasm:wasmSnapshot,maxError:Math.max(...differences)},scenarios,comparison:{weakening:{enabled:withFW.final.rpm,disabled:without.final.rpm},positionFeedforward:{enabled:withFF.metrics.trackingRms,disabled:noFF.metrics.trackingRms}}};
 fs.writeFileSync(path.join(root,'validation.json'),JSON.stringify(report,null,2)+'\n');console.log('PASS '+native.total+' native C checks and '+checks.length+' WebAssembly/model checks');console.log(JSON.stringify(report.comparison,null,2));
})().catch(e=>{console.error(e);process.exit(1);});
