'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),A=require('./analysis.js'),S=require('./scenarios.js');
const checks=[];function near(name,a,b,tol){const error=Math.abs(a-b);assert.ok(error<tol,`${name}: ${a} != ${b}`);checks.push({name,error,tolerance:tol,passed:true});}
const {c,add,mul,scale,div,abs,db}=A.complex;
const p={...A.fromSettings({}),elastic:true,inertia:.0002,loadInertia:.0008,stiffness:8,friction:.000002,loadFriction:.000002,shaftDamping:.00002};
near('two-mass antiresonance',A.natural(p).antiresonance,15.915494309,1e-8);near('two-mass resonance',A.natural(p).resonance,35.588127171,1e-8);
const N=A.notchCoefficients(200,.03,.25);[.8872547441581076,-1.4107321069061622,.8565060380194095].forEach((v,i)=>near('prewarped numerator '+i,N.b[i],v,1e-12));near('prewarped pole coefficient',N.a[2],.743760782177517,1e-12);
const np={notch:true,notchHz:200,notchZero:.03,notchPole:.25};near('notch centre attenuation',db(A.notchResponse(np,200)),20*Math.log10(.03/.25),1e-10);near('notch low frequency gain',abs(A.notchResponse(np,.00001)),1,1e-9);near('notch Nyquist gain',abs(A.notchResponse(np,1000)),1,1e-12);
const m=A.margins(f=>{const s=c(0,2*Math.PI*f);return div(c(25),add(add(add(mul(mul(s,s),s),scale(mul(s,s),10)),scale(s,10)),c(10)));},.001,100,1200);
near('classic gain margin',10**(m.gainMargin.margin/20),3.6,1e-8);near('classic phase margin',m.phaseMargin.margin,29.1104,.0001);
const multi=A.margins(f=>{const s=c(0,2*Math.PI*f);return mul(add(c(.1),div(c(.5),s)),A.mechanics(p,f));},.01,500,4000);assert.equal(multi.gainCrossings.length,3);checks.push({name:'all three resonance gain crossings retained',error:0,tolerance:0,passed:true});
(async()=>{
 const wasm=await WebAssembly.instantiate(fs.readFileSync(path.join(__dirname,'servo.wasm')),{env:{sin:Math.sin,cos:Math.cos,sqrt:Math.sqrt}}),core=wasm.instance.exports;
 core.sv_init(1);core.sv_set(8,.0002);core.sv_set(33,.0008);core.sv_set(34,8);
 const t=.1,w=Math.sqrt(8*(1/.0002+1/.0008)),expected=[.2+.8*Math.cos(w*t),.2*(1-Math.cos(w*t)),.2*t+.8*Math.sin(w*t)/w,.2*(t-Math.sin(w*t)/w)];
 expected.forEach((v,i)=>near('C two-mass analytic state '+i,core.sv_mechanical_probe(i,8000),v,.0001));
 core.sv_init(1);core.sv_set(38,1);core.sv_set(39,200);core.sv_set(40,.03);core.sv_set(41,.25);let re=0,im=0;for(let n=0;n<12000;n++){const angle=2*Math.PI*200*n*.0005,y=core.sv_notch(Math.sin(angle));if(n>=8000){re+=y*Math.sin(angle);im+=y*Math.cos(angle);}}near('C notch tone amplitude',2*Math.hypot(re,im)/4000,.03/.25,1e-9);
 const before=S.run(core,'resonance',{mtpa:false,calibration:{32:1,6:20,38:0}}),after=S.run(core,'resonance',{mtpa:false,calibration:{32:1,6:20,38:1}});
 assert.ok(!before.final.fault&&!after.final.fault);assert.ok(after.metrics.residualRms<before.metrics.residualRms);checks.push({name:'C notch reduces load-side residual positioning RMS',error:0,tolerance:0,passed:true});
 const settings={profile:1,target:90,calibration:{6:20,7:600,8:.00012,9:8,10:.025,11:.0012,12:.0012,13:.55,14:48,15:.8,28:4,29:131072,30:12,31:.000025,32:1,33:.00036,34:35.53057584,35:.00016,36:.000025,37:0,38:0,39:100,40:.04,41:.35,42:1000}};
 settings.gains=A.automatic(A.fromSettings(settings),20,600);
 const full=S.run(core,'resonance',JSON.parse(JSON.stringify(settings))),filtered=S.run(core,'resonance',{...settings,calibration:{...settings.calibration,38:1}});
 near('full motor definition survives JSON roundtrip',core.sv_get(77),.00016,1e-15);near('motor inductance uses H internally',core.sv_get(43),.0012,1e-15);
 assert.ok(filtered.metrics.residualRms<full.metrics.residualRms*.85);checks.push({name:'elastic workbench preset reduces residual RMS by more than 15 percent',error:0,tolerance:0,passed:true});
 const p0=A.fromSettings({}),g0=A.automatic(p0,25,800),suggestion=A.recommend(p0,g0,'speed',50,800);assert.ok(suggestion&&suggestion.phaseMargin>=50);checks.push({name:'recommended PI satisfies requested model phase margin',error:0,tolerance:0,passed:true});
 const report={generatedAt:new Date().toISOString(),passed:checks.length,total:checks.length,checks,mechanicalComparison:{before:before.metrics,after:after.metrics},workbenchPreset:{before:full.metrics,after:filtered.metrics},crossings:multi.gainCrossings};fs.writeFileSync(path.join(__dirname,'analysis-validation.json'),JSON.stringify(report,null,2)+'\n');console.log('PASS '+checks.length+' frequency/mechanical checks');console.log(JSON.stringify(report.workbenchPreset,null,2));
})().catch(e=>{console.error(e);process.exit(1);});
