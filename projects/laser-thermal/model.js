(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.ThermalOptics=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
// SI internally. Inputs use mm for material/waist, m for propagation, nm for wavelength.
const defaults={absorbed:1.5,pumpRadius:1,rodRadius:5,length:5,conductivity:1.38,dndT:9,coolant:25,wavelength:1064,waist:.35,m2:1,lensZ:.15,screenZ:3,compensator:0,cells:120};
function validate(p){for(const k of Object.keys(defaults))if(!Number.isFinite(p[k]))throw Error('请填写有限数值参数');if(p.absorbed<0||p.pumpRadius<=0||p.rodRadius<=p.pumpRadius||p.length<=0||p.conductivity<=0||p.waist<=0||p.m2<1||p.wavelength<=0||p.lensZ<0||p.screenZ<=p.lensZ||p.cells<20||p.cells>400)throw Error('请检查功率、材料尺寸、光束和传播距离');if(p.compensator!==0&&Math.abs(p.compensator)<.01)throw Error('补偿镜焦距绝对值须 ≥ 0.01 m；0 表示无补偿');}
function solveTridiagonal(a,b,c,d){b=b.slice();d=d.slice();for(let i=1;i<b.length;i++){const f=a[i]/b[i-1];b[i]-=f*c[i-1];d[i]-=f*d[i-1];}const x=Array(b.length);x[b.length-1]=d[b.length-1]/b[b.length-1];for(let i=b.length-2;i>=0;i--)x[i]=(d[i]-c[i]*x[i+1])/b[i];return x;}
function heat(p){validate(p);const n=Math.round(p.cells),R=p.rodRadius/1000,L=p.length/1000,wp=p.pumpRadius/1000,dr=R/n,norm=1-Math.exp(-2*R*R/(wp*wp)),k=p.conductivity;
 const centers=Array.from({length:n},(_,i)=>(i+.5)*dr),a=Array(n).fill(0),b=Array(n).fill(0),c=Array(n).fill(0),source=[],rhs=[];
 for(let i=0;i<n;i++){const lo=i*dr,hi=(i+1)*dr,Q=p.absorbed*(Math.exp(-2*lo*lo/(wp*wp))-Math.exp(-2*hi*hi/(wp*wp)))/norm,gl=i?2*Math.PI*k*L/Math.log(centers[i]/centers[i-1]):0,gr=2*Math.PI*k*L/Math.log((i<n-1?centers[i+1]:R)/centers[i]);a[i]=-gl;b[i]=gl+gr;c[i]=i<n-1?-gr:0;source.push(Q);rhs.push(Q+(i===n-1?gr*p.coolant:0));}
 const T=solveTridiagonal(a,b,c,rhs),qPeak=2*p.absorbed/(Math.PI*wp*wp*L*norm),center=T[0]+qPeak*centers[0]**2/(4*k),outflow=2*Math.PI*k*L/Math.log(R/centers[n-1])*(T[n-1]-p.coolant),power=p.absorbed*(p.dndT*1e-6)/(Math.PI*k*wp*wp*norm);
 const radial=[{r:0,T:center},...centers.map((r,i)=>({r,T:T[i]})),{r:R,T:p.coolant}];
 return{radial,center,rise:center-p.coolant,outflow,balance:outflow-p.absorbed,power,focal:power?1/power:Infinity,R,L,wp,norm,source};
}
function qLens(q,power){const den=q.re*q.re+q.im*q.im,a=q.re/den-power,b=-q.im/den,d=a*a+b*b;return{re:a/d,im:-b/d};}
function radius(q,lambdaEffective){return Math.sqrt(lambdaEffective*(q.re*q.re+q.im*q.im)/(Math.PI*q.im));}
function propagate(p,h){const le=p.wavelength*1e-9*p.m2,w0=p.waist/1000,zr=Math.PI*w0*w0/le,qBefore={re:p.lensZ,im:zr},qThermal=qLens(qBefore,h.power),qComp=qLens(qBefore,h.power+(p.compensator?1/p.compensator:0)),at=(z,q)=>z<=p.lensZ?radius({re:z,im:zr},le):radius({re:q.re+z-p.lensZ,im:q.im},le);
 const profile=Array.from({length:201},(_,i)=>{const z=i*p.screenZ/200;return{z,cold:Math.hypot(w0,w0*z/zr),hot:at(z,qThermal),compensated:at(z,qComp)};}),waistHot=Math.sqrt(le*qThermal.im/Math.PI),waistComp=Math.sqrt(le*qComp.im/Math.PI);
 return{profile,zr,inputDivergence:le/(Math.PI*w0),thermalDivergence:le/(Math.PI*waistHot),compensatedDivergence:le/(Math.PI*waistComp),hotWaist:waistHot,hotWaistZ:p.lensZ-qThermal.re,compWaist:waistComp,compWaistZ:p.lensZ-qComp.re,screen:profile[profile.length-1],qBefore,qThermal,qComp,probeAtMaterial:radius(qBefore,le)};
}
function analyze(p){const thermal=heat(p),beam=propagate(p,thermal),opd=thermal.radial.map(s=>({r:s.r,actual:thermal.L*p.dndT*1e-6*(s.T-thermal.center),paraxial:-thermal.power*s.r*s.r/2}));let sw=0,s1=0,s2=0;
 for(let i=0;i<opd.length-1;i++){const r=(opd[i].r+opd[i+1].r)/2,weight=Math.exp(-2*r*r/(beam.probeAtMaterial**2))*(opd[i+1].r**2-opd[i].r**2),e=((opd[i].actual-opd[i].paraxial)+(opd[i+1].actual-opd[i+1].paraxial))/2;sw+=weight;s1+=weight*e;s2+=weight*e*e;}
 return{params:p,thermal,beam,opd,residualRms:Math.sqrt(Math.max(0,s2/sw-(s1/sw)**2)),warning:thermal.rise>100?'温升超过 100 K：常数材料参数假设需复核。':Math.max(beam.thermalDivergence,beam.compensatedDivergence)>.05?'发散角较大：近轴高斯模型的适用性需复核。':''};
}
// Independent reference: integral solution of cylindrical conduction (Simpson quadrature).
function referenceCenter(p,steps=20000){const R=p.rodRadius/1000,wp=p.pumpRadius/1000,norm=1-Math.exp(-2*R*R/wp**2),f=r=>r?(1-Math.exp(-2*r*r/wp**2))/r:0,dr=R/steps;let sum=f(0)+f(R);for(let i=1;i<steps;i++)sum+=(i%2?4:2)*f(i*dr);return p.coolant+p.absorbed/(2*Math.PI*p.conductivity*(p.length/1000)*norm)*sum*dr/3;}
return{defaults,heat,analyze,propagate,qLens,radius,referenceCenter};
});
