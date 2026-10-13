(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.DriveReference=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
// Independent RK4 reference for the dq electrical/mechanical equations.
const p={rs:.12,ld:.0004,lq:.0008,flux:.085,inertia:.006,friction:.001,pairs:4};
function derivative(x,vd,vq,load){const[id,iq,w]=x,we=p.pairs*w;return[(vd-p.rs*id+we*p.lq*iq)/p.ld,(vq-p.rs*iq-we*(p.ld*id+p.flux))/p.lq,(1.5*p.pairs*(p.flux+(p.ld-p.lq)*id)*iq-load*w/Math.sqrt(w*w+.04)-p.friction*w)/p.inertia,w];}
function solve(steps,vd,vq,load,dt=.0000125){let x=[0,0,0,0];for(let n=0;n<steps;n++){const a=derivative(x,vd,vq,load),b=derivative(x.map((v,i)=>v+dt*a[i]/2),vd,vq,load),c=derivative(x.map((v,i)=>v+dt*b[i]/2),vd,vq,load),d=derivative(x.map((v,i)=>v+dt*c[i]),vd,vq,load);x=x.map((v,i)=>v+dt*(a[i]+2*b[i]+2*c[i]+d[i])/6);}return x;}
return{p,solve};
});
