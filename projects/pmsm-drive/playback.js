(function(root,factory){if(typeof module==='object'&&module.exports)module.exports=factory();else root.DrivePlayback=factory();})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
return class DrivePlayback {
 constructor(){this.trace=[];this.time=0;this.duration=0;this.playing=false;this.rate=1;this.anchor=0;this.offset=0;}
 load(trace){this.trace=trace;this.duration=trace.length?trace.at(-1).time:0;this.time=0;this.offset=0;this.playing=false;}
 play(now){if(!this.trace.length)return;if(this.time>=this.duration)this.time=0;this.offset=this.time;this.anchor=now;this.playing=true;}
 update(now){if(this.playing){this.time=Math.min(this.duration,this.offset+Math.max(0,now-this.anchor)*this.rate/1000);if(this.time>=this.duration)this.playing=false;}return this.time;}
 pause(now){this.update(now);this.playing=false;}
 seek(time){this.time=Math.max(0,Math.min(this.duration,time));this.offset=this.time;this.playing=false;}
 setRate(rate,now){if(!Number.isFinite(rate)||rate<=0)throw Error('Invalid playback rate');this.update(now);this.rate=rate;this.offset=this.time;this.anchor=now;}
 state(){
  if(!this.trace.length)return null;
  let lo=0,hi=this.trace.length-1;
  while(lo<hi){const mid=(lo+hi+1)>>1;if(this.trace[mid].time<=this.time)lo=mid;else hi=mid-1;}
  const a=this.trace[lo],b=this.trace[Math.min(lo+1,this.trace.length-1)];
  const ratio=b.time>a.time?Math.max(0,Math.min(1,(this.time-a.time)/(b.time-a.time))):0;
  // Only the displayed pose is interpolated; logged controller values remain measured samples.
  return {...a,time:this.time,position:a.position+(b.position-a.position)*ratio,loadPosition:(a.loadPosition??a.position)+((b.loadPosition??b.position)-(a.loadPosition??a.position))*ratio};
 }
};
});
