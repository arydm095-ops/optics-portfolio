(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.ThermalMaterials=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
// Reference constants, not a temperature-dependent or spectral material database.
const presets=[
 {id:'silica',name:'熔石英（HPFS 7980）',conductivity:1.38,dndT:9.598,
  reference:'导热率：25°C；dn/dT：1064nm、20–25°C，由手册多项式计算后取三位小数。',
  sourceTitle:'Corning HPFS 材料手册',sourceUrl:'https://www.corning.com/content/dam/corning/media/worldwide/csm/documents/HPFS_Product_Brochure_All_Grades_2022_6.pdf'},
 {id:'nbk7',name:'N-BK7光学玻璃',conductivity:1.114,dndT:1.1,
  reference:'dn/dT采用绝对折射率温度系数：1060nm、20–40°C。',
  sourceTitle:'SCHOTT N-BK7 数据表',sourceUrl:'https://media.schott.com/api/public/content/41e799d0bf874807a0bb8e702fbb75b5?v=54856406'},
 {id:'caf2',name:'氟化钙（CaF₂）',conductivity:9.71,dndT:-10.6,
  reference:'采用供应商数据表代表值；表中未注明dn/dT的参考波长和温度。',
  sourceTitle:'Crystran CaF₂ 数据表',sourceUrl:'https://www.crystran.co.uk/optical-materials/calcium-fluoride-caf2/'}
].map(m=>Object.freeze(m));
const custom=Object.freeze({id:'custom',name:'自定义介质',reference:'导热率和dn/dT采用当前输入值。',sourceTitle:null,sourceUrl:null});
function describe(id){const material=id==='custom'?custom:presets.find(m=>m.id===id);if(!material)throw Error('未知介质选项');return{...material};}
function apply(id,params){const material=describe(id);return{...params,...(id==='custom'?{}:{conductivity:material.conductivity,dndT:material.dndT}),material};}
return{presets:Object.freeze(presets),describe,apply,defaultId:'silica'};
});
