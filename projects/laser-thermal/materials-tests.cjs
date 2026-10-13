'use strict';
const assert=require('node:assert/strict'),T=require('./model.js'),materials=require('./materials.js');
const p={...T.defaults},cases=materials.presets.map(m=>T.analyze(materials.apply(m.id,p)));
// Compare the silica preset with the supplier's wavelength polynomial.
const wavelength=1.064,corning=9.390590+.235290/wavelength**2-.00131856/wavelength**4+.000302887/wavelength**6;
assert.ok(Math.abs(materials.describe('silica').dndT-corning)<.0005);
assert.equal(materials.describe('nbk7').dndT,1.1); // Absolute dn/dT at 1060 nm, 20–40 °C, not the relative 2.4.
for(const r of cases){
 assert.equal(r.params.absorbed,p.absorbed);
 assert.equal(r.params.rodRadius,p.rodRadius);
 assert.equal(r.params.length,p.length);
 assert.ok(Math.abs(r.thermal.center-T.referenceCenter(r.params))<.06);
 assert.ok(Math.abs(r.thermal.outflow-p.absorbed)<1e-9);
 const cold=T.analyze({...r.params,absorbed:0});
 assert.ok(Math.abs(cold.thermal.rise)<1e-9);
 assert.ok(Math.abs(cold.beam.screen.hot-cold.beam.screen.cold)<1e-12);
 const compensated=T.analyze({...r.params,compensator:-r.thermal.focal});
 assert.ok(Math.abs(compensated.beam.screen.compensated-cold.beam.screen.cold)<1e-12);
}
assert.ok(cases[1].thermal.rise>cases[0].thermal.rise);
assert.ok(cases[2].thermal.rise<cases[0].thermal.rise);
assert.ok(cases[2].thermal.focal<0);
assert.ok(cases[2].opd.at(-1).actual>0); // Negative thermo-optic coefficient gives outward-increasing OPD.
assert.ok(cases[2].beam.screen.hot>cases[2].beam.screen.cold);
assert.equal(new Set(cases.map(r=>r.beam.screen.hot.toFixed(9))).size,3);
const input={...p,conductivity:3.2,dndT:-2.4},custom=materials.apply('custom',input);
assert.equal(custom.conductivity,3.2);assert.equal(custom.dndT,-2.4);assert.equal(custom.material.id,'custom');
assert.equal(input.material,undefined); // The caller's input remains unchanged.
assert.throws(()=>materials.apply('unknown',p),/未知介质/);
console.log('Material references, heat balance, analytic temperatures, lens sign, propagation and compensation: PASS');
console.log(JSON.stringify(cases.map(r=>({material:r.params.material.name,centerC:r.thermal.center,focalM:r.thermal.focal,screenRadiusMm:r.beam.screen.hot*1000})),null,2));
