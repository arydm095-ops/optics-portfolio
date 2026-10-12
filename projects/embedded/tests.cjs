const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const root=__dirname,report=JSON.parse(fs.readFileSync(path.join(root,'validation.json'),'utf8'));
let c,sequence=0,tests=[];
function check(name,condition){assert.ok(condition,name);tests.push({id:name,passed:true});console.log('PASS '+name);}
function bytes(){return new Uint8Array(c.memory.buffer,c.ed_buffer(),65536);}
function frame(cmd,value=0,len=0){const b=new Uint8Array(len+7);b.set([0xaa,0x55,(++sequence)&255,cmd,len]);for(let i=0;i<len;i++)b[5+i]=(value>>>(i*8))&255;bytes().set(b);const crc=c.ed_crc16(c.ed_buffer(),len+5);b[5+len]=crc&255;b[6+len]=crc>>>8;return b;}
function feed(b){for(const x of b)c.ed_uart_feed(x);c.ed_comm();}
function command(cmd,value=0,len=0){feed(frame(cmd,value,len));}
(async()=>{
  const module=await WebAssembly.compile(fs.readFileSync(path.join(root,'device.wasm')));
  c=(await WebAssembly.instantiate(module,{})).exports;
  check('wasm_has_no_external_runtime_imports',WebAssembly.Module.imports(module).length===0);
  c.ed_init();bytes().set(new TextEncoder().encode('123456789'));
  check('wasm_crc16_reference',c.ed_crc16(c.ed_buffer(),9)===0x4b37);
  check('wasm_crc32_unsigned_reference',(c.ed_crc32(c.ed_buffer(),9)>>>0)===0xcbf43926);
  command(2,1200,2);command(1,1,1);c.ed_step(2000);c.ed_action(1,1);c.ed_step(20);
  check('native_and_wasm_39_fields_agree',report.nativeSnapshot.every((expected,i)=>Math.abs(c.ed_get(i)-expected)<0.00001));
  c.ed_init();const split=frame(2,1100,2);feed(split.slice(0,4));check('wasm_waits_for_partial_frame',c.ed_get(4)===800);feed(split.slice(4));check('wasm_reassembles_frame',c.ed_get(4)===1100);
  const corrupt=frame(2,400,2);corrupt[corrupt.length-1]^=1;feed(corrupt);check('wasm_rejects_corrupt_frame',c.ed_get(4)===1100&&c.ed_get(12)===1);
  c.ed_init();c.ed_step(80);check('wasm_dma_bank_switch',c.ed_get(34)===1&&c.ed_get(10)===1&&c.ed_dma_get(0,15)>0);
  c.ed_init();command(1,1,1);c.ed_action(1,8);c.ed_step(200);check('wasm_watchdog_shuts_down_drive',(c.ed_get(14)&8)!==0&&c.ed_get(5)===0);
  c.ed_init();command(2,1650,2);command(16);c.ed_power_cycle();check('wasm_nv_parameter_recovery',c.ed_get(4)===1650);
  c.ed_init();const image=fs.readFileSync(path.join(root,'firmware-v101.bin'));bytes().set(image);
  check('wasm_image_matches_build_checksum',(c.ed_crc32(c.ed_buffer(),image.length)>>>0)===report.image.crc32);
  c.ed_stage_image(image.length,101,report.image.crc32);c.ed_step(2600);c.ed_power_cycle();check('wasm_trial_boot_switches_slot',c.ed_get(20)===101&&c.ed_get(22)===3);
  c.ed_power_cycle();check('wasm_trial_boot_rollback',c.ed_get(20)===100);
  c.ed_init();bytes().set(image);bytes()[37]^=1;c.ed_stage_image(image.length,101,report.image.crc32);c.ed_step(2600);check('wasm_corrupt_image_preserves_slot',c.ed_get(22)===5&&c.ed_get(20)===100);
  c.ed_init();command(2,700,2);command(16);const length=c.ed_export_nv(),saved=bytes().slice(0,length);c.ed_init();bytes().set(saved);check('wasm_storage_image_round_trip',c.ed_import_nv(length)===1&&c.ed_get(4)===700);
  bytes().set(saved);bytes()[90]^=1;check('wasm_storage_checksum_detects_corruption',c.ed_import_nv(length)===0);
  report.wasmTests=tests;report.wasmPassed=tests.length;report.wasmTotal=tests.length;
  fs.writeFileSync(path.join(root,'validation.json'),JSON.stringify(report,null,2)+'\n');
  console.log('Validated '+tests.length+' WebAssembly checks.');
})().catch(error=>{console.error(error);process.exit(1);});
