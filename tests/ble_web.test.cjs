const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const html=fs.readFileSync('docs/平衡车控制界面（新版）.html','utf8');
const source=html.match(/<script>([\s\S]*?)<\/script>/)[1];
function target(){return {listeners:{},style:{},disabled:false,textContent:'',innerHTML:'',setAttribute(k,v){this[k]=v},addEventListener(k,f){(this.listeners[k]??=[]).push(f)},fire(k,e={}){for(const f of this.listeners[k]??[])f(e)},getBoundingClientRect(){return {left:0,top:0,width:300,height:300}},setPointerCapture(){}};}
function setup(){
 const elements=Object.fromEntries(['connect','status','pad','knob','speed','direction'].map(k=>[k,target()]));
 const document=Object.assign(target(),{hidden:false,getElementById:k=>elements[k]});
 const window=target(),writes=[],intervals=new Map(),timeouts=new Map();let next=1,write=async()=>{};
 const characteristic={async writeValueWithResponse(data){writes.push(new TextDecoder().decode(data));await write();}};
 const device=Object.assign(target(),{gatt:{connected:false,async connect(){this.connected=true;return {async getPrimaryService(uuid){assert.equal(uuid,'6e400001-b5a3-f393-e0a9-e50e24dcca9e');return {async getCharacteristic(uuid){assert.equal(uuid,'6e400006-b5a3-f393-e0a9-e50e24dcca9e');return characteristic}}}}},disconnect(){this.connected=false;device.fire('gattserverdisconnected')}}});
 const context=vm.createContext({document,window,navigator:{bluetooth:{requestDevice:async()=>device}},TextEncoder,console,setInterval:f=>{const id=next++;intervals.set(id,f);return id},clearInterval:id=>intervals.delete(id),setTimeout:f=>{const id=next++;timeouts.set(id,f);return id},clearTimeout:id=>timeouts.delete(id)});
 vm.runInContext(source,context);
 return {elements,document,window,device,writes,intervals,timeouts,run:s=>vm.runInContext(s,context),setWrite:f=>write=f};
}
const tick=()=>new Promise(resolve=>setImmediate(resolve));
const point=(x=250,y=50)=>({pointerId:1,button:0,clientX:x,clientY:y,preventDefault(){}});
test('connect starts centered and uses only the motion characteristic',async()=>{const t=setup();await t.run('connect()');assert.deepEqual(t.writes,['D,0,0,0']);assert.equal(t.elements.pad['aria-disabled'],'false');});
test('joystick clamps to circle and release sends zero immediately',async()=>{const t=setup();await t.run('connect()');t.elements.pad.fire('pointerdown',point(1000,-1000));await t.run('flush(session)');const v=t.writes.at(-1).split(',').slice(2).map(Number);assert.ok(Math.hypot(...v)<=101);t.elements.pad.fire('pointerup',{pointerId:1});await tick();assert.match(t.writes.at(-1),/,0,0$/);});
test('zero is retained behind an in-flight write without overlapping writes',async()=>{const t=setup();await t.run('connect()');let resolve;t.setWrite(()=>new Promise(r=>resolve=r));t.elements.pad.fire('pointerdown',point());t.run('void flush(session)');const count=t.writes.length;t.window.fire('blur');for(const f of t.intervals.values())f();assert.equal(t.writes.length,count);t.setWrite(async()=>{});resolve();await tick();assert.equal(t.writes.length,count+1);assert.match(t.writes.at(-1),/,0,0$/);});
test('hidden page and cancelled pointer clear targets',async()=>{const t=setup();await t.run('connect()');for(const action of ['pointercancel','hidden']){t.elements.pad.fire('pointerdown',point());if(action==='hidden'){t.document.hidden=true;t.document.fire('visibilitychange')}else t.elements.pad.fire(action,{pointerId:1});await tick();assert.match(t.writes.at(-1),/,0,0$/);assert.equal(t.run('x+y'),0);}});
test('write failure disconnects and resets UI',async()=>{const t=setup();await t.run('connect()');t.setWrite(async()=>{throw new Error('lost')});await t.run('flush(session)');assert.equal(t.device.gatt.connected,false);assert.equal(t.intervals.size,0);assert.equal(t.elements.pad['aria-disabled'],'true');});
test('stalled GATT write disconnects at local deadline',async()=>{const t=setup();await t.run('connect()');t.setWrite(()=>new Promise(()=>{}));t.run('void flush(session)');for(const f of t.timeouts.values())f();await tick();assert.equal(t.device.gatt.connected,false);assert.match(t.elements.status.textContent,/超时/);});
test('late old write cannot close a new connection',async()=>{const t=setup();await t.run('connect()');let reject;t.setWrite(()=>new Promise((_,r)=>reject=r));t.run('void flush(session)');t.run('close()');t.setWrite(async()=>{});await t.run('connect()');reject(new Error('old'));await tick();assert.equal(t.device.gatt.connected,true);assert.equal(t.writes.at(-1),'D,0,0,0');});
test('keyboard release and focus loss return to zero',async()=>{const t=setup();await t.run('connect()');t.elements.pad.fire('keydown',{key:'ArrowUp',preventDefault(){}});await t.run('flush(session)');assert.match(t.writes.at(-1),/,0,50$/);t.elements.pad.fire('keyup',{key:'ArrowUp',preventDefault(){}});await tick();assert.match(t.writes.at(-1),/,0,0$/);});
