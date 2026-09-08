// 无第三方依赖：执行HTML中的真实脚本，模拟DOM/GATT/单调时钟。
const {test} = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');
const html = fs.readFileSync(path.join(__dirname, '../docs/平衡车控制界面.html'), 'utf8');
const source = html.match(/<script>([\s\S]*?)<\/script>/)[1];

function packet(mode = 1, sequence = 0, flags = 5) {
  const data = new DataView(new ArrayBuffer(20));
  data.setUint8(0, 2); data.setUint8(1, mode); data.setUint8(2, flags);
  data.setUint16(4, sequence, true); data.setUint16(6, 10, true);
  data.setInt16(8, -1234, true); data.setInt16(10, 250, true);
  data.setInt16(12, -150, true); data.setInt16(14, 1000, true);
  data.setInt16(16, -667, true);
  return data;
}

function harness() {
  const elements = new Map();
  function target() {
    const events = {};
    return {events,style:{},classList:{toggle(){}},textContent:'',disabled:false,
      addEventListener(name, fn) { (events[name] ??= []).push(fn); },
      emit(name, event = {}) { return Promise.all((events[name] || []).map(fn => fn(event))); },
      setAttribute(){},setPointerCapture(){},
      getBoundingClientRect(){return {left:0,top:0,width:320,height:320};}};
  }
  const document = Object.assign(target(), {hidden:false,getElementById(id) {
    if (!elements.has(id)) elements.set(id, target()); return elements.get(id);
  }});
  const window = Object.assign(target(), {isSecureContext:true});
  const h = {time:0,writes:[],active:0,maxActive:0,hold:false,pending:[],devices:[]};
  const timers = new Map(); let timerId = 0;
  function makeDevice() {
    const telemetry = Object.assign(target(), {async startNotifications(){},async readValue(){return packet();}});
    const command = {async writeValueWithResponse(bytes) {
      h.writes.push(new TextDecoder().decode(bytes)); h.active++;
      h.maxActive = Math.max(h.maxActive, h.active);
      if (h.hold) await new Promise(resolve => h.pending.push(resolve));
      h.active--;
    }};
    const device = Object.assign(target(), {name:'测试车',telemetry,command});
    device.gatt = {connected:false,async connect(){this.connected=true;return this;},
      disconnect(){this.connected=false;void device.emit('gattserverdisconnected');},
      async getPrimaryService(){return {async getCharacteristic(uuid){return uuid.startsWith('6e400004') ? command : telemetry;}};}};
    h.devices.push(device); return device;
  }
  const context = vm.createContext({document,window,navigator:{bluetooth:{async requestDevice(){return makeDevice();}}},
    performance:{now:()=>h.time},TextEncoder,DataView,console,
    setInterval(fn){timers.set(++timerId,fn);return timerId;},clearInterval(id){timers.delete(id);}});
  vm.runInContext(source, context);
  h.run = code => vm.runInContext(code, context);
  h.flush = async () => {for(let i=0;i<12;i++) await Promise.resolve();};
  h.tick = async ms => {h.time += ms;for(const fn of timers.values()) fn();await h.flush();};
  h.connect = async () => {await document.getElementById('connect').emit('click');await h.flush();};
  h.status = async (mode, seq, flags=13, device=h.devices.at(-1)) => {
    await device.telemetry.emit('characteristicvaluechanged',{target:{value:packet(mode,seq,flags)}});await h.flush();
  };
  h.arm = async () => {
    await document.getElementById('arm').emit('click');await h.flush();await h.tick(150);
    const arm = h.writes.findLast(x=>x.startsWith('A,'));assert.ok(arm);
    await h.status(2,Number(arm.split(',')[1]));
  };
  h.document=document;h.window=window;h.elements=elements;
  return h;
}

test('连接完成后才可授权，解析小端有符号状态', async()=>{
  const h=harness();await h.connect();
  assert.equal(h.elements.get('pitch').textContent,'-12.34');
  assert.equal(h.elements.get('right').textContent,'-1.50');
  assert.equal(h.elements.get('turnTarget').textContent,'-0.667 V');
  assert.equal(h.run('canDrive()'),false);
  await h.arm();assert.equal(h.run('canDrive()'),true);
});

test('授权确认序号不匹配时不发送非零驾驶命令',async()=>{
  const h=harness();await h.connect();
  await h.elements.get('arm').emit('click');await h.flush();await h.tick(150);
  await h.status(2,65000);assert.equal(h.run('canDrive()'),false);
  await h.tick(1000);assert.match(h.writes.at(-1),/^S,/);
});

test('慢写入期间松手先清目标，后续仅发送停止且无并发写入',async()=>{
  const h=harness();await h.connect();await h.arm();
  h.hold=true;
  await h.elements.get('pad').emit('pointerdown',{pointerId:1,pointerType:'touch',clientX:160,clientY:60,preventDefault(){}});
  await h.tick(50);assert.match(h.writes.at(-1),/^D,\d+,0,98$/);
  await h.elements.get('pad').emit('pointerup',{pointerId:1});
  assert.equal(h.elements.get('throttle').textContent,'0%');
  const count=h.writes.length;await h.tick(50);assert.equal(h.writes.length,count);
  h.hold=false;h.pending.shift()();await h.flush();
  assert.match(h.writes.at(-1),/^S,/);assert.equal(h.maxActive,1);
});

test('急停不能被失焦停止或周期驾驶覆盖',async()=>{
  const h=harness();await h.connect();await h.arm();h.hold=true;
  await h.elements.get('emergency').emit('click');
  await h.window.emit('blur');await h.tick(50);
  assert.match(h.writes.at(-1),/^E,/);const count=h.writes.length;
  h.hold=false;h.pending.shift()();await h.flush();await h.tick(50);
  assert.equal(h.writes.length,count);
  await h.status(5,0);assert.equal(h.elements.get('state').textContent,'急停已执行');
});

test('状态停更自动撤销遥控，并清除过期测量显示',async()=>{
  const h=harness();await h.connect();await h.arm();await h.tick(600);
  assert.match(h.writes.at(-1),/^S,/);assert.equal(h.run('wanted'),false);
  assert.equal(h.elements.get('pitch').textContent,'—');
  assert.equal(h.elements.get('state').textContent,'状态回报中断');
});

test('页面后台发停止，停止后不再周期发送',async()=>{
  const h=harness();await h.connect();await h.arm();h.document.hidden=true;
  await h.document.emit('visibilitychange');await h.flush();
  assert.match(h.writes.at(-1),/^S,/);const count=h.writes.length;
  await h.tick(50);assert.equal(h.writes.length,count);
});

test('重连忽略旧连接通知且不恢复授权',async()=>{
  const h=harness();await h.connect();await h.arm();const old=h.devices.at(-1);
  old.gatt.disconnect();await h.flush();await h.connect();
  await h.status(5,0,13,old);
  assert.equal(h.run('ready'),true);assert.equal(h.run('wanted'),false);
  assert.equal(h.elements.get('state').textContent,'待授权');
});

test('pointercancel撤销驾驶，非法协议断开连接',async()=>{
  const h=harness();await h.connect();await h.arm();
  await h.elements.get('pad').emit('pointerdown',{pointerId:7,pointerType:'touch',clientX:160,clientY:160,preventDefault(){}});
  await h.elements.get('pad').emit('pointercancel',{pointerId:7});await h.flush();
  assert.match(h.writes.at(-1),/^S,/);
  await h.devices.at(-1).telemetry.emit('characteristicvaluechanged',{target:{value:new DataView(new ArrayBuffer(19))}});
  assert.equal(h.run('ready'),false);
});

test('命令序号65535回绕到0',async()=>{
  const h=harness();await h.connect();h.run('sequence=65535');
  await h.tick(50);await h.tick(50);
  assert.deepEqual(h.writes.slice(-2),['D,65535,0,0','D,0,0,0']);
});

test('GATT写入永久等待时断开，不开启并发重试',async()=>{
  const h=harness();await h.connect();await h.arm();h.hold=true;
  await h.tick(50);await h.tick(600);
  assert.equal(h.run('ready'),false);assert.equal(h.maxActive,1);
  assert.equal(h.devices.at(-1).gatt.connected,false);
  h.hold=false;h.pending.shift()();await h.flush();
  assert.equal(h.run('ready'),false);
});
