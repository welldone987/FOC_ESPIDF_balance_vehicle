// Run with: node --test tests/ble_diagnostics_web.test.cjs
// Uses the shipped single-file page, without Bluetooth or motor side effects.
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'docs/平衡车控制界面（新版）.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
function harness() {
  const elements = new Map();
  function element() {
    return {textContent:'',dataset:{},style:{},children:[],disabled:false,classList:{toggle(){}},
      addEventListener(){},setAttribute(){},getBoundingClientRect(){return {width:300};},
      append(child){this.children.push(child);},replaceChildren(){this.children=[];},click(){},remove(){}};
  }
  const context = vm.createContext({clock:0,performance:{now:()=>context.clock},
    document:{hidden:false,body:element(),getElementById(id){if(!elements.has(id))elements.set(id,element());return elements.get(id);},
      createElement:element,addEventListener(){}},window:{isSecureContext:true,addEventListener(){}},navigator:{},
    setInterval(){return 1;},clearInterval(){},setTimeout(){},TextEncoder,Blob,URL,Date});
  vm.runInContext(script,context);
  return {context,elements,run:source=>vm.runInContext(source,context)};
}
function bytes(h,hex) {h.context.input=Uint8Array.from(Buffer.from(hex.replaceAll(' ',''),'hex'));return h.run('new DataView(input.buffer,input.byteOffset,input.byteLength)');}
function receive(h,hex){h.context.packet=bytes(h,hex);h.run('receiveDiag(packet)');}
function status(h,sequence,flags=5) {
  const packet=Buffer.alloc(20);packet[0]=2;packet[1]=1;packet[2]=flags;packet.writeUInt16LE(sequence,18);
  h.context.packet=bytes(h,packet.toString('hex'));h.run('receiveStatus(packet)');
}
const event='01 04 02 78 56 34 12 01 03 85 ff ff ff 03';
const metadata='01 03 0d 00 20 00 00 00 02 00 00 00 04 02 10 10 f7 00 01 01';

test('embedded dictionary matches the firmware-generated dictionary',()=>{
  const h=harness();assert.deepEqual(JSON.parse(h.run('JSON.stringify(DIAG_POINTS)')),JSON.parse(fs.readFileSync(path.join(root,'scripts/diagnostic_dictionary.json'))));
});
test('production golden packets preserve signed raw codes, channel and boot metadata',()=>{
  const h=harness();h.run('startDiagCapture({name:"bench"})');receive(h,metadata);receive(h,event);
  assert.equal(h.run('diagCapture.firstFault.rawCode'),-123);
  assert.equal(h.run('diagCapture.firstFault.channel'),3);
  assert.equal(h.run('diagCapture.metadata.mtu'),247);
  assert.match(h.elements.get('diag-first-title').textContent,/516/);
  assert.match(h.elements.get('diag-boot').textContent,/通过/);
});
test('first-fault replay is deduplicated and history remains ordered',()=>{
  const h=harness();h.run('startDiagCapture({})');
  receive(h,'01 00 05 09 00 00 00 00 03 00 00 00 00 ff');
  receive(h,'01 00 07 08 00 00 00 01 00 ff ff ff ff ff');
  receive(h,'01 00 05 09 00 00 00 00 03 00 00 00 00 ff');
  assert.equal(h.run('diagCapture.events.length'),2);
  assert.equal(h.run('diagCapture.events[0].sequence'),8);
  assert.equal(h.run('diagCapture.firstFault.sequence'),9);
  assert.equal(h.run('diagCapture.firstFault.channel'),null);
});
test('invalid schemas do not disconnect control and unknown points remain inspectable',()=>{
  const h=harness();h.run('startDiagCapture({});ready=true');
  receive(h,'02 00 05 01 00 00 00 00 03 00 00 00 00 ff');
  receive(h,'01 ff ff 01 00 00 00 00 00 00 00 00 00 ff');
  assert.equal(h.run('ready'),true);assert.equal(h.run('diagCapture.decodeErrors'),1);
  assert.equal(h.run('diagCapture.events[0].pointName'),'unknown_65535');
  assert.match(h.elements.get('diag-transport').textContent,/无效包/);
});
test('sample progress handles uint16 rollover and distinguishes frozen and stale status',()=>{
  const h=harness();h.run('startDiagCapture({});ready=true');status(h,65535);
  h.context.clock=100;status(h,0);
  assert.match(h.elements.get('diag-loop').textContent,/循环推进/);
  h.context.clock=800;status(h,0);
  assert.match(h.elements.get('diag-loop').textContent,/未推进/);
  h.context.clock=1500;h.run('render()');assert.match(h.elements.get('diag-loop').textContent,/回报中断/);
});
test('disconnect retains evidence while a new connection starts independent sequence space',()=>{
  const h=harness();h.run('startDiagCapture({name:"one"})');receive(h,event);h.run('teardown("offline")');
  assert.equal(h.run('diagCapture.events.length'),1);assert.equal(h.elements.get('diag-export').disabled,false);
  h.run('startDiagCapture({name:"two"})');receive(h,event);
  assert.equal(h.run('diagCaptures.length'),2);assert.equal(h.run('diagCapture.events.length'),1);
});
test('bounded event history retains the first fault outside the ring',()=>{
  const h=harness();h.run('startDiagCapture({})');
  for(let sequence=1;sequence<=300;sequence++) {
    const packet=Buffer.alloc(14);packet[0]=1;packet.writeUInt16LE(1280,1);packet.writeUInt32LE(sequence,3);packet[8]=sequence===1?3:0;packet[13]=255;
    receive(h,packet.toString('hex'));
  }
  assert.equal(h.run('diagCapture.events.length'),256);assert.equal(h.run('diagCapture.firstFault.sequence'),1);
  assert.equal(h.run('diagCapture.evicted'),44);
});
test('DIAG setup is optional and never writes a driving command',async()=>{
  const h=harness();h.run('startDiagCapture({})');
  const calls=[];h.context.meta=bytes(h,metadata);
  h.context.service={async getCharacteristic(uuid){calls.push(uuid);return {addEventListener(){},async readValue(){calls.push('read');return h.context.meta;},async startNotifications(){calls.push('subscribe');}};}};
  await h.run('setupDiag(service,session)');assert.deepEqual(calls.slice(1),['read','subscribe']);
  h.context.service={async getCharacteristic(){throw new Error('NotFoundError');}};
  await h.run('setupDiag(service,session)');h.run('renderDiag()');assert.match(h.elements.get('diag-transport').textContent,/不可用/);
});
test('late DIAG setup from a disconnected session cannot overwrite the new session',async()=>{
  const h=harness();h.run('startDiagCapture({})');let resolve;
  h.context.service={getCharacteristic(){return new Promise(r=>{resolve=r;});}};
  const pending=h.run('setupDiag(service,session)');h.run('teardown("offline");startDiagCapture({name:"new"})');
  resolve({});await pending;assert.equal(h.run('diagCharacteristic'),null);
});
test('manual metadata read serializes GATT access and cannot overlap ARM',async()=>{
  const h=harness();h.run('startDiagCapture({});ready=true');status(h,1);let resolve;
  h.context.characteristic={readValue(){return new Promise(r=>{resolve=r;});},async startNotifications(){}};
  h.run('diagCharacteristic=characteristic');const pending=h.run('refreshDiag()');
  h.run('beginControl()');assert.equal(h.run('wanted'),false);assert.equal(h.run('busy'),true);
  resolve(bytes(h,metadata));await pending;assert.equal(h.run('busy'),false);assert.equal(h.run('diagReading'),false);
  assert.equal(h.run('diagSubscribed'),true);
});
test('full connection remains usable without DIAG and sends only a neutral command',async()=>{
  const h=harness(),writes=[];const packet=Buffer.alloc(20);packet[0]=2;packet[1]=1;packet[2]=5;
  const telemetry={addEventListener(){},async startNotifications(){},async readValue(){return bytes(h,packet.toString('hex'));}};
  const service={async getCharacteristic(uuid){
    if(uuid.includes('0004-'))return {async writeValueWithResponse(data){writes.push(new TextDecoder().decode(data));}};
    if(uuid.includes('0003-'))return telemetry;
    throw new Error('NotFoundError');
  }};
  const device={name:'mock',addEventListener(){},gatt:{connected:true,async connect(){return {async getPrimaryService(){return service;}};},disconnect(){}}};
  h.context.navigator.bluetooth={async requestDevice(){return device;}};
  await h.run('connect()');assert.equal(h.run('ready'),true);assert.equal(h.run('wanted'),false);
  assert.deepEqual(writes,['D,0,0,0']);assert.match(h.elements.get('diag-transport').textContent,/不可用/);
});
test('JSON export includes raw evidence and separate sessions after disconnect',async()=>{
  const h=harness();let blob;
  h.context.URL={createObjectURL(value){blob=value;return 'blob:mock';},revokeObjectURL(){}};
  h.run('startDiagCapture({name:"bench"})');receive(h,metadata);receive(h,event);h.run('teardown("offline");exportDiag()');
  const output=JSON.parse(await blob.text());assert.equal(output.format,'dengfoc-ble-diagnostics-v1');
  assert.equal(output.captures[0].firstFault.rawCode,-123);assert.equal(output.captures[0].firstFault.rawHex,event);
  assert.ok(output.captures[0].endedAt);assert.equal(output.captures[0].metadata.firstPoint,516);
});
test('received FAULT and subsequent stop or blur revoke do not write S or disconnect',()=>{
  const h=harness(),writes=[];let disconnected=false;
  h.context.mockCommand={async writeValueWithResponse(data){writes.push(new TextDecoder().decode(data));}};
  h.context.mockDevice={gatt:{connected:true,disconnect(){disconnected=true;}}};
  h.run('startDiagCapture({});ready=true;wanted=true;arming=3;device=mockDevice;commandCharacteristic=mockCommand');
  h.context.packet=bytes(h,'02 06 11 02 00 00 61 00 00 00 00 00 00 00 00 00 00 00 00 00');
  h.run('receiveStatus(packet);revoke();pump()');
  assert.deepEqual(writes,[]);assert.equal(disconnected,false);assert.equal(h.run('ready'),true);assert.equal(h.run('wanted'),false);
});
test('in-flight command rejection after fault retains diagnostic connection and exports error',async()=>{
  const h=harness();let reject;
  h.context.mockDevice={gatt:{connected:true,disconnect(){throw new Error('must retain diagnostic link');}}};
  h.context.mockCommand={writeValueWithResponse(){return new Promise((resolve,r)=>{reject=r;});}};
  h.run('startDiagCapture({});ready=true;device=mockDevice;commandCharacteristic=mockCommand');status(h,17);
  const pending=h.run('pump()');receive(h,'01 12 03 01 00 00 00 00 03 00 00 00 00 ff');
  reject(new Error('GATT operation failed'));await pending;
  assert.equal(h.run('ready'),true);assert.equal(h.run('diagCapture.lastGattError.command'),'D,0,0,0');
  assert.equal(h.run('diagCapture.lastValidStatus.sampleSequence'),17);
  assert.equal(h.run('diagCapture.endedAt'),null);
});
test('unexplained write failure still disconnects and records why',async()=>{
  const h=harness();let disconnected=false;
  h.context.mockDevice={gatt:{connected:true,disconnect(){disconnected=true;}}};
  h.context.mockCommand={async writeValueWithResponse(){throw new Error('link lost');}};
  h.run('startDiagCapture({});ready=true;device=mockDevice;commandCharacteristic=mockCommand');
  await h.run('pump()');assert.equal(disconnected,true);assert.match(h.run('diagCapture.endReason'),/link lost/);
});
