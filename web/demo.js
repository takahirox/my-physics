const status=document.querySelector('#status');
const canvas=document.querySelector('#track');
const ctx=canvas.getContext('2d');
const ui=Object.fromEntries(['speed','rpm','gear','time','lod','damage','damageText','tires'].map(id=>[id,document.querySelector(`#${id}`)]));
const keys=new Set();let api;let previous=performance.now();let accumulator=0;let gear=1;

addEventListener('keydown',e=>{keys.add(e.code);if(/^Digit[1-6]$/.test(e.code))gear=Number(e.code.at(-1));if(e.code==='KeyR')api?.physics_reset();});
addEventListener('keyup',e=>keys.delete(e.code));

function controls(){let steer=(keys.has('ArrowLeft')||keys.has('KeyA')?-1:0)+(keys.has('ArrowRight')||keys.has('KeyD')?1:0);let throttle=keys.has('ArrowUp')||keys.has('KeyW')?1:0;let brake=keys.has('ArrowDown')||keys.has('KeyS')?1:0;let handbrake=keys.has('Space')?1:0;
  const pad=navigator.getGamepads?.()[0];if(pad){steer=Math.abs(pad.axes[0])>.08?pad.axes[0]:steer;throttle=Math.max(throttle,pad.buttons[7]?.value||0);brake=Math.max(brake,pad.buttons[6]?.value||0);handbrake=Math.max(handbrake,pad.buttons[0]?.value||0);}return{steer,throttle,brake,handbrake};}
function resize(){const r=canvas.getBoundingClientRect();const d=Math.min(devicePixelRatio,2);canvas.width=Math.round(r.width*d);canvas.height=Math.round(r.height*d);ctx.setTransform(d,0,0,d,0,0);return r;}
function draw(){const r=resize(),cx=r.width/2,cy=r.height*.7;const px=api.physics_x(0),pz=api.physics_z(0);ctx.fillStyle='#0c120e';ctx.fillRect(0,0,r.width,r.height);ctx.save();ctx.translate(cx,cy);ctx.scale(10,10);ctx.translate(-px,pz);
  ctx.fillStyle='#151b17';ctx.fillRect(-7,-220,14,440);ctx.strokeStyle='#384239';ctx.lineWidth=.08;for(let z=-220;z<220;z+=5){ctx.beginPath();ctx.moveTo(0,z);ctx.lineTo(0,z+2.2);ctx.stroke()}ctx.strokeStyle='#b9ef42';ctx.lineWidth=.04;ctx.strokeRect(-7,-220,14,440);
  const count=api.physics_vehicle_count();for(let i=0;i<count;i++){const x=api.physics_x(i),z=api.physics_z(i),yaw=api.physics_yaw(i);ctx.save();ctx.translate(x,-z);ctx.rotate(-yaw);ctx.fillStyle=i===0?'#b9ef42':`hsl(${28+i*17} 72% 56%)`;ctx.fillRect(-.88,-2.15,1.76,4.3);ctx.fillStyle='#090d0b';ctx.fillRect(-.68,-1.15,1.36,1.35);ctx.restore()}ctx.restore();}
function updateUi(){ui.speed.textContent=(api.physics_speed(0)*3.6).toFixed(1);ui.rpm.textContent=Math.round(api.physics_rpm(0));ui.gear.textContent=Math.round(api.physics_gear(0));ui.time.textContent=api.physics_time().toFixed(2);ui.lod.textContent=Math.round(api.physics_fidelity(0)*100);const damage=api.physics_damage(0);ui.damage.style.width=`${damage*100}%`;ui.damageText.textContent=`${Math.round(damage*100)}%`;ui.tires.innerHTML=[0,1,2,3].map((w)=>`<div class="tire"><span>${['FL','FR','RL','RR'][w]}</span><b>${(api.physics_tire_temp(0,w)-273.15).toFixed(1)} °C</b><span>${(api.physics_tire_pressure(0,w)/1000).toFixed(0)} kPa</span></div>`).join('');}
function frame(now){const elapsed=Math.min((now-previous)/1000,.05);previous=now;accumulator+=elapsed;const c=controls();api.physics_set_input(c.steer,c.throttle,c.brake,c.handbrake,gear);let steps=Math.min(Math.floor(accumulator/.001),50);if(steps){api.physics_step(steps);accumulator-=steps*.001;}draw();updateUi();requestAnimationFrame(frame);}

try{const response=await fetch('./physics.wasm');if(!response.ok)throw new Error(`HTTP ${response.status}: run scripts/build-wasm.sh first`);const result=await WebAssembly.instantiateStreaming(response,{});api=result.instance.exports;status.textContent='CORE ONLINE · FIXED DT 0.001 s';status.classList.add('ready');requestAnimationFrame(frame);}catch(error){status.textContent=`LOAD FAILED · ${error.message}`;console.error(error);}
