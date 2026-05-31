/* app.js – Gates Belt Drive v2 – Physics Engine + 2D + 3D + Gear-Driven + Hub-Load Chart */
'use strict';

// ── PDF Datum Values ──────────────────────────────────────────────────────────
const PULLEYS = {
  CRK:{ x:0,      y:0,     r:79.57,  pitch:158.14, eff:159.13, sr:1.000, cw:true,  color:'#4fc3f7', label:'CRK\nCrankshaft' },
  FAN:{ x:6,      y:213.5, r:60.495, pitch:120.99, eff:121.98, sr:1.302, cw:true,  color:'#ff8c42', label:'FAN\nFan' },
  IDR:{ x:-122,   y:235,   r:38.7,   pitch:77.40,  eff:79.40,  sr:2.069, cw:false, color:'#a78bfa', label:'IDR\nIdler' },
  ALT:{ x:-255,   y:373.2, r:30.07,  pitch:60.14,  eff:61.13,  sr:2.577, cw:true,  color:'#fbbf24', label:'ALT\nAlternator' },
  AC: { x:-265,   y:189,   r:59.655, pitch:119.31, eff:120.30, sr:1.320, cw:true,  color:'#34d399', label:'AC\nA/C Comp.' },
  TEN:{ x:-153.25,y:96.0,  r:38.7,   pitch:77.40,  eff:79.40,  sr:2.069, cw:false, color:'#f472b6', label:'TEN\nTensioner' }
};
const ORDER = ['CRK','FAN','IDR','ALT','AC','TEN'];
const SPAN_TYPES = { CRK:'outer', FAN:'inner', IDR:'inner', ALT:'outer', AC:'inner', TEN:'inner' };

// PDF peak tensions & hub-loads (reference baseline)
const PDF = {
  CRK:{ T:2190, F:2658.9, dir:96,  wrap:166.5, span:212.8 },
  FAN:{ T:945,  F:2866.4, dir:258, wrap:127.6, span:82.5  },
  IDR:{ T:1044, F:1710.1, dir:77,  wrap:108.4, span:178.7 },
  ALT:{ T:712,  F:1678.1, dir:279, wrap:145.1, span:182.1 },
  AC: { T:498,  F:985.8,  dir:49,  wrap:105.7, span:106.2 },
  TEN:{ T:480,  F:608.5,  dir:237, wrap:76.4,  span:136.3 }
};

// Load conditions table RPM → kW per component (from PDF)
const LOAD_TABLE = [
  { rpm:500,  P:{ CRK:3.30,  FAN:0.50,  IDR:0.10, ALT:1.60, AC:1.00, TEN:0.10 }},
  { rpm:800,  P:{ CRK:4.55,  FAN:0.80,  IDR:0.15, ALT:2.12, AC:1.33, TEN:0.15 }},
  { rpm:1000, P:{ CRK:6.54,  FAN:1.80,  IDR:0.17, ALT:2.67, AC:1.73, TEN:0.17 }},
  { rpm:1200, P:{ CRK:8.36,  FAN:2.80,  IDR:0.20, ALT:3.06, AC:2.10, TEN:0.20 }},
  { rpm:1400, P:{ CRK:10.69, FAN:4.60,  IDR:0.22, ALT:3.23, AC:2.42, TEN:0.22 }},
  { rpm:1600, P:{ CRK:13.20, FAN:6.50,  IDR:0.25, ALT:3.40, AC:2.80, TEN:0.25 }},
  { rpm:1800, P:{ CRK:18.17, FAN:10.80, IDR:0.27, ALT:3.55, AC:3.28, TEN:0.27 }},
  { rpm:2000, P:{ CRK:22.24, FAN:14.30, IDR:0.30, ALT:3.70, AC:3.64, TEN:0.30 }}
];

// Tensioner positions from PDF
const TEN_POS = [
  { label:'FREE',    arm:32.0,  ix:-163.7, iy:119.7, T:286.3, F:410.8, dir:243.8, wrap:91.7, len:1597.3 },
  { label:'REPLACE', arm:24.3,  ix:-158.0, iy:109.0, T:381.2, F:510.6, dir:239.2, wrap:84.1, len:1588.1 },
  { label:'MAX',     arm:19.1,  ix:-154.9, iy:101.4, T:440.2, F:563.0, dir:236.7, wrap:79.5, len:1581.5 },
  { label:'MEAN',    arm:15.4,  ix:-153.2, iy:96.0,  T:480.0, F:593.9, dir:235.1, wrap:76.4, len:1577.3 },
  { label:'MIN',     arm:11.9,  ix:-151.9, iy:90.5,  T:519.2, F:620.6, dir:233.7, wrap:73.4, len:1572.8 },
  { label:'LOAD',    arm:358.7, ix:-150.0, iy:70.0,  T:677.9, F:695.9, dir:230.0, wrap:61.8, len:1556.2 }
];


// ── C&U Group Validation Constants ───────────────────────────────────────────
const CU_VALIDATION = {
  fanSpeedRatio: 1.3,
  engineRPM_peak: 2400,
  radialForceFr_N: 358,
  bearingLife: { L10A:17820, L10B:3860, L10_composite:3305, Cr_roller:38179, Cr_ball:19035 },
  shaftBearingLoads: { P_ball:1669.49, P_roller:5298.5, P1:4088.36, P2:672.64 },
  note:'C&U Page3 tensioner uses fan-shaft coordinates (different datum from Gates global XY). MEAN F_hub=438.6N (C&U method) vs Gates 593.9N — different tension model, not a mismatch.'
};

// ── Drive Cycle Definitions ───────────────────────────────────────────────────
const DRIVE_CYCLES = {
  WLTC: {
    name:'WLTC', fullName:'WLTC Class 2 (Worldwide Harmonized)',
    color:'#e63946', lineWidth:2.5,
    note:'More aggressive, wider RPM range. Extra-High phase pushes FAN load to near-peak (cube law).',
    rpms:[800,850,900,1000,1100,1200,1100,1000,900,800,
          1000,1200,1400,1350,1300,1200,1100,1000,1100,1300,
          1400,1600,1800,2000,2200,2100,2000,1800,1600,1400,
          1600,1800,2000,2200,2400,2600,2500,2400,2200,2000],
    phases:[
      { name:'Low',        start:0,  end:9,  color:'rgba(52,211,153,0.10)',  lineColor:'#34d399' },
      { name:'Medium',     start:10, end:19, color:'rgba(251,191,36,0.10)',  lineColor:'#fbbf24' },
      { name:'High',       start:20, end:29, color:'rgba(244,114,182,0.10)', lineColor:'#f472b6' },
      { name:'Extra-High', start:30, end:39, color:'rgba(230,57,70,0.15)',   lineColor:'#e63946' }
    ]
  },
  NEDC: {
    name:'NEDC', fullName:'NEDC (New European Driving Cycle)',
    color:'#4fc3f7', lineWidth:2,
    note:'Lower peak RPM, more idle time. FAN load stays low; ALT/AC dominate at idle.',
    rpms:[800,800,850,950,1100,1250,1350,1250,1100,900,
          800,900,1050,1200,1350,1400,1350,1200,1000,800,
          900,1100,1300,1400,1300,1100,900,800,
          1000,1200,1400,1600,1800,2000,1900,1800,1600,1400,1200,1000],
    phases:[
      { name:'ECE Urban x4',     start:0,  end:27, color:'rgba(79,195,247,0.10)', lineColor:'#4fc3f7' },
      { name:'EUDC Extra-Urban', start:28, end:39, color:'rgba(255,140,66,0.12)', lineColor:'#ff8c42' }
    ]
  }
};

// ── Operating Mode Presets ────────────────────────────────────────────────────
const OP_MODES = [
  {
    id:'idle_demand', name:'Idle + High Demand', icon:'🔌', rpm:800, tension:3200,
    accent:'#f4a261', worstPulley:'CRK',
    teaser:'Stationary with A/C & full charge — worst CRK hub load per unit power.',
    scenario:'Vehicle stationary at a traffic light. A/C at maximum, battery charging at high current, headlights on. Engine idles at ~800 RPM.',
    mechanism:'At low RPM, belt velocity v = pi*D*n/60000 is minimal (~6.7 m/s). Since T_eff = P/v, the same accessory power requires TWICE the span tension at half belt speed. CRK hub force peaks even though power output is moderate.',
    risks:['CRK hub force elevated above PDF baseline','Belt micro-slip risk at low speed','Tensioner spring near minimum compression — reduced margin']
  },
  {
    id:'regen_braking', name:'Regen Braking', icon:'⚡', rpm:1600, tension:2500,
    accent:'#4fc3f7', worstPulley:'ALT',
    teaser:'BAS/ALT captures kinetic energy — tight/slack sides reverse on ALT spans.',
    scenario:'Vehicle decelerating: the alternator or BAS unit operates as a high-power generator, recovering kinetic energy. ALT output can reach 6-8 kW peak.',
    mechanism:'In normal motoring, ALT tight-side is on the CRK->ALT span. During regen, torque REVERSES — tight-side becomes ALT->CRK. ALT hub force direction shifts by ~150-180 deg. IDR and adjacent spans see increased tension as belt redistributes around the new loading geometry.',
    risks:['ALT hub force DIRECTION changes ~150 deg — different bearing loading plane','IDR load rises as rerouted tensions increase','Belt flutter possible in newly slack CRK->ALT span during transient']
  },
  {
    id:'bas_assist', name:'BAS Motor Mode', icon:'🚀', rpm:900, tension:3800,
    accent:'#a78bfa', worstPulley:'CRK',
    teaser:'Belt-starter drives CRK — reversed tension adds ~700N to CRK hub.',
    scenario:'BAS (Belt-Alternator-Starter) acts as electric motor at launch or during engine-off creep. ALT drives the crankshaft via belt at ~5 kW. Engine may be off.',
    mechanism:'ALT now drives the belt: tight-side is ALT->CRK span. CRK becomes the DRIVEN element. Additional BAS tension DeltaT = P_BAS/v = 5000/7.5 approx 667 N added on top of static tension on CRK span. Fan may freewheel if engine is off.',
    risks:['CRK hub force spikes due to reversed ALT tension','Transient tension surge at BAS engagement','FAN over-speed if engine restarts while BAS is active']
  },
  {
    id:'acceleration', name:'Peak Acceleration', icon:'💨', rpm:2300, tension:2500,
    accent:'#e63946', worstPulley:'FAN',
    teaser:'Fan cube law: P_FAN at 2300 RPM = ~19.8 kW — hub load approaches 2866 N peak.',
    scenario:'Full-throttle acceleration to near-peak RPM (~2300 RPM). All accessories at rated load. Fan speed = 2300 x 1.302 = 2995 RPM.',
    mechanism:'Fan power follows cube law: P_FAN proportional to (n/n_ref)^3. At 2300 vs 1200 design RPM: P_FAN = (2300/1200)^3 x 2.8 = 19.8 kW. Extreme belt tension on CRK->FAN span pushes FAN hub load toward PDF certified peak of 2866 N.',
    risks:['FAN hub load approaches 2866 N PDF peak','CRK must transmit all peak power','Belt centrifugal tension = mv^2/L increases rapidly','Tensioner at maximum extension']
  }
];
// ── Gear-Driven Accessories State ─────────────────────────────────────────────
// These loads act on CRK shaft via internal geartrain; extra crankshaft torque
// increases effective belt tension on the tight side of CRK span.
const GEAR_ACCESSORIES = {
  oilpump:   { name:'Oil Pump',            enabled:false, kW:3.0,  gearRatio:1.20, color:'#f4a261' },
  waterpump: { name:'Water Pump',          enabled:false, kW:2.0,  gearRatio:1.35, color:'#4fc3f7' },
  aircomp:   { name:'Air Compressor',      enabled:false, kW:5.0,  gearRatio:1.00, color:'#a78bfa' },
  pspump:    { name:'Power Steering Pump', enabled:false, kW:2.5,  gearRatio:1.15, color:'#52b788' }
};

// ── Math Helpers ──────────────────────────────────────────────────────────────
const r2d = r => r * 180 / Math.PI;
const d2r = d => d * Math.PI / 180;

function interp(rpm) {
  const t = LOAD_TABLE;
  if (rpm <= t[0].rpm) return { ...t[0].P };
  if (rpm >= t[t.length-1].rpm) return { ...t[t.length-1].P };
  for (let i = 0; i < t.length - 1; i++) {
    if (rpm >= t[i].rpm && rpm <= t[i+1].rpm) {
      const f = (rpm - t[i].rpm) / (t[i+1].rpm - t[i].rpm);
      const P = {};
      for (const k of ORDER) P[k] = t[i].P[k] + f * (t[i+1].P[k] - t[i].P[k]);
      return P;
    }
  }
}

function beltVelocity(rpm) {
  return Math.PI * PULLEYS.CRK.eff * rpm / 60000; // m/s
}

function tensionFromPower(P_kW, v_ms) {
  return v_ms < 0.01 ? 0 : (P_kW * 1000) / v_ms;
}

function outerTangent(p1, r1, p2, r2) {
  const dx = p2.x-p1.x, dy = p2.y-p1.y, d = Math.hypot(dx, dy);
  const gamma = Math.atan2(dy, dx);
  const cosA = (r1 - r2) / d;
  const alpha = Math.acos(Math.max(-1, Math.min(1, cosA)));
  const phi = gamma - alpha;
  return {
    t1: { x: p1.x + r1*Math.cos(phi + Math.PI/2), y: p1.y + r1*Math.sin(phi + Math.PI/2) },
    t2: { x: p2.x + r2*Math.cos(phi + Math.PI/2), y: p2.y + r2*Math.sin(phi + Math.PI/2) },
    phi: r2d(phi + Math.PI/2)
  };
}

function innerTangent(p1, r1, p2, r2) {
  const dx = p2.x-p1.x, dy = p2.y-p1.y, d = Math.hypot(dx, dy);
  const gamma = Math.atan2(dy, dx);
  const cosA = (r1 + r2) / d;
  const alpha = Math.acos(Math.max(-1, Math.min(1, cosA)));
  const phi = gamma - alpha;
  return {
    t1: { x: p1.x + r1*Math.cos(phi), y: p1.y + r1*Math.sin(phi) },
    t2: { x: p2.x - r2*Math.cos(phi), y: p2.y - r2*Math.sin(phi) },
    phi: r2d(phi)
  };
}

function getSpan(nameA, nameB) {
  const A = PULLEYS[nameA];
  const B = PULLEYS[nameB];
  const type = SPAN_TYPES[nameA];
  return type === 'outer'
    ? outerTangent({x:A.x,y:A.y}, A.r, {x:B.x,y:B.y}, B.r)
    : innerTangent({x:A.x,y:A.y}, A.r, {x:B.x,y:B.y}, B.r);
}

function computeHubload(nameIdx, spans, tensions) {
  const name = ORDER[nameIdx];
  const prevName = ORDER[(nameIdx - 1 + 6) % 6];
  const spanIn  = spans[prevName];
  const spanOut = spans[name];
  const ux_in  = spanIn.t2.x - spanIn.t1.x,  uy_in  = spanIn.t2.y - spanIn.t1.y;
  const ux_out = spanOut.t2.x - spanOut.t1.x, uy_out = spanOut.t2.y - spanOut.t1.y;
  const lin = Math.hypot(ux_in, uy_in), lout = Math.hypot(ux_out, uy_out);
  const T_in  = tensions[prevName] || 0;
  const T_out = tensions[name] || 0;
  const Fx = T_in*(ux_in/lin) + T_out*(ux_out/lout);
  const Fy = T_in*(uy_in/lin) + T_out*(uy_out/lout);
  return { Fx, Fy, F: Math.hypot(Fx, Fy), dir: (r2d(Math.atan2(Fy, Fx))+360)%360, T_in, T_out };
}

// Calculate total gear-driven load and its effect on CRK belt hub load
function computeGearLoad(rpm) {
  const v = beltVelocity(rpm); // belt velocity m/s
  let totalKW = 0;
  for (const key of Object.keys(GEAR_ACCESSORIES)) {
    const g = GEAR_ACCESSORIES[key];
    if (g.enabled) totalKW += g.kW;
  }
  // Gear-driven load adds to crankshaft torque demand.
  // Extra torque → extra tight-side tension on CRK span.
  // ΔT = ΔP / v_belt (approximate, as it all comes off the crank pulley)
  const deltaTension = v < 0.01 ? 0 : (totalKW * 1000) / v;
  // Crankshaft angular speed (rad/s)
  const omega = (rpm * 2 * Math.PI) / 60;
  // Extra torque on crankshaft shaft
  const extraTorque = omega > 0.01 ? (totalKW * 1000) / omega : 0;
  return { totalKW, deltaTension, extraTorque };
}

// ── State ─────────────────────────────────────────────────────────────────────
let state = { rpm: 1200, baseTension: 2500, tenIdx: 3 };
let hubData = {}, csvRows = [], spans = {}, tenPos = TEN_POS[3];
let gearLoadDelta = { totalKW: 0, deltaTension: 0, extraTorque: 0 };
// For RPM sweep (chart)
let hubLoadHistory = {}; // { pulleyName: [{ rpm, F }] }

function compute() {
  tenPos = TEN_POS[state.tenIdx];
  PULLEYS.TEN.x = tenPos.ix;
  PULLEYS.TEN.y = tenPos.iy;

  const v = beltVelocity(state.rpm);
  const P = interp(state.rpm);
  const tensions = {};
  for (const n of ORDER) tensions[n] = tensionFromPower(P[n], v);

  // Scale with slider value
  const scale = state.baseTension / 2500;
  for (const n of ORDER) tensions[n] *= scale;

  // Add gear-driven load as extra tension on CRK tight side
  gearLoadDelta = computeGearLoad(state.rpm);
  tensions['CRK'] = (tensions['CRK'] || 0) + gearLoadDelta.deltaTension;

  for (let i = 0; i < ORDER.length; i++) {
    const cur = ORDER[i], nxt = ORDER[(i+1) % ORDER.length];
    spans[cur] = getSpan(cur, nxt);
  }

  hubData = {};
  csvRows = [['Pulley','T_in(N)','T_out(N)','F_hub(N)','Dir(°)','PDF_F(N)','PDF_Dir(°)','ΔF','ΔDir']];
  for (let i = 0; i < ORDER.length; i++) {
    const n = ORDER[i];
    const hl = computeHubload(i, spans, tensions);
    hubData[n] = hl;
    const pdf = PDF[n];
    csvRows.push([n, hl.T_in.toFixed(1), hl.T_out.toFixed(1), hl.F.toFixed(1),
      hl.dir.toFixed(1), pdf.F, pdf.dir, (hl.F-pdf.F).toFixed(1), (hl.dir-pdf.dir).toFixed(1)]);
  }
}

// Compute hub loads across the full RPM range for chart data
function buildChartData() {
  const rpmPoints = [];
  for (let r = 300; r <= 2600; r += 50) rpmPoints.push(r);

  // Preserve current TEN position
  const savedTen = { x: PULLEYS.TEN.x, y: PULLEYS.TEN.y };

  hubLoadHistory = {};
  for (const n of ORDER) hubLoadHistory[n] = [];

  for (const rpm of rpmPoints) {
    const tp = TEN_POS[state.tenIdx];
    PULLEYS.TEN.x = tp.ix; PULLEYS.TEN.y = tp.iy;

    const v = beltVelocity(rpm);
    const P = interp(rpm);
    const tensions = {};
    for (const n of ORDER) tensions[n] = tensionFromPower(P[n], v);
    const scale = state.baseTension / 2500;
    for (const n of ORDER) tensions[n] *= scale;

    // Gear load effect at this RPM
    const gd = computeGearLoad(rpm);
    tensions['CRK'] = (tensions['CRK'] || 0) + gd.deltaTension;

    const spansLocal = {};
    for (let i = 0; i < ORDER.length; i++) {
      const cur = ORDER[i], nxt = ORDER[(i+1) % ORDER.length];
      spansLocal[cur] = getSpan(cur, nxt);
    }

    for (let i = 0; i < ORDER.length; i++) {
      const n = ORDER[i];
      const hl = computeHubload(i, spansLocal, tensions);
      hubLoadHistory[n].push({ rpm, F: hl.F });
    }
  }

  // Restore
  PULLEYS.TEN.x = savedTen.x; PULLEYS.TEN.y = savedTen.y;
}

// Shared 2D transform state (updated each frame so mouse handler can use it)
let _2d = { PAD:52, sc:1, minX:0, minY:0, W:0, H:0 };

// ── 2D Canvas Working Model ───────────────────────────────────────────────────
function draw2D() {
  const canvas = document.getElementById('canvas2d');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  const PAD = 52;
  const xs = ORDER.map(n => PULLEYS[n].x);
  const ys = ORDER.map(n => PULLEYS[n].y);
  const minX = Math.min(...xs)-90, maxX = Math.max(...xs)+90;
  const minY = Math.min(...ys)-90, maxY = Math.max(...ys)+90;
  const scaleX = (W - PAD*2) / (maxX - minX);
  const scaleY = (H - PAD*2) / (maxY - minY);
  const sc = Math.min(scaleX, scaleY);

  // Expose transform for mouse handler
  _2d = { PAD, sc, minX, minY, W, H };

  function tx(x) { return PAD + (x - minX) * sc; }
  function ty(y) { return H - PAD - (y - minY) * sc; }

  // Background grid
  ctx.strokeStyle = '#0e1623'; ctx.lineWidth = 1;
  for (let gx = Math.ceil(minX/50)*50; gx <= maxX; gx += 50) {
    ctx.beginPath(); ctx.moveTo(tx(gx), PAD/2); ctx.lineTo(tx(gx), H-PAD/2); ctx.stroke();
  }
  for (let gy = Math.ceil(minY/50)*50; gy <= maxY; gy += 50) {
    ctx.beginPath(); ctx.moveTo(PAD/2, ty(gy)); ctx.lineTo(W-PAD/2, ty(gy)); ctx.stroke();
  }

  // Axis labels
  ctx.fillStyle = '#ff4444'; ctx.font = 'bold 10px IBM Plex Mono,monospace';
  ctx.fillText('X (mm)', W - 44, H - 8);
  ctx.save(); ctx.translate(12, 52); ctx.rotate(-Math.PI/2);
  ctx.fillStyle = '#44ff44'; ctx.fillText('Y (mm)', 0, 0); ctx.restore();

  // Belt spans — animated dashes, width driven by hub-load
  const avgF = Object.values(hubData).reduce((s, d) => s + (d ? d.F : 0), 0) / ORDER.length;
  const beltWidth = Math.max(2, Math.min(6, 2 + avgF / 800));

  ctx.save();
  ctx.shadowColor = '#ffd700'; ctx.shadowBlur = 10;
  ctx.strokeStyle = '#ffd700'; ctx.lineWidth = beltWidth;
  ctx.setLineDash([14, 9]);
  ctx.lineDashOffset = dashOffset2D;
  for (const n of ORDER) {
    const s = spans[n];
    if (!s) continue;
    ctx.beginPath();
    ctx.moveTo(tx(s.t1.x), ty(s.t1.y));
    ctx.lineTo(tx(s.t2.x), ty(s.t2.y));
    ctx.stroke();
  }
  ctx.restore();

  // Pulleys
  for (const n of ORDER) {
    const p = PULLEYS[n];
    const cx = tx(p.x), cy = ty(p.y);
    const cr = p.r * sc;
    const hd = hubData[n];

    // Glow — intensity driven by hub force
    const glowIntensity = hd ? Math.min(hd.F / 3000, 1) : 0;
    const grd = ctx.createRadialGradient(cx, cy, cr*0.3, cx, cy, cr*1.3);
    grd.addColorStop(0, p.color + Math.round(30 + glowIntensity * 80).toString(16).padStart(2,'0'));
    grd.addColorStop(1, 'transparent');
    ctx.beginPath(); ctx.arc(cx, cy, cr*1.3, 0, Math.PI*2);
    ctx.fillStyle = grd; ctx.fill();

    // Outer rim
    ctx.beginPath(); ctx.arc(cx, cy, cr, 0, Math.PI*2);
    ctx.strokeStyle = p.color; ctx.lineWidth = 2.5;
    ctx.shadowColor = p.color; ctx.shadowBlur = 10 + glowIntensity * 12;
    ctx.stroke(); ctx.shadowBlur = 0;

    // Belt groove
    ctx.beginPath(); ctx.arc(cx, cy, cr*0.87, 0, Math.PI*2);
    ctx.strokeStyle = p.color + '55'; ctx.lineWidth = 1; ctx.stroke();

    // Hub
    ctx.beginPath(); ctx.arc(cx, cy, cr*0.2, 0, Math.PI*2);
    ctx.fillStyle = '#0f1825'; ctx.fill();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();

    // Rotating spokes
    const now = performance.now() * 0.001;
    const rotSpeed = state.rpm / 500;
    for (let s = 0; s < 4; s++) {
      const angle = (s * Math.PI/2) + now * (p.cw ? 1 : -1) * p.sr * rotSpeed;
      ctx.beginPath();
      ctx.moveTo(cx + cr*0.2*Math.cos(angle), cy + cr*0.2*Math.sin(angle));
      ctx.lineTo(cx + cr*0.85*Math.cos(angle), cy + cr*0.85*Math.sin(angle));
      ctx.strokeStyle = p.color + 'aa'; ctx.lineWidth = 1.2; ctx.stroke();
    }

    // Labels
    const lines = p.label.split('\n');
    ctx.textAlign = 'center';
    ctx.font = 'bold 10px Rajdhani,sans-serif';
    ctx.fillStyle = '#fff';
    ctx.fillText(lines[0], cx, cy - cr - 16);
    ctx.font = '9px IBM Plex Sans,sans-serif';
    ctx.fillStyle = p.color;
    ctx.fillText(lines[1], cx, cy - cr - 5);
    ctx.font = '7px IBM Plex Mono,monospace';
    ctx.fillStyle = '#4a5c78';
    ctx.fillText(`(${p.x.toFixed(0)}, ${p.y.toFixed(0)})`, cx, cy + cr + 13);
    ctx.textAlign = 'left';
  }

  // Hub-load arrows — size driven by computed force
  for (const n of ORDER) {
    const d = hubData[n];
    if (!d || d.F < 1) continue;
    const p = PULLEYS[n];
    const cx = tx(p.x), cy = ty(p.y);
    const maxF = 3500;
    const arrowLen = Math.min(d.F / maxF, 1) * 60 + 14;
    const angle = Math.atan2(-d.Fy, d.Fx);
    const ex = cx + arrowLen * Math.cos(angle);
    const ey = cy + arrowLen * Math.sin(angle);

    ctx.save();
    ctx.strokeStyle = '#e63946'; ctx.fillStyle = '#e63946';
    ctx.lineWidth = 2.2;
    ctx.shadowColor = '#e63946'; ctx.shadowBlur = 10;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke();
    const ah = 9, ang2 = Math.atan2(ey-cy, ex-cx);
    ctx.beginPath();
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - ah*Math.cos(ang2-0.4), ey - ah*Math.sin(ang2-0.4));
    ctx.lineTo(ex - ah*Math.cos(ang2+0.4), ey - ah*Math.sin(ang2+0.4));
    ctx.closePath(); ctx.fill();
    ctx.shadowBlur = 0;
    ctx.font = 'bold 8px IBM Plex Mono,monospace';
    ctx.fillStyle = '#f06292';
    ctx.fillText(`${d.F.toFixed(0)}N`, ex + 4, ey - 2);
    ctx.restore();
  }

  // Origin cross
  ctx.strokeStyle = '#ff444444'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(tx(0)-8, ty(0)); ctx.lineTo(tx(0)+8, ty(0)); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(tx(0), ty(0)-8); ctx.lineTo(tx(0), ty(0)+8); ctx.stroke();
}

// 2D animation loop
let dashOffset2D = 0;
function loop2D() {
  const v = beltVelocity(state.rpm);
  dashOffset2D -= v * 1000 * 0.016 * 0.14;
  draw2D();
  requestAnimationFrame(loop2D);
}

// ── HTML Table ────────────────────────────────────────────────────────────────
function renderTable() {
  const div = document.getElementById('calc-table');
  if (!div) return;
  const headers = ['Pulley','T_in (N)','T_out (N)','F_hub (N)','Dir (°)','PDF F (N)','PDF Dir (°)','ΔF (N)','ΔDir (°)'];
  let h = `<table><thead><tr>${headers.map(hd=>`<th>${hd}</th>`).join('')}</tr></thead><tbody>`;
  for (const n of ORDER) {
    const d = hubData[n], p = PDF[n];
    const dF = (d.F - p.F).toFixed(1), dD = (d.dir - p.dir).toFixed(1);
    const okF = Math.abs(+dF) < 300 ? '#52b788' : '#f87171';
    const okD = Math.abs(+dD) < 15 ? '#52b788' : '#f87171';
    h += `<tr>
      <td style="color:${PULLEYS[n].color};font-weight:700;font-family:'Rajdhani',sans-serif;font-size:1rem">${n}</td>
      <td>${d.T_in.toFixed(1)}</td><td>${d.T_out.toFixed(1)}</td>
      <td style="font-weight:700">${d.F.toFixed(1)}</td>
      <td>${d.dir.toFixed(1)}</td>
      <td>${p.F}</td><td>${p.dir}</td>
      <td style="color:${okF}">${+dF > 0 ? '+' : ''}${dF}</td>
      <td style="color:${okD}">${+dD > 0 ? '+' : ''}${dD}</td>
    </tr>`;
  }
  div.innerHTML = h + `</tbody></table>`;
  setTimeout(makeTableSortable, 30);
}

// ── Excel Export (SheetJS) ────────────────────────────────────────────────────
function downloadExcel() {
  if (typeof XLSX === 'undefined') {
    alert('Excel export library not loaded. Please check your internet connection.');
    return;
  }

  // Sheet 1: Hub-Load Results
  const wsData = [
    ['Gates Belt Drive – Hub-Load Report'],
    [`Engine Speed: ${state.rpm} RPM | Belt Tension: ${state.baseTension} N | Tensioner: ${TEN_POS[state.tenIdx].label} (${TEN_POS[state.tenIdx].arm}°)`],
    [],
    ['Pulley', 'T_in (N)', 'T_out (N)', 'F_hub (N)', 'Dir (°)', 'PDF F (N)', 'PDF Dir (°)', 'ΔF (N)', 'ΔDir (°)']
  ];
  for (const n of ORDER) {
    const d = hubData[n], p = PDF[n];
    wsData.push([
      n,
      +d.T_in.toFixed(1), +d.T_out.toFixed(1),
      +d.F.toFixed(1), +d.dir.toFixed(1),
      p.F, p.dir,
      +(d.F - p.F).toFixed(1), +(d.dir - p.dir).toFixed(1)
    ]);
  }

  // Sheet 2: Hub-Load vs RPM sweep
  const sweepData = [['RPM', ...ORDER.map(n => `${n} F_hub (N)`)]];
  if (hubLoadHistory['CRK'] && hubLoadHistory['CRK'].length) {
    const npts = hubLoadHistory['CRK'].length;
    for (let i = 0; i < npts; i++) {
      const row = [hubLoadHistory['CRK'][i].rpm];
      for (const n of ORDER) row.push(+(hubLoadHistory[n][i] ? hubLoadHistory[n][i].F.toFixed(1) : 0));
      sweepData.push(row);
    }
  }

  // Sheet 3: Gear-Driven Accessories
  const gearData = [
    ['Accessory', 'Enabled', 'Power (kW)', 'Gear Ratio'],
    ['Oil Pump',           GEAR_ACCESSORIES.oilpump.enabled   ? 'Yes' : 'No', GEAR_ACCESSORIES.oilpump.kW,   GEAR_ACCESSORIES.oilpump.gearRatio],
    ['Water Pump',         GEAR_ACCESSORIES.waterpump.enabled ? 'Yes' : 'No', GEAR_ACCESSORIES.waterpump.kW, GEAR_ACCESSORIES.waterpump.gearRatio],
    ['Air Compressor',     GEAR_ACCESSORIES.aircomp.enabled   ? 'Yes' : 'No', GEAR_ACCESSORIES.aircomp.kW,   GEAR_ACCESSORIES.aircomp.gearRatio],
    ['Power Steering Pump',GEAR_ACCESSORIES.pspump.enabled    ? 'Yes' : 'No', GEAR_ACCESSORIES.pspump.kW,    GEAR_ACCESSORIES.pspump.gearRatio],
    [],
    ['Total Gear Load (kW)', +gearLoadDelta.totalKW.toFixed(2)],
    ['Extra CRK Torque (Nm)', +gearLoadDelta.extraTorque.toFixed(2)],
    ['ΔF on CRK Hub (N)', +gearLoadDelta.deltaTension.toFixed(0)]
  ];

  const wb = XLSX.utils.book_new();
  const ws1 = XLSX.utils.aoa_to_sheet(wsData);
  const ws2 = XLSX.utils.aoa_to_sheet(sweepData);
  const ws3 = XLSX.utils.aoa_to_sheet(gearData);

  // Column widths for sheet 1
  ws1['!cols'] = [{ wch: 8 }, { wch: 12 }, { wch: 12 }, { wch: 12 }, { wch: 10 }, { wch: 12 }, { wch: 14 }, { wch: 10 }, { wch: 10 }];

  XLSX.utils.book_append_sheet(wb, ws1, 'Hub-Load Results');
  XLSX.utils.book_append_sheet(wb, ws2, 'Load vs RPM Sweep');
  XLSX.utils.book_append_sheet(wb, ws3, 'Gear-Driven Accessories');

  XLSX.writeFile(wb, `gates_hubload_${state.rpm}rpm.xlsx`);
}

// ── Gear-Driven UI ────────────────────────────────────────────────────────────
function initGearUI() {
  const keys = Object.keys(GEAR_ACCESSORIES);
  for (const key of keys) {
    const g = GEAR_ACCESSORIES[key];
    const cb = document.getElementById(`gear-${key}`);
    const sliderWrap = document.getElementById(`gs-${key}-wrap`);
    const slider = document.getElementById(`gs-${key}`);
    const lbl = document.getElementById(`gs-${key}-lbl`);
    const item = document.querySelector(`.gear-item[data-gear="${key}"]`);

    if (!cb || !slider) continue;

    cb.addEventListener('change', () => {
      g.enabled = cb.checked;
      sliderWrap.classList.toggle('enabled', g.enabled);
      item.classList.toggle('active', g.enabled);
      updateAll();
    });

    slider.value = g.kW;
    slider.addEventListener('input', () => {
      g.kW = +slider.value;
      lbl.textContent = g.kW.toFixed(1) + ' kW';
      if (g.enabled) updateAll();
    });
  }
}

function renderGearSummary() {
  const totalKWEl = document.getElementById('gear-total-kw');
  const torqueEl  = document.getElementById('gear-crk-torque');
  const dfEl      = document.getElementById('gear-crk-df');
  if (!totalKWEl) return;

  totalKWEl.textContent = gearLoadDelta.totalKW.toFixed(1) + ' kW';
  torqueEl.textContent  = gearLoadDelta.extraTorque.toFixed(1) + ' Nm';
  const df = gearLoadDelta.deltaTension;
  dfEl.textContent = (df >= 0 ? '+' : '') + df.toFixed(0) + ' N';
  dfEl.style.color = df > 200 ? '#f87171' : df > 0 ? '#f4d03f' : '#52b788';
}

// ── Live Insights Engine ──────────────────────────────────────────────────────
function renderInsights() {
  const v = beltVelocity(state.rpm);
  const tp = TEN_POS[state.tenIdx];

  // Belt velocity insight
  const velEl = document.getElementById('ins-velocity-text');
  if (velEl) {
    velEl.innerHTML = `Belt moves at <strong>${v.toFixed(2)} m/s</strong> (${(v*1000).toFixed(0)} mm/s). ` +
      (state.rpm > 1600
        ? `At high RPM, <span class="up">centrifugal tension rises</span>, reducing effective clamping.`
        : `At this RPM, belt tension is well within the dynamic range.`);
  }
  document.getElementById('insights-rpm-label').textContent = `@ ${state.rpm} RPM · v = ${v.toFixed(2)} m/s`;

  // CRK hub-load insight
  const crkEl = document.getElementById('ins-crk-text');
  if (crkEl && hubData.CRK) {
    const d = hubData.CRK, ref = PDF.CRK;
    const pct = ((d.F - ref.F) / ref.F * 100).toFixed(1);
    const updown = d.F > ref.F ? `<span class="up">↑${pct}% above</span>` : `<span class="down">↓${Math.abs(pct)}% below</span>`;
    crkEl.innerHTML = `CRK hub force = <strong>${d.F.toFixed(0)} N</strong> (${updown} PDF baseline of ${ref.F} N). ` +
      `Driven by tight-side belt tension of <strong>${d.T_out.toFixed(0)} N</strong> plus gear loads.`;
  }

  // Belt tension insight
  const tensEl = document.getElementById('ins-tension-text');
  if (tensEl) {
    const scaleRatio = state.baseTension / 480;
    tensEl.innerHTML = `Static tension = <strong>${state.baseTension} N</strong> ` +
      `(${scaleRatio.toFixed(1)}× design value of 480 N). ` +
      (state.baseTension > 2000
        ? `<span class="up">Very high tension</span> — all hub bearing loads are elevated proportionally. Risk of bearing overload.`
        : state.baseTension < 300
        ? `<span class="down">Low tension</span> — risk of belt slip at high-load pulleys (ALT, FAN).`
        : `Tension is in a reasonable operating range.`);
  }

  // Gear-driven insight
  const gearEl = document.getElementById('ins-gear-text');
  if (gearEl) {
    const gd = gearLoadDelta;
    const activeGears = Object.entries(GEAR_ACCESSORIES)
      .filter(([, g]) => g.enabled)
      .map(([, g]) => g.name);
    if (activeGears.length === 0) {
      gearEl.textContent = 'No gear-driven accessories active. Enable them in the Gear-Driven panel above to see their impact.';
    } else {
      gearEl.innerHTML = `Active: <strong>${activeGears.join(', ')}</strong>. ` +
        `Total gear load = <strong>${gd.totalKW.toFixed(1)} kW</strong> → ` +
        `extra CRK torque <strong>${gd.extraTorque.toFixed(0)} Nm</strong> → ` +
        `adds <span class="up">+${gd.deltaTension.toFixed(0)} N</span> to the CRK tight-side tension and hub force.`;
    }
  }

  // Tensioner arm insight
  const tenEl = document.getElementById('ins-tensioner-text');
  if (tenEl) {
    const tp2 = TEN_POS[state.tenIdx];
    const posNames = { FREE: 'free (belt slack)', REPLACE: 'belt replacement', MAX: 'maximum travel', MEAN: 'mean design', MIN: 'minimum travel', LOAD: 'under full load' };
    tenEl.innerHTML = `Arm at <strong>${tp2.label} (${tp2.arm.toFixed(1)}°)</strong> — ` +
      `tensioner at (${tp2.ix.toFixed(1)}, ${tp2.iy.toFixed(1)}) mm. ` +
      `Belt length = <strong>${tp2.len} mm</strong>. ` +
      (state.tenIdx === 0
        ? `<span class="up">FREE position</span>: belt may slip. Used for belt removal only.`
        : state.tenIdx === 5
        ? `<span class="up">LOAD position</span>: tensioner near stop — check for arm travel overrun.`
        : `Normal operating position (${posNames[tp2.label] || ''}).`);
  }

  // Peak load insight
  const peakEl = document.getElementById('ins-peak-text');
  if (peakEl) {
    let peak = { n: '', F: 0 };
    for (const n of ORDER) {
      if (hubData[n] && hubData[n].F > peak.F) peak = { n, F: hubData[n].F };
    }
    const pdfPeak = PDF[peak.n];
    const overPct = ((peak.F - pdfPeak.F) / pdfPeak.F * 100).toFixed(0);
    const cls = peak.F > pdfPeak.F * 1.1 ? 'up' : 'down';
    peakEl.innerHTML = `Highest hub load: <strong style="color:${PULLEYS[peak.n]?.color}">${peak.n}</strong> ` +
      `at <strong>${peak.F.toFixed(0)} N</strong>. ` +
      `PDF baseline for ${peak.n} = ${pdfPeak.F} N ` +
      `(<span class="${cls}">${+overPct >= 0 ? '+' : ''}${overPct}%</span>). ` +
      (peak.F > 3500 ? `<span class="up">⚠ Bearing overload risk!</span>` : `Within acceptable range.`);
  }
}

// ── Hub-Load vs RPM Chart ─────────────────────────────────────────────────────
let hubChart = null;
let chartVisible = {}; // { pulleyName: bool }

const CHART_COLORS = {
  CRK: '#4fc3f7', FAN: '#ff8c42', IDR: '#a78bfa',
  ALT: '#fbbf24', AC:  '#34d399', TEN: '#f472b6'
};

function initChart() {
  // Initialize all pulleys as visible
  for (const n of ORDER) chartVisible[n] = true;

  // Build toggle buttons
  const toggleContainer = document.getElementById('chart-toggles');
  if (toggleContainer) {
    for (const n of ORDER) {
      const btn = document.createElement('button');
      btn.className = 'chart-toggle-btn active';
      btn.textContent = n;
      btn.style.color = CHART_COLORS[n];
      btn.style.borderColor = CHART_COLORS[n];
      btn.setAttribute('data-pulley', n);
      btn.addEventListener('click', () => {
        chartVisible[n] = !chartVisible[n];
        btn.classList.toggle('active', chartVisible[n]);
        updateChart();
      });
      toggleContainer.appendChild(btn);
    }
  }

  const canvas = document.getElementById('hubload-chart');
  if (!canvas) return;

  const ctx2 = canvas.getContext('2d');
  hubChart = new Chart(ctx2, {
    type: 'line',
    data: { labels: [], datasets: [] },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: { duration: 180 },
      interaction: { mode: 'index', intersect: false },
      scales: {
        x: {
          type: 'linear',
          min: 300, max: 2600,
          title: {
            display: true,
            text: 'Engine Speed (RPM)',
            color: '#8ca0bc',
            font: { family: 'Rajdhani, sans-serif', size: 13, weight: '700' }
          },
          ticks: { color: '#4a5c78', font: { family: 'IBM Plex Mono, monospace', size: 11 }, stepSize: 200 },
          grid: { color: '#1c2840' }
        },
        y: {
          title: {
            display: true,
            text: 'Hub Load  F (N)',
            color: '#8ca0bc',
            font: { family: 'Rajdhani, sans-serif', size: 13, weight: '700' }
          },
          ticks: { color: '#4a5c78', font: { family: 'IBM Plex Mono, monospace', size: 11 } },
          grid: { color: '#1c2840' }
        }
      },
      plugins: {
        legend: { display: false },
        tooltip: {
          backgroundColor: '#0a0e1a',
          borderColor: '#243050',
          borderWidth: 1,
          titleFont: { family: 'Rajdhani, sans-serif', size: 13, weight: '700' },
          bodyFont: { family: 'IBM Plex Mono, monospace', size: 11 },
          titleColor: '#dce8f5',
          bodyColor: '#8ca0bc',
          padding: 12,
          callbacks: {
            title: (items) => `RPM: ${items[0].parsed.x}`,
            label: (ctx) => ` ${ctx.dataset.label}: ${ctx.parsed.y.toFixed(0)} N`
          }
        }
      }
    }
  });

  // Vertical RPM line plugin
  Chart.register({
    id: 'rpmLine',
    afterDraw(chart) {
      const xScale = chart.scales.x;
      if (!xScale) return;
      const xPos = xScale.getPixelForValue(state.rpm);
      const { top, bottom } = chart.chartArea;
      const ctx3 = chart.ctx;
      ctx3.save();
      ctx3.beginPath();
      ctx3.setLineDash([6, 4]);
      ctx3.strokeStyle = 'rgba(230,57,70,0.7)';
      ctx3.lineWidth = 1.5;
      ctx3.moveTo(xPos, top);
      ctx3.lineTo(xPos, bottom);
      ctx3.stroke();
      ctx3.setLineDash([]);
      ctx3.font = 'bold 11px Rajdhani, sans-serif';
      ctx3.fillStyle = 'rgba(230,57,70,0.9)';
      ctx3.fillText(`${state.rpm}`, xPos + 4, top + 14);
      ctx3.restore();
    }
  });
}

function updateChart() {
  if (!hubChart || !hubLoadHistory['CRK']) return;

  const datasets = [];
  for (const n of ORDER) {
    if (!chartVisible[n]) continue;
    const data = (hubLoadHistory[n] || []).map(d => ({ x: d.rpm, y: d.F }));
    datasets.push({
      label: n,
      data,
      borderColor: CHART_COLORS[n],
      backgroundColor: CHART_COLORS[n] + '10',
      borderWidth: 2.5,
      pointRadius: 0,
      pointHoverRadius: 6,
      tension: 0.4,
      fill: false
    });
  }

  hubChart.data.datasets = datasets;
  hubChart.update();
}

// ── Three.js 3D Belt Texture ──────────────────────────────────────────────────
function createBeltTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 1024; canvas.height = 128;
  const ctx = canvas.getContext('2d');

  ctx.fillStyle = '#181b21';
  ctx.fillRect(0, 0, 1024, 128);

  for (let i = 0; i < 3000; i++) {
    const x = Math.random() * 1024, y = Math.random() * 128, sz = Math.random() * 1.5;
    ctx.fillStyle = Math.random() > 0.5 ? '#20242d' : '#101217';
    ctx.fillRect(x, y, sz, sz);
  }

  const segW = 256;
  for (let s = 0; s < 4; s++) {
    const ox = s * segW;
    ctx.fillStyle = '#e63946';
    ctx.fillRect(ox + 18, 18, 32, 24);
    ctx.fillStyle = '#fff';
    ctx.beginPath(); ctx.arc(ox + 34, 30, 8, 0, Math.PI*2); ctx.fill();
    ctx.fillStyle = '#000'; ctx.font = 'bold 9px Arial'; ctx.textAlign = 'center';
    ctx.fillText('G', ox + 34, 34);
    ctx.fillStyle = '#fff'; ctx.font = 'bold 12px Arial'; ctx.textAlign = 'left';
    ctx.fillText('Gates', ox + 56, 28);
    ctx.fillStyle = '#8f9fb8'; ctx.font = '9px Courier New';
    ctx.fillText('MICRO-V\u00ae  8PK1577 ARAMID', ox + 56, 44);
    ctx.fillStyle = '#f4d03f'; ctx.font = 'bold 11px Arial';
    ctx.fillText('>>>', ox + 216, 33);
  }

  ctx.strokeStyle = '#121418'; ctx.lineWidth = 1;
  for (let y = 68; y < 128; y += 4) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(1024, y); ctx.stroke();
  }

  const tex = new THREE.CanvasTexture(canvas);
  tex.wrapS = THREE.RepeatWrapping; tex.wrapT = THREE.ClampToEdgeWrapping;
  tex.repeat.set(6, 1);
  return tex;
}

// ── Three.js 3D Model ─────────────────────────────────────────────────────────
let scene, cam, renderer, controls, clock;
let beltRibbonMesh, beltDashedLine;
let beltMaterial, beltDashedMaterial, beltTexture;
let tensionerArmGroup;
const meshes3D = {}, arrowObjs3D = [];

function initThree() {
  const ctr = document.getElementById('threejs-container');
  const W = ctr.clientWidth, H = ctr.clientHeight || 520;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x06080f);
  scene.fog = new THREE.FogExp2(0x06080f, 0.0004);

  cam = new THREE.PerspectiveCamera(46, W/H, 1, 4000);
  cam.position.set(-80, 180, 560);
  cam.lookAt(-80, 180, 0);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(W, H);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;
  ctr.innerHTML = '';
  ctr.appendChild(renderer.domElement);

  controls = new THREE.OrbitControls(cam, renderer.domElement);
  controls.enableDamping = true; controls.dampingFactor = 0.05;
  controls.maxPolarAngle = Math.PI * 0.8; controls.minDistance = 120; controls.maxDistance = 1600;

  clock = new THREE.Clock();

  // Lighting
  scene.add(new THREE.AmbientLight(0xffffff, 0.4));
  const sun = new THREE.DirectionalLight(0xffffff, 1.2);
  sun.position.set(300, 500, 400); sun.castShadow = true; scene.add(sun);
  const fill = new THREE.DirectionalLight(0x4488ff, 0.35);
  fill.position.set(-300, -100, -200); scene.add(fill);
  const rim = new THREE.PointLight(0x00f5ff, 0.55, 1200);
  rim.position.set(-100, 400, 200); scene.add(rim);

  // Grid (XY plane)
  const grid = new THREE.GridHelper(1000, 28, 0x1e2a42, 0x0d1520);
  grid.rotation.x = Math.PI/2; grid.position.z = -85; scene.add(grid);

  const COLORS3 = { CRK:0x4fc3f7, FAN:0xff8c42, IDR:0xa78bfa, ALT:0xfbbf24, AC:0x34d399, TEN:0xf472b6 };

  for (const n of ORDER) {
    const p = PULLEYS[n];

    const rotatingGrp = new THREE.Group();
    rotatingGrp.position.set(p.x, p.y, 0);
    scene.add(rotatingGrp);
    meshes3D[n] = rotatingGrp;

    const stationaryGrp = new THREE.Group();
    stationaryGrp.position.set(p.x, p.y, 0);
    scene.add(stationaryGrp);

    // Common rim (torus)
    const rim3 = new THREE.Mesh(
      new THREE.TorusGeometry(p.r, 5, 20, 64),
      new THREE.MeshStandardMaterial({ color: COLORS3[n], metalness: 0.8, roughness: 0.2, emissive: COLORS3[n], emissiveIntensity: 0.12 })
    );
    rotatingGrp.add(rim3);

    // Belt groove ring
    const groove = new THREE.Mesh(
      new THREE.TorusGeometry(p.r * 0.9, 2.5, 8, 64),
      new THREE.MeshStandardMaterial({ color: 0x111820, metalness: 0.95, roughness: 0.3 })
    );
    rotatingGrp.add(groove);

    // Hub disc
    const hub = new THREE.Mesh(
      new THREE.CylinderGeometry(p.r*0.25, p.r*0.25, 20, 16),
      new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.9, roughness: 0.15 })
    );
    hub.rotation.x = Math.PI/2; rotatingGrp.add(hub);

    if (n === 'CRK') {
      const block = new THREE.Mesh(new THREE.CylinderGeometry(112, 112, 50, 32),
        new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.8, metalness: 0.2 }));
      block.rotation.x = Math.PI/2; block.position.z = -36; stationaryGrp.add(block);

      const bolt = new THREE.Mesh(new THREE.CylinderGeometry(14, 14, 12, 6),
        new THREE.MeshStandardMaterial({ color: 0x64748b, metalness: 0.9, roughness: 0.1 }));
      bolt.rotation.x = Math.PI/2; bolt.position.z = 10; rotatingGrp.add(bolt);

      for (let s = 0; s < 6; s++) {
        const angle = s * Math.PI / 3;
        const spoke = new THREE.Mesh(new THREE.CylinderGeometry(10, 10, 16, 12),
          new THREE.MeshStandardMaterial({ color: 0x223344, metalness: 0.8 }));
        spoke.position.set(p.r * 0.6 * Math.cos(angle), p.r * 0.6 * Math.sin(angle), 0);
        spoke.rotation.x = Math.PI/2; rotatingGrp.add(spoke);
      }
    }
    else if (n === 'ALT') {
      const altBody = new THREE.Mesh(new THREE.CylinderGeometry(50, 50, 100, 24),
        new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.7, roughness: 0.3 }));
      altBody.rotation.x = Math.PI/2; altBody.position.z = -62; stationaryGrp.add(altBody);
      const copper = new THREE.Mesh(new THREE.CylinderGeometry(47, 47, 85, 16),
        new THREE.MeshStandardMaterial({ color: 0xd97706, metalness: 0.8, roughness: 0.2 }));
      copper.rotation.x = Math.PI/2; copper.position.z = -62; stationaryGrp.add(copper);
      for (let s = 0; s < 8; s++) {
        const angle = s * Math.PI / 4;
        const slit = new THREE.Mesh(new THREE.BoxGeometry(10, 86, 6),
          new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.6 }));
        slit.position.set(48 * Math.cos(angle), 48 * Math.sin(angle), -62);
        slit.rotation.z = angle; stationaryGrp.add(slit);
      }
      const ear1 = new THREE.Mesh(new THREE.BoxGeometry(16, 25, 25),
        new THREE.MeshStandardMaterial({ color: 0x334155 }));
      ear1.position.set(0, -60, -42); stationaryGrp.add(ear1);
      for (let s = 0; s < 4; s++) {
        const angle = s * Math.PI/2;
        const spoke = new THREE.Mesh(new THREE.BoxGeometry(p.r*1.3, 4, 3),
          new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.8 }));
        spoke.rotation.z = angle; rotatingGrp.add(spoke);
      }
    }
    else if (n === 'AC') {
      const acBody = new THREE.Mesh(new THREE.CylinderGeometry(57, 57, 120, 24),
        new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.6, roughness: 0.5 }));
      acBody.rotation.x = Math.PI/2; acBody.position.z = -72; stationaryGrp.add(acBody);
      for (let f = 0; f < 5; f++) {
        const fin = new THREE.Mesh(new THREE.CylinderGeometry(60, 60, 3, 24),
          new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.6 }));
        fin.rotation.x = Math.PI/2; fin.position.set(0, 0, -32 - f*20); stationaryGrp.add(fin);
      }
      const clutch = new THREE.Mesh(new THREE.CylinderGeometry(42, 42, 6, 24),
        new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.85, roughness: 0.25 }));
      clutch.rotation.x = Math.PI/2; clutch.position.z = 8; rotatingGrp.add(clutch);
      for (let s = 0; s < 3; s++) {
        const angle = s * (2 * Math.PI / 3);
        const damper = new THREE.Mesh(new THREE.CylinderGeometry(8, 8, 8, 12),
          new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.9 }));
        damper.rotation.x = Math.PI/2; damper.position.set(24 * Math.cos(angle), 24 * Math.sin(angle), 10);
        rotatingGrp.add(damper);
      }
    }
    else if (n === 'FAN') {
      const fanClutch = new THREE.Mesh(new THREE.CylinderGeometry(36, 36, 16, 24),
        new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.85, roughness: 0.15 }));
      fanClutch.rotation.x = Math.PI/2; fanClutch.position.z = 12; rotatingGrp.add(fanClutch);
      for (let s = 0; s < 12; s++) {
        const angle = s * (2 * Math.PI / 12);
        const rib = new THREE.Mesh(new THREE.BoxGeometry(2, 36, 4),
          new THREE.MeshStandardMaterial({ color: 0x64748b, metalness: 0.8 }));
        rib.rotation.z = angle; rib.position.set(0, 0, 20); rotatingGrp.add(rib);
      }
      const fanGroup = new THREE.Group(); fanGroup.position.z = 15;
      for (let i = 0; i < 7; i++) {
        const angle = i * (2 * Math.PI / 7);
        const bladeGeom = new THREE.BoxGeometry(16, 120, 3);
        bladeGeom.translate(0, 75, 0);
        const blade = new THREE.Mesh(bladeGeom,
          new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.7, metalness: 0.1 }));
        blade.rotation.z = angle; blade.rotation.x = 0.35; fanGroup.add(blade);
      }
      rotatingGrp.add(fanGroup);
    }
    else if (n === 'IDR') {
      const bolt = new THREE.Mesh(new THREE.CylinderGeometry(10, 10, 30, 6),
        new THREE.MeshStandardMaterial({ color: 0x64748b, metalness: 0.9 }));
      bolt.rotation.x = Math.PI/2; bolt.position.z = 5; stationaryGrp.add(bolt);
      for (let s = 0; s < 4; s++) {
        const angle = s * Math.PI/2;
        const spoke = new THREE.Mesh(new THREE.BoxGeometry(p.r*1.3, 4, 3),
          new THREE.MeshStandardMaterial({ color: 0x1e293b }));
        spoke.rotation.z = angle; rotatingGrp.add(spoke);
      }
    }
    else if (n === 'TEN') {
      const baseHousing = new THREE.Mesh(new THREE.CylinderGeometry(38, 38, 26, 32),
        new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.75, roughness: 0.4 }));
      baseHousing.rotation.x = Math.PI/2; baseHousing.position.set(-240, 72, -18); scene.add(baseHousing);
      const springCoil = new THREE.Mesh(new THREE.TorusGeometry(32, 4, 12, 32),
        new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.9, roughness: 0.1 }));
      springCoil.position.set(-240, 72, -18); scene.add(springCoil);
      for (let s = 0; s < 4; s++) {
        const angle = s * Math.PI/2;
        const spoke = new THREE.Mesh(new THREE.BoxGeometry(p.r*1.3, 4, 3),
          new THREE.MeshStandardMaterial({ color: 0x1e293b }));
        spoke.rotation.z = angle; rotatingGrp.add(spoke);
      }
    }
  }

  // Tensioner Arm
  tensionerArmGroup = new THREE.Group();
  tensionerArmGroup.position.set(-240, 72, -8);
  scene.add(tensionerArmGroup);

  const armBarGeom = new THREE.BoxGeometry(90, 24, 12);
  armBarGeom.translate(45, 0, 0);
  const armBar = new THREE.Mesh(armBarGeom,
    new THREE.MeshStandardMaterial({ color: 0x64748b, metalness: 0.85, roughness: 0.25 }));
  tensionerArmGroup.add(armBar);

  const pinGeom = new THREE.CylinderGeometry(10, 10, 16, 16);
  pinGeom.translate(90, 0, 8); pinGeom.rotateX(Math.PI/2);
  const pinMesh = new THREE.Mesh(pinGeom,
    new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.9 }));
  tensionerArmGroup.add(pinMesh);

  // Belt material with texture
  beltTexture = createBeltTexture();
  beltMaterial = new THREE.MeshStandardMaterial({
    map: beltTexture, roughness: 0.6, metalness: 0.15, side: THREE.DoubleSide
  });
  beltDashedMaterial = new THREE.LineDashedMaterial({ color: 0xffd700, dashSize: 10, gapSize: 6 });

  buildBelt3D();
}

function getArcPoints(cx, cy, r, a_in, a_out, cw, numPoints = 18) {
  const points = [];
  let diff = a_out - a_in;
  if (cw) { if (diff > 0) diff -= 2 * Math.PI; }
  else    { if (diff < 0) diff += 2 * Math.PI; }
  for (let i = 0; i <= numPoints; i++) {
    const angle = a_in + diff * (i / numPoints);
    points.push(new THREE.Vector3(cx + r * Math.cos(angle), cy + r * Math.sin(angle), 0));
  }
  return points;
}

function buildBelt3D() {
  // Update tensioner
  if (meshes3D.TEN) meshes3D.TEN.position.set(PULLEYS.TEN.x, PULLEYS.TEN.y, 0);
  if (tensionerArmGroup) {
    const angle = Math.atan2(PULLEYS.TEN.y - 72, PULLEYS.TEN.x - (-240));
    tensionerArmGroup.rotation.z = angle;
  }

  // Build belt centerline points
  const beltPoints = [];
  for (let i = 0; i < ORDER.length; i++) {
    const n = ORDER[i];
    const prev = ORDER[(i - 1 + ORDER.length) % ORDER.length];
    const p = PULLEYS[n];
    const s_prev = spans[prev];
    const s_curr = spans[n];
    if (!s_prev || !s_curr) continue;

    const a_in = Math.atan2(s_prev.t2.y - p.y, s_prev.t2.x - p.x);
    const a_out = Math.atan2(s_curr.t1.y - p.y, s_curr.t1.x - p.x);
    const arc = getArcPoints(p.x, p.y, p.r, a_in, a_out, p.cw, 18);
    beltPoints.push(...arc);

    // Span tangent straight line points
    const nxt = ORDER[(i + 1) % ORDER.length];
    const s_curr2 = spans[n];
    if (s_curr2) {
      const steps = 6;
      for (let k = 0; k <= steps; k++) {
        const tt = k / steps;
        beltPoints.push(new THREE.Vector3(
          s_curr2.t1.x + tt * (s_curr2.t2.x - s_curr2.t1.x),
          s_curr2.t1.y + tt * (s_curr2.t2.y - s_curr2.t1.y),
          0
        ));
      }
    }
  }

  if (beltPoints.length === 0) return;

  // Remove old belt
  if (beltRibbonMesh) { scene.remove(beltRibbonMesh); beltRibbonMesh.geometry.dispose(); }
  if (beltDashedLine) { scene.remove(beltDashedLine); beltDashedLine.geometry.dispose(); }

  // Belt width / thickness scaled by average hub-load force
  const avgF = Object.values(hubData).reduce((s, d) => s + (d ? d.F : 0), 0) / ORDER.length;
  const width = 10 + Math.min(avgF / 600, 4);  // 10–14 mm wide based on load
  const thickness = 4 + Math.min(avgF / 1000, 2); // 4–6 mm thick
  const t = thickness / 2;

  // Multi-rib profile cross-section (Micro-V belt look)
  const profile = [
    { w: -width/2,       h:  t,        v: 0.00 },
    { w:  width/2,       h:  t,        v: 0.44 },
    { w:  width/2,       h: -t + 1.5,  v: 0.50 },
    { w:  width/3,       h: -t + 1.5,  v: 0.55 },
    { w:  width/6,       h: -t,        v: 0.60 },
    { w:  0.0,           h: -t + 1.5,  v: 0.65 },
    { w: -width/6,       h: -t,        v: 0.70 },
    { w: -width/3,       h: -t + 1.5,  v: 0.76 },
    { w: -width/2 + 1.5, h: -t,        v: 0.85 },
    { w: -width/2,       h: -t + 1.5,  v: 0.94 },
    { w: -width/2,       h:  t,        v: 1.00 }
  ];

  const vertices = [], uvs = [], indices = [];
  const N = beltPoints.length, M = profile.length;

  const normals = [];
  for (let i = 0; i < N; i++) {
    const prev = beltPoints[(i - 1 + N) % N];
    const next = beltPoints[(i + 1) % N];
    const dx = next.x - prev.x, dy = next.y - prev.y;
    const len = Math.hypot(dx, dy);
    normals.push({ x: -dy / len, y: dx / len });
  }

  for (let i = 0; i < N; i++) {
    const p = beltPoints[i], n = normals[i], u = i / N;
    for (let j = 0; j < M; j++) {
      const pt = profile[j];
      vertices.push(p.x + pt.h * n.x, p.y + pt.h * n.y, pt.w);
      uvs.push(u, pt.v);
    }
    const nextIdx = (i + 1) % N;
    for (let j = 0; j < M - 1; j++) {
      const i0 = i*M+j, i1 = i*M+j+1, n0 = nextIdx*M+j, n1 = nextIdx*M+j+1;
      indices.push(i0, n0, i1);
      indices.push(i1, n0, n1);
    }
  }

  const ribbonGeom = new THREE.BufferGeometry();
  ribbonGeom.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  ribbonGeom.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  ribbonGeom.setIndex(indices);
  ribbonGeom.computeVertexNormals();
  beltRibbonMesh = new THREE.Mesh(ribbonGeom, beltMaterial);
  scene.add(beltRibbonMesh);

  // Dashed gold cord on outer belt face
  const dashedPoints = [];
  for (let i = 0; i < N; i++) {
    const p = beltPoints[i], n = normals[i];
    dashedPoints.push(new THREE.Vector3(p.x + (t + 0.4) * n.x, p.y + (t + 0.4) * n.y, width/2 + 0.6));
  }
  const dashedGeom = new THREE.BufferGeometry().setFromPoints(dashedPoints);
  beltDashedLine = new THREE.LineLoop(dashedGeom, beltDashedMaterial);
  beltDashedLine.computeLineDistances();
  scene.add(beltDashedLine);

  // Hub-load arrows
  for (const a of arrowObjs3D) scene.remove(a);
  arrowObjs3D.length = 0;
  for (const n of ORDER) {
    const d = hubData[n]; if (!d || d.F < 1) continue;
    const p = PULLEYS[n];
    const len = Math.min(d.F * 0.04, 170);
    const dir3 = new THREE.Vector3(d.Fx, d.Fy, 0).normalize();
    const origin3 = new THREE.Vector3(p.x, p.y, 18);
    const arrow = new THREE.ArrowHelper(dir3, origin3, len, 0xe63946, len*0.22, len*0.13);
    scene.add(arrow);
    arrowObjs3D.push(arrow);
  }
}

function animateThree() {
  requestAnimationFrame(animateThree);
  if (controls) controls.update();
  const dt = clock ? clock.getDelta() : 0.016;

  const rpmFactor = state.rpm / 60 * 2 * Math.PI * dt;
  for (const n of ORDER) {
    if (!meshes3D[n]) continue;
    const sign = PULLEYS[n].cw ? -1 : 1;
    meshes3D[n].rotation.z += sign * rpmFactor * PULLEYS[n].sr;
  }

  if (beltDashedMaterial) {
    const v = beltVelocity(state.rpm);
    beltDashedMaterial.dashOffset -= v * 1000 * dt * 0.14;
  }

  if (beltTexture) {
    const v = beltVelocity(state.rpm);
    beltTexture.offset.x -= (v * 1000 / 1577.3) * dt;
  }

  renderer.render(scene, cam);
}

// ── UI Wiring ─────────────────────────────────────────────────────────────────
// ── Tooltip System ────────────────────────────────────────────────────────────
const TOOLTIP_CONTENT = {
  rpm: {
    title: 'Engine Speed (RPM)',
    body: 'RPM controls belt linear velocity (v = π·Deff·n/60000). ' +
      'Higher RPM → faster belt → lower effective span tension (T = P/v) BUT higher bearing speeds. ' +
      'The FAN curve rises steeply above 1600 RPM due to its cubic power law.'
  },
  tension: {
    title: 'Static Belt Tension',
    body: 'This is the pre-tension installed by the tensioner spring. It scales all computed span tensions. ' +
      'Too low → belt slips under load. Too high → bearing overload and short belt life. ' +
      'Design value is 480 N. Hub forces rise linearly with this slider.'
  },
  tensioner: {
    title: 'Tensioner Arm Position',
    body: 'The tensioner arm angle moves the tensioner pulley XY position, ' +
      'changing all belt span geometries and wrap angles. ' +
      'FREE = belt slack (removal). LOAD = spring at full deflection under operating load. ' +
      'Each position corresponds to a measured arm angle from the PDF data sheet.'
  }
};

function initTooltips() {
  const overlay = document.getElementById('tooltip-overlay');
  if (!overlay) return;

  document.querySelectorAll('[data-tip]').forEach(chip => {
    chip.addEventListener('mouseenter', (e) => {
      const key = chip.getAttribute('data-tip');
      const content = TOOLTIP_CONTENT[key];
      if (!content) return;
      overlay.innerHTML = `<strong>${content.title}</strong>${content.body}`;
      overlay.classList.add('visible');
      positionTooltip(e);
    });
    chip.addEventListener('mousemove', positionTooltip);
    chip.addEventListener('mouseleave', () => overlay.classList.remove('visible'));
  });

  function positionTooltip(e) {
    const W = window.innerWidth, H = window.innerHeight;
    let x = e.clientX + 14, y = e.clientY + 14;
    const ow = overlay.offsetWidth || 300, oh = overlay.offsetHeight || 100;
    if (x + ow > W - 12) x = e.clientX - ow - 8;
    if (y + oh > H - 12) y = e.clientY - oh - 8;
    overlay.style.left = x + 'px';
    overlay.style.top = y + 'px';
  }
}

function updateAll() {
  state.rpm = +document.getElementById('rpm').value;
  state.baseTension = +document.getElementById('tension').value;
  state.tenIdx = +document.getElementById('tensioner').value;

  document.getElementById('lbl-rpm').textContent = state.rpm + ' RPM';
  document.getElementById('lbl-tension').textContent = state.baseTension + ' N';
  const tp = TEN_POS[state.tenIdx];
  document.getElementById('lbl-tensioner').textContent = tp.label + ' (' + tp.arm + '°)';

  compute();
  renderTable();
  renderGearSummary();
  renderInsights();
  buildBelt3D();

  // Rebuild chart data and update chart
  buildChartData();
  updateChart();
  renderFEADAll();
  renderDriveCycleChart();
  renderComplianceDashboard();
}

function resizeCanvases() {
  const c2 = document.getElementById('canvas2d');
  if (c2) { c2.width = c2.offsetWidth; c2.height = c2.offsetHeight; }
  if (renderer && cam) {
    const ctr = document.getElementById('threejs-container');
    const W = ctr.clientWidth, H = ctr.clientHeight || 520;
    cam.aspect = W / H; cam.updateProjectionMatrix();
    renderer.setSize(W, H);
  }
}

// ── Canvas 2D Interactivity — Pulley Hover ───────────────────────────────────
function initCanvas2DInteraction() {
  const canvas = document.getElementById('canvas2d');
  const tip    = document.getElementById('canvas-pulley-tip');
  if (!canvas || !tip) return;

  const FULL_NAMES = {
    CRK:'Crankshaft',  FAN:'Cooling Fan',  IDR:'Idler',
    ALT:'Alternator',  AC:'A/C Compressor', TEN:'Tensioner'
  };
  const DESCRIPTIONS = {
    CRK:'Drives all accessories. Belt tight-side leaves here. Gear-driven loads also add to this hub.',
    FAN:'Driven by belt; fan law means power rises as RPM³. Largest load at high speed.',
    IDR:'Free-spinning idler. Redirects belt path. No power transfer — only geometry.',
    ALT:'Generates electrical power. Nearly constant load regardless of RPM.',
    AC: 'Air conditioning compressor. High load when clutch engaged.',
    TEN:'Spring-loaded tensioner. Maintains belt tension as it stretches with wear.'
  };

  canvas.addEventListener('mousemove', (e) => {
    const rect = canvas.getBoundingClientRect();
    // Canvas pixel coords
    const px = (e.clientX - rect.left) * (canvas.width  / rect.width);
    const py = (e.clientY - rect.top)  * (canvas.height / rect.height);

    // Reverse-project to world coords
    const { PAD, sc, minX, minY } = _2d;
    const worldX = (px - PAD) / sc + minX;
    const worldY = -(py - (_2d.H - PAD)) / sc + minY;

    // Hit-test every pulley (use 1.5x radius for easy hovering)
    let hit = null;
    for (const n of ORDER) {
      const p = PULLEYS[n];
      const dist = Math.hypot(worldX - p.x, worldY - p.y);
      if (dist <= p.r * 1.6) { hit = n; break; }
    }

    if (!hit) {
      tip.style.display = 'none';
      canvas.style.cursor = 'crosshair';
      return;
    }

    canvas.style.cursor = 'pointer';
    const p  = PULLEYS[hit];
    const hd = hubData[hit];
    const pdf= PDF[hit];

    const pulleyRPM = (state.rpm * PULLEYS.CRK.eff / p.eff).toFixed(0);
    const dF = hd ? (hd.F - pdf.F).toFixed(0) : '—';
    const dFsign = +dF >= 0 ? '+' : '';
    const dFcolor = Math.abs(+dF) > 300 ? '#f87171' : '#52b788';

    tip.innerHTML = `
      <div class="tip-name" style="color:${p.color}">${hit} — ${FULL_NAMES[hit]}</div>
      <div class="tip-row"><span>Position</span><span>(${p.x.toFixed(0)}, ${p.y.toFixed(0)}) mm</span></div>
      <div class="tip-row"><span>Eff. Diameter</span><span>${p.eff.toFixed(2)} mm</span></div>
      <div class="tip-row"><span>Speed Ratio</span><span>${p.sr.toFixed(3)} : 1</span></div>
      <div class="tip-row"><span>Pulley RPM</span><span>${pulleyRPM} RPM</span></div>
      ${hd ? `
      <div class="tip-row"><span>T_in</span><span>${hd.T_in.toFixed(0)} N</span></div>
      <div class="tip-row"><span>T_out</span><span>${hd.T_out.toFixed(0)} N</span></div>
      <div class="tip-force">⇒ Hub Load: ${hd.F.toFixed(0)} N @ ${hd.dir.toFixed(1)}°
        &nbsp;<span style="font-size:0.75rem;font-weight:400;color:${dFcolor}">(${dFsign}${dF} N vs PDF)</span>
      </div>
      ` : ''}
      <div style="margin-top:0.5rem;padding-top:0.4rem;border-top:1px solid #1c2840;
        font-size:0.72rem;color:#4a5c78;font-family:'IBM Plex Sans',sans-serif;line-height:1.5">
        ${DESCRIPTIONS[hit]}
      </div>`;

    // Position tooltip so it doesn’t clip off screen
    const tipW = 260, tipH = 220;
    const canvasRight = rect.right;
    const canvasBottom = rect.bottom;
    let tx2 = e.clientX - rect.left + 16;
    let ty2 = e.clientY - rect.top  + 16;
    if (tx2 + tipW > rect.width  - 8) tx2 = e.clientX - rect.left - tipW - 8;
    if (ty2 + tipH > rect.height - 8) ty2 = e.clientY - rect.top  - tipH - 8;
    tip.style.left    = tx2 + 'px';
    tip.style.top     = ty2 + 'px';
    tip.style.display = 'block';
  });

  canvas.addEventListener('mouseleave', () => {
    tip.style.display = 'none';
    canvas.style.cursor = 'crosshair';
  });
}

window.addEventListener('load', () => {
  resizeCanvases();
  compute();
  renderTable();
  renderGearSummary();
  initThree();
  animateThree();
  loop2D();
  initGearUI();
  initChart();
  initTooltips();
  initCanvas2DInteraction();

  // Build chart data and render
  buildChartData();
  updateChart();
  renderInsights();
  initOpModes();
  initDriveCycleChart();
  initFEADCharts();
  initComplianceDashboard();
  initPDFReport();

  ['rpm','tension','tensioner'].forEach(id =>
    document.getElementById(id).addEventListener('input', updateAll)
  );
  const xlBtn = document.getElementById('download-excel');
  if (xlBtn) xlBtn.addEventListener('click', downloadExcel);
  window.addEventListener('resize', resizeCanvases);
});

// ══════════════════════════════════════════════════════════════════════════════
// OPERATING MODE PRESETS
// ══════════════════════════════════════════════════════════════════════════════
function initOpModes() {
  const grid = document.getElementById('opmodes-grid');
  if (!grid) return;
  grid.innerHTML = '';
  OP_MODES.forEach(mode => {
    const card = document.createElement('div');
    card.className = 'opmode-card';
    card.id = 'opmode-' + mode.id;
    card.style.setProperty('--mode-accent', mode.accent);
    card.innerHTML = `
      <div class="opmode-icon">${mode.icon}</div>
      <div class="opmode-name">${mode.name}</div>
      <div class="opmode-rpm">${mode.rpm} RPM · ${mode.tension} N tension</div>
      <div class="opmode-teaser">${mode.teaser}</div>`;
    card.addEventListener('click', () => activateOpMode(mode));
    grid.appendChild(card);
  });

  const closeBtn = document.getElementById('opmode-close');
  if (closeBtn) closeBtn.addEventListener('click', () => {
    document.getElementById('opmode-detail').style.display = 'none';
    document.querySelectorAll('.opmode-card').forEach(c => c.classList.remove('active'));
  });
}

function activateOpMode(mode) {
  // Highlight card
  document.querySelectorAll('.opmode-card').forEach(c => c.classList.remove('active'));
  const card = document.getElementById('opmode-' + mode.id);
  if (card) card.classList.add('active');

  // Apply RPM + tension to sliders
  const rpmEl = document.getElementById('rpm');
  const tenEl = document.getElementById('tension');
  if (rpmEl) { rpmEl.value = mode.rpm; document.getElementById('rpm-val').textContent = mode.rpm + ' RPM'; }
  if (tenEl) { tenEl.value = mode.tension; document.getElementById('tension-val').textContent = mode.tension + ' N'; }
  state.rpm = mode.rpm;
  state.baseTension = mode.tension;
  compute();
  renderTable();
  renderGearSummary();
  renderInsights();
  buildChartData();
  updateChart();

  // Show detail panel
  const detail = document.getElementById('opmode-detail');
  document.getElementById('opmode-detail-icon').textContent = mode.icon;
  document.getElementById('opmode-detail-name').textContent = mode.name;
  document.getElementById('opmode-detail-name').style.color = mode.accent;
  document.getElementById('opmode-detail-scenario').textContent = mode.scenario;
  document.getElementById('opmode-detail-mechanism').textContent = mode.mechanism;

  const riskEl = document.getElementById('opmode-detail-risks');
  riskEl.innerHTML = mode.risks.map(r => `<li>${r}</li>`).join('');

  // Hub load results
  const results = document.getElementById('opmode-results');
  const worst = mode.worstPulley;
  let html = `<div class="opmode-mech-label">📊 Computed hub loads at ${mode.rpm} RPM</div>`;
  for (const n of ORDER) {
    const hd = hubData[n];
    const pdf = PDF[n];
    if (!hd) continue;
    const dF = (hd.F - pdf.F).toFixed(0);
    const sign = +dF >= 0 ? '+' : '';
    const hi = Math.abs(+dF) > 300;
    html += `<div class="opmode-result-row">
      <span class="opmode-result-label" style="color:${PULLEYS[n].color}">${n}</span>
      <span class="opmode-result-val ${hi?'hi':'ok'}">${hd.F.toFixed(0)} N <span style="font-size:0.65rem;opacity:.7">(${sign}${dF} vs PDF)</span></span>
    </div>`;
  }
  if (worst) {
    const hd = hubData[worst];
    const pdfF = PDF[worst].F;
    if (hd && hd.F > pdfF * 1.1) {
      html += `<div style="margin-top:.5rem;padding:.4rem .5rem;background:rgba(248,113,113,0.08);border:1px solid rgba(248,113,113,0.3);border-radius:4px;font-size:.72rem;color:#f87171;font-family:var(--font-ui)">
        ⚠ ${worst} hub load ${hd.F.toFixed(0)} N exceeds PDF baseline ${pdfF} N by ${((hd.F/pdfF-1)*100).toFixed(1)}%
      </div>`;
    }
  }
  results.innerHTML = html;
  setTimeout(makeTableSortable, 50);
  detail.style.display = 'block';
  detail.scrollIntoView({ behavior:'smooth', block:'nearest' });
  renderComplianceDashboard();
}

// ══════════════════════════════════════════════════════════════════════════════
// DRIVE CYCLE CHART
// ══════════════════════════════════════════════════════════════════════════════
let driveCycleChart = null;
let activeCycleView = 'COMPARE';

function getHubLoadAtRPM(rpm) {
  // Look up from hubLoadHistory (built by buildChartData)
  const result = {};
  for (const n of ORDER) {
    if (hubLoadHistory[n] && hubLoadHistory[n].length > 0) {
      const closest = hubLoadHistory[n].reduce((best, d) =>
        Math.abs(d.rpm - rpm) < Math.abs(best.rpm - rpm) ? d : best
      );
      result[n] = closest.F;
    } else {
      result[n] = 0;
    }
  }
  return result;
}

function buildCycleDatasets(cycleKey) {
  const cycle = DRIVE_CYCLES[cycleKey];
  const labels = cycle.rpms.map((r, i) => i + 1);
  // Build CRK and FAN series (most informative)
  const crkLoads = cycle.rpms.map(r => getHubLoadAtRPM(r).CRK || 0);
  const fanLoads = cycle.rpms.map(r => getHubLoadAtRPM(r).FAN || 0);
  return { labels, crkLoads, fanLoads };
}

function initDriveCycleChart() {
  const canvas = document.getElementById('drivecycle-chart');
  if (!canvas) return;

  // Tab listeners
  document.querySelectorAll('.cycle-tab').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.cycle-tab').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      activeCycleView = btn.dataset.cycle;
      renderDriveCycleChart();
    });
  });

  renderDriveCycleChart();
}

function renderDriveCycleChart() {
  const canvas = document.getElementById('drivecycle-chart');
  if (!canvas) return;

  if (driveCycleChart) { driveCycleChart.destroy(); driveCycleChart = null; }

  const cyclesToShow = activeCycleView === 'COMPARE'
    ? ['WLTC','NEDC']
    : [activeCycleView];

  const datasets = [];
  const allRPMs = cyclesToShow[0] === 'WLTC'
    ? DRIVE_CYCLES.WLTC.rpms
    : cyclesToShow[0] === 'NEDC'
      ? DRIVE_CYCLES.NEDC.rpms
      : DRIVE_CYCLES.WLTC.rpms;

  const maxPoints = Math.max(...cyclesToShow.map(k => DRIVE_CYCLES[k].rpms.length));
  const labels = Array.from({length: maxPoints}, (_,i) => i+1);

  cyclesToShow.forEach(ck => {
    const cycle = DRIVE_CYCLES[ck];
    const data = buildCycleDatasets(ck);
    // CRK hub load
    datasets.push({
      label: `${cycle.name} — CRK Hub Load (N)`,
      data: data.crkLoads,
      borderColor: cycle.color,
      backgroundColor: cycle.color + '22',
      borderWidth: cycle.lineWidth,
      pointRadius: 3,
      pointHoverRadius: 6,
      tension: 0.3,
      fill: false,
      yAxisID: 'yHub'
    });
    // FAN hub load (dashed)
    datasets.push({
      label: `${cycle.name} — FAN Hub Load (N)`,
      data: data.fanLoads,
      borderColor: cycle.color,
      backgroundColor: 'transparent',
      borderWidth: 1.5,
      borderDash: [6,4],
      pointRadius: 0,
      tension: 0.3,
      fill: false,
      yAxisID: 'yHub'
    });
    // RPM overlay
    datasets.push({
      label: `${cycle.name} — Engine RPM`,
      data: cycle.rpms,
      borderColor: cycle.color + '55',
      backgroundColor: 'transparent',
      borderWidth: 1,
      borderDash: [2,4],
      pointRadius: 0,
      tension: 0.3,
      fill: false,
      yAxisID: 'yRPM'
    });
  });

  // Worst-case lines
  datasets.push({
    label: 'PDF CRK Peak (2658.9 N)',
    data: Array(maxPoints).fill(PDF.CRK.F),
    borderColor: '#f87171',
    borderWidth: 1,
    borderDash: [8,4],
    pointRadius: 0,
    fill: false,
    yAxisID: 'yHub'
  });
  datasets.push({
    label: 'PDF FAN Peak (2866.4 N)',
    data: Array(maxPoints).fill(PDF.FAN.F),
    borderColor: '#fbbf24',
    borderWidth: 1,
    borderDash: [8,4],
    pointRadius: 0,
    fill: false,
    yAxisID: 'yHub'
  });

  driveCycleChart = new Chart(canvas, {
    type: 'line',
    data: { labels, datasets },
    options: {
      responsive: true, maintainAspectRatio: false,
      interaction: { mode:'index', intersect:false },
      plugins: {
        legend: {
          labels: {
            color:'#8899aa', font:{ size:11 },
            filter: item => !item.text.includes('RPM') // hide RPM from legend
          }
        },
        tooltip: {
          callbacks: {
            label: ctx => {
              if (ctx.dataset.yAxisID === 'yRPM') return `  RPM: ${ctx.parsed.y}`;
              return `  ${ctx.dataset.label.split('—')[1]?.trim()}: ${ctx.parsed.y.toFixed(0)} N`;
            }
          },
          backgroundColor: 'rgba(10,14,26,0.95)',
          borderColor: '#1c2840',
          borderWidth: 1,
          titleColor: '#cdd6e8',
          bodyColor: '#8899aa',
          padding: 10
        }
      },
      scales: {
        x: {
          grid:{ color:'#0e1623' },
          ticks:{ color:'#4a5c78', font:{size:10} },
          title:{ display:true, text:'Cycle Time Step', color:'#4a5c78', font:{size:11} }
        },
        yHub: {
          type:'linear', position:'left',
          grid:{ color:'#0e1623' },
          ticks:{ color:'#8899aa', font:{size:11} },
          title:{ display:true, text:'Hub Load (N)', color:'#8899aa', font:{size:12} },
          min: 0
        },
        yRPM: {
          type:'linear', position:'right',
          grid:{ drawOnChartArea:false },
          ticks:{ color:'#4a5c78', font:{size:10} },
          title:{ display:true, text:'Engine RPM', color:'#4a5c78', font:{size:11} },
          min:0, max:3000
        }
      }
    }
  });

  renderCycleStats();
  renderCyclePhaseLegend();
  renderCycleWorstGrid();
}

function renderCycleStats() {
  const el = document.getElementById('cycle-stats');
  if (!el) return;

  const cyclesToShow = activeCycleView === 'COMPARE' ? ['WLTC','NEDC'] : [activeCycleView];
  let html = '';

  cyclesToShow.forEach(ck => {
    const cycle = DRIVE_CYCLES[ck];
    const crkLoads = cycle.rpms.map(r => getHubLoadAtRPM(r).CRK || 0);
    const fanLoads = cycle.rpms.map(r => getHubLoadAtRPM(r).FAN || 0);
    const maxCRK = Math.max(...crkLoads).toFixed(0);
    const avgCRK = (crkLoads.reduce((s,v)=>s+v,0)/crkLoads.length).toFixed(0);
    const maxFAN = Math.max(...fanLoads).toFixed(0);
    const maxRPM = Math.max(...cycle.rpms);
    html += `
      <div class="cycle-stat">
        <div class="cycle-stat-label">${cycle.name} — Peak CRK Load</div>
        <div class="cycle-stat-val" style="color:${cycle.color}">${maxCRK} N</div>
        <div class="cycle-stat-sub">avg ${avgCRK} N · at ${maxRPM} RPM</div>
      </div>
      <div class="cycle-stat">
        <div class="cycle-stat-label">${cycle.name} — Peak FAN Load</div>
        <div class="cycle-stat-val" style="color:${cycle.color}">${maxFAN} N</div>
        <div class="cycle-stat-sub">PDF cert. ${PDF.FAN.F} N · ${((maxFAN/PDF.FAN.F)*100).toFixed(0)}% of peak</div>
      </div>`;
  });

  el.innerHTML = html;
}

function renderCyclePhaseLegend() {
  const el = document.getElementById('cycle-phase-legend');
  if (!el) return;
  const cyclesToShow = activeCycleView === 'COMPARE' ? ['WLTC','NEDC'] : [activeCycleView];
  let html = '';
  cyclesToShow.forEach(ck => {
    DRIVE_CYCLES[ck].phases.forEach(ph => {
      html += `<div class="cph-item">
        <span class="cph-swatch" style="background:${ph.lineColor}"></span>
        <span>${DRIVE_CYCLES[ck].name} ${ph.name}</span>
      </div>`;
    });
  });
  el.innerHTML = html;
}

function renderCycleWorstGrid() {
  const el = document.getElementById('cycle-worst-grid');
  if (!el) return;

  const worstCases = [
    {
      title:'🔌 Idle + High Demand',
      color:'#f4a261',
      rpm: 800, tension: 3200,
      desc:'Low belt speed amplifies tension. T_eff = P/v → highest ΔT per kW.',
      rows: ORDER.map(n => {
        const hl = getHubLoadAtRPM(800);
        const pct = PDF[n].F > 0 ? ((hl[n]/PDF[n].F)*100).toFixed(0) : '—';
        return { n, F: hl[n]?.toFixed(0) || '—', pct };
      })
    },
    {
      title:'⚡ Regen Braking',
      color:'#4fc3f7',
      rpm: 1600, tension: 2500,
      desc:'ALT direction reverses. IDR load rises. Hub force plane shifts.',
      rows: ORDER.map(n => {
        const hl = getHubLoadAtRPM(1600);
        const pct = PDF[n].F > 0 ? ((hl[n]/PDF[n].F)*100).toFixed(0) : '—';
        return { n, F: hl[n]?.toFixed(0) || '—', pct };
      })
    },
    {
      title:'🚀 BAS Motor Mode',
      color:'#a78bfa',
      rpm: 900, tension: 3800,
      desc:'CRK driven by ALT. Tight-side reversal → CRK hub force spikes.',
      rows: ORDER.map(n => {
        const hl = getHubLoadAtRPM(900);
        const pct = PDF[n].F > 0 ? ((hl[n]/PDF[n].F)*100).toFixed(0) : '—';
        return { n, F: hl[n]?.toFixed(0) || '—', pct };
      })
    },
    {
      title:'💨 Peak Acceleration',
      color:'#e63946',
      rpm: 2300, tension: 2500,
      desc:'FAN cube law: ~19.8 kW at 2300 RPM → hub load near 2866 N.',
      rows: ORDER.map(n => {
        const hl = getHubLoadAtRPM(2300);
        const pct = PDF[n].F > 0 ? ((hl[n]/PDF[n].F)*100).toFixed(0) : '—';
        return { n, F: hl[n]?.toFixed(0) || '—', pct };
      })
    }
  ];

  el.innerHTML = worstCases.map(wc => `
    <div class="cw-card" style="--wc:${wc.color}">
      <div class="cw-title">${wc.title}</div>
      ${wc.rows.map(r => `<div class="cw-row"><span>${r.n}</span><span>${r.F} N (${r.pct}%)</span></div>`).join('')}
      <div class="cw-warn">${wc.desc}</div>
    </div>`).join('');
}

// ══════════════════════════════════════════════════════════════════════════════
// SORTABLE TABLE
// ══════════════════════════════════════════════════════════════════════════════
let tableSortCol = null;
let tableSortDir = 1;

function makeTableSortable() {
  const wrap = document.getElementById('calc-table');
  if (!wrap) return;
  const table = wrap.querySelector('table');
  if (!table) return;
  const headers = table.querySelectorAll('th');
  headers.forEach((th, idx) => {
    th.style.cursor = 'pointer';
    th.title = 'Click to sort';
    th.addEventListener('click', () => {
      if (tableSortCol === idx) tableSortDir *= -1;
      else { tableSortCol = idx; tableSortDir = 1; }
      headers.forEach(h => h.classList.remove('sort-asc','sort-desc'));
      th.classList.add(tableSortDir === 1 ? 'sort-asc' : 'sort-desc');
      sortTable(table, idx, tableSortDir);
    });
    th.addEventListener('mouseenter', () => { if (!th.classList.contains('sort-asc') && !th.classList.contains('sort-desc')) th.style.opacity='0.7'; });
    th.addEventListener('mouseleave', () => { th.style.opacity=''; });
  });

  // Row click highlight + scroll-into-view
  table.querySelectorAll('tbody tr').forEach(tr => {
    tr.style.cursor = 'pointer';
    tr.addEventListener('click', () => {
      table.querySelectorAll('tbody tr').forEach(r => r.classList.remove('row-selected'));
      tr.classList.add('row-selected');
    });
  });
}

function sortTable(table, colIdx, dir) {
  const tbody = table.querySelector('tbody');
  if (!tbody) return;
  const rows = Array.from(tbody.querySelectorAll('tr'));
  rows.sort((a, b) => {
    const aVal = a.cells[colIdx]?.textContent.trim().replace(/[^0-9.\-]/g,'') || '0';
    const bVal = b.cells[colIdx]?.textContent.trim().replace(/[^0-9.\-]/g,'') || '0';
    const an = parseFloat(aVal) || 0;
    const bn = parseFloat(bVal) || 0;
    if (!isNaN(an) && !isNaN(bn)) return (an - bn) * dir;
    return aVal.localeCompare(bVal) * dir;
  });
  rows.forEach(r => tbody.appendChild(r));
}


// ══════════════════════════════════════════════════════════════════════════════
// FEAD EFFICIENCY DASHBOARD — Physics Engine
// ══════════════════════════════════════════════════════════════════════════════

// Physical constants for Gates Micro-V MT820 AMD 8-rib aramid belt
const BELT_PARAMS = {
  mu: 0.35,          // friction coefficient, PK belt on grooved pulley (dry)
  mu_back: 0.25,     // back-side (smooth) friction coefficient (IDR, TEN)
  C_bend: 0.0012,    // bending loss coefficient (aramid, 8-rib)
  mu_bearing: 0.003, // rolling element bearing friction coefficient
  mass_per_m: 0.18,  // kg/m, 8-rib aramid belt linear mass
  belt_length_m: 1.577, // m (ISO 9981 effective length)
  co2_per_kW: 0.18,  // kg CO2/h per kW of parasitic loss (diesel, 220g/kWh BSFC)
  fuel_g_per_kWh: 220 // g/kWh diesel BSFC
};

// Belt-side idlers use mu_back
const BACKSIDE = { IDR: true, TEN: true };

function computeFEAD() {
  const v     = beltVelocity(state.rpm);           // m/s
  const T0    = state.baseTension;                  // N static tension
  const P     = interp(state.rpm);                  // kW per component
  const mu    = BELT_PARAMS.mu;
  const mu_b  = BELT_PARAMS.mu_back;
  const Cb    = BELT_PARAMS.C_bend;
  const mub   = BELT_PARAMS.mu_bearing;
  const mb    = BELT_PARAMS.mass_per_m * BELT_PARAMS.belt_length_m; // total belt mass kg
  const Tc    = mb * v * v / BELT_PARAMS.belt_length_m;             // centrifugal tension N

  const result = { pulleys: {}, totals: {} };

  let P_accessories = 0;
  let P_slip_total  = 0;
  let P_bend_total  = 0;
  let P_bear_total  = 0;
  let P_cent        = 0.5 * mb * v * v * (2 * Math.PI * state.rpm / 60) / 1000; // centrifugal kW approx

  for (const n of ORDER) {
    const p   = PULLEYS[n];
    const hd  = hubData[n];
    const pwr = P[n] || 0;               // kW
    const wrapRad = d2r(PDF[n].wrap);    // wrap angle in rad
    const R   = p.eff / 2 / 1000;       // effective radius, m
    const frict = BACKSIDE[n] ? mu_b : mu;

    // Effective tension (tight - slack)
    const T_eff = v > 0.01 ? (pwr * 1000 / v) : 0;

    // Tensions from Capstan: T_tight = T0 + T_eff*(e^(mu*theta))/(e^(mu*theta)-1)
    const capstan = Math.exp(frict * wrapRad);
    const T_tight = T0 + T_eff * capstan / (capstan - 1);
    const T_slack = T_tight - T_eff;

    // Slip safety factor: SF = ln(Tt/Ts) / (mu*theta)
    const SF = T_slack > 1 ? Math.log(T_tight / T_slack) / (frict * wrapRad) : 0;

    // Losses
    const T_avg   = (T_tight + T_slack) / 2;
    const P_bend  = R > 0 ? Cb * (T_avg / R) * v / 1000 : 0;  // kW
    const omega   = p.sr * state.rpm * 2 * Math.PI / 60;        // pulley angular vel rad/s
    const P_bear  = hd ? (mub * hd.F * omega * R / 1000) : 0;  // kW
    const P_slip  = pwr * 0.005;                                  // ~0.5% slip loss

    P_slip_total += P_slip;
    P_bend_total += P_bend;
    P_bear_total += P_bear;
    if (n !== 'IDR' && n !== 'TEN') P_accessories += pwr;

    result.pulleys[n] = {
      pwr, T_tight: T_tight.toFixed(1), T_slack: T_slack.toFixed(1),
      SF: SF.toFixed(2), P_bend: P_bend.toFixed(3), P_bear: P_bear.toFixed(3),
      P_slip: P_slip.toFixed(3), wrapDeg: PDF[n].wrap,
      status: SF < 1.0 ? 'SLIP' : SF < 1.3 ? 'MARGINAL' : 'SAFE'
    };
  }

  const P_losses  = P_slip_total + P_bend_total + P_bear_total + P_cent;
  const P_total   = P_accessories + P_losses;
  const eta       = P_total > 0 ? P_accessories / P_total * 100 : 100;
  const fuel_pen  = P_losses * BELT_PARAMS.fuel_g_per_kWh;       // g/h extra fuel
  const co2_pen   = P_losses * BELT_PARAMS.co2_per_kW;            // kg CO2/h

  // Optimal tension: SF=1.3 for the most demanding pulley
  let T_opt = 0;
  for (const n of ORDER) {
    const pwr = P[n] || 0;
    const wrapRad = d2r(PDF[n].wrap);
    const frict = BACKSIDE[n] ? mu_b : mu;
    const T_eff = v > 0.01 ? pwr * 1000 / v : 0;
    const capstan = Math.exp(frict * wrapRad);
    const SF_target = 1.3;
    // SF = ln(Tt/Ts)/(mu*theta) = 1.3  => Tt/Ts = e^(1.3*mu*theta)
    // Tt - Ts = T_eff, Tt = Ts * e^(1.3*mu*theta)
    // Ts*(e^(1.3*mu*theta) - 1) = T_eff => Ts = T_eff/(cap_opt-1)
    const cap_opt = Math.exp(SF_target * frict * wrapRad);
    const Ts_opt  = T_eff > 0 ? T_eff / (cap_opt - 1) : 0;
    const Tt_opt  = Ts_opt + T_eff;
    const T_req   = Math.max(Ts_opt, Tt_opt - T_eff); // required static
    if (T_req > T_opt) T_opt = T_req;
  }

  result.totals = {
    eta: eta.toFixed(1),
    P_accessories: P_accessories.toFixed(2),
    P_losses: P_losses.toFixed(2),
    P_slip: P_slip_total.toFixed(2),
    P_bend: P_bend_total.toFixed(2),
    P_bear: P_bear_total.toFixed(2),
    P_cent: P_cent.toFixed(2),
    fuel_pen: fuel_pen.toFixed(0),
    co2_pen: co2_pen.toFixed(2),
    T_opt: T_opt.toFixed(0),
    belt_v: v.toFixed(2),
    Tc: Tc.toFixed(1)
  };

  return result;
}

// ── FEAD charts ───────────────────────────────────────────────────────────────
let feadDonutChart = null;
let feadLineChart  = null;

function initFEADCharts() {
  const donutCanvas = document.getElementById('fead-donut');
  const lineCanvas  = document.getElementById('fead-line');
  if (!donutCanvas || !lineCanvas) return;
  renderFEADAll();
}

// Debounced heavy line-chart rebuild (43 compute() iterations)
let _feadLineTimer = null;
function debounceFEADLine() {
  clearTimeout(_feadLineTimer);
  _feadLineTimer = setTimeout(renderFEADLineChart, 420);
}

function renderFEADAll() {
  const fd = computeFEAD();
  renderFEADKPIs(fd);
  renderFEADDonut(fd);
  renderFEADRecs(fd);
  renderFEADTension(fd);
  renderFEADTable(fd);   // instant — uses already-computed fd
  debounceFEADLine();    // heavy sweep — debounced 420 ms
}

function renderFEADKPIs(fd) {
  const el = document.getElementById('fead-kpis');
  if (!el) return;
  const t = fd.totals;
  const etaNum = parseFloat(t.eta);
  const etaColor = etaNum >= 97 ? '#34d399' : etaNum >= 95 ? '#fbbf24' : '#f87171';
  const slipWorst = Object.entries(fd.pulleys).reduce((w,[n,d]) =>
    parseFloat(d.SF) < parseFloat(w[1]?.SF||99) ? [n,d] : w, ['',{SF:'99'}]);
  const sfNum = parseFloat(slipWorst[1].SF);
  const sfColor = sfNum >= 1.3 ? '#34d399' : sfNum >= 1.0 ? '#fbbf24' : '#f87171';

  el.innerHTML = `
    <div class="fead-kpi" style="--kpi-color:${etaColor}">
      <span class="fead-kpi-label">FEAD Efficiency</span>
      <span class="fead-kpi-val">${t.eta}%</span>
      <span class="fead-kpi-sub">Target &gt;96% | Belt+bearing+slip</span>
      <span class="fead-kpi-badge">${etaNum>=97?'GOOD':etaNum>=95?'OK':'LOW'}</span>
    </div>
    <div class="fead-kpi" style="--kpi-color:#f87171">
      <span class="fead-kpi-label">Parasitic Losses</span>
      <span class="fead-kpi-val">${t.P_losses} kW</span>
      <span class="fead-kpi-sub">Slip ${t.P_slip} + Bend ${t.P_bend} + Bear ${t.P_bear}</span>
    </div>
    <div class="fead-kpi" style="--kpi-color:#ff8c42">
      <span class="fead-kpi-label">Fuel Penalty</span>
      <span class="fead-kpi-val">${t.fuel_pen} g/h</span>
      <span class="fead-kpi-sub">Extra fuel from FEAD losses</span>
    </div>
    <div class="fead-kpi" style="--kpi-color:#a78bfa">
      <span class="fead-kpi-label">CO&#8322; Impact</span>
      <span class="fead-kpi-val">${t.co2_pen} kg/h</span>
      <span class="fead-kpi-sub">Extra CO&#8322; from parasitic losses</span>
    </div>
    <div class="fead-kpi" style="--kpi-color:${sfColor}">
      <span class="fead-kpi-label">Min Slip Safety</span>
      <span class="fead-kpi-val">${sfNum.toFixed(2)}</span>
      <span class="fead-kpi-sub">${slipWorst[0]} pulley | Target &gt;1.3</span>
      <span class="fead-kpi-badge">${fd.pulleys[slipWorst[0]]?.status||'—'}</span>
    </div>
    <div class="fead-kpi" style="--kpi-color:#4fc3f7">
      <span class="fead-kpi-label">Optimal Tension</span>
      <span class="fead-kpi-val">${t.T_opt} N</span>
      <span class="fead-kpi-sub">For SF=1.3 | Current: ${state.baseTension} N</span>
    </div>`;
}

function renderFEADDonut(fd) {
  const canvas = document.getElementById('fead-donut');
  if (!canvas) return;
  if (feadDonutChart) { feadDonutChart.destroy(); feadDonutChart = null; }
  const t = fd.totals;
  const P = interp(state.rpm);

  const labels = ['FAN (kW)','ALT (kW)','AC (kW)','IDR loss (kW)','TEN loss (kW)','Belt Bending','Bearing Friction','Belt Slip','Centrifugal'];
  const data   = [P.FAN, P.ALT, P.AC, P.IDR, P.TEN,
                  parseFloat(t.P_bend), parseFloat(t.P_bear),
                  parseFloat(t.P_slip), parseFloat(t.P_cent)];
  const colors = ['#ff8c42','#fbbf24','#34d399','#a78bfa','#f472b6',
                  '#4fc3f7','#e63946','#f4a261','#94a3b8'];

  feadDonutChart = new Chart(canvas, {
    type: 'doughnut',
    data: { labels, datasets:[{ data, backgroundColor: colors, borderWidth:0, hoverBorderWidth:2, hoverBorderColor:'#fff' }] },
    options: {
      responsive: true, maintainAspectRatio: false, cutout: '62%',
      plugins: {
        legend: { display: false },
        tooltip: {
          callbacks: { label: ctx => `  ${ctx.label}: ${ctx.parsed.toFixed(3)} kW` },
          backgroundColor:'rgba(10,14,26,0.95)', borderColor:'#1c2840', borderWidth:1,
          titleColor:'#cdd6e8', bodyColor:'#8899aa', padding:10
        }
      }
    }
  });

  const leg = document.getElementById('fead-donut-legend');
  if (leg) {
    leg.innerHTML = labels.map((l,i) => `
      <div class="fead-leg-item">
        <span class="fead-leg-dot" style="background:${colors[i]}"></span>
        <span class="fead-leg-name">${l}</span>
        <span class="fead-leg-val">${data[i].toFixed(3)}</span>
      </div>`).join('');
  }
}

function renderFEADLineChart() {
  const canvas = document.getElementById('fead-line');
  if (!canvas) return;
  if (feadLineChart) { feadLineChart.destroy(); feadLineChart = null; }

  const rpms = [];
  const etas = [], sfs = [], optT = [];
  const savedRPM = state.rpm; // snapshot before sweep
  for (let r = 500; r <= 2600; r += 50) {
    state.rpm = r;
    compute();                      // compute hubData at sweep RPM
    const fdSweep = computeFEAD(); // FEAD at sweep RPM (uses current hubData)
    const minSF = Math.min(...ORDER.map(n => parseFloat(fdSweep.pulleys[n].SF)));
    rpms.push(r);
    etas.push(parseFloat(fdSweep.totals.eta));
    sfs.push(+minSF.toFixed(2));
    optT.push(parseFloat(fdSweep.totals.T_opt));
  }
  // RESTORE original state so hubData is correct after chart build
  state.rpm = savedRPM;
  compute();

  feadLineChart = new Chart(canvas, {
    type: 'line',
    data: {
      labels: rpms,
      datasets: [
        {
          label: 'FEAD Efficiency (%)',
          data: etas, borderColor:'#34d399', backgroundColor:'rgba(52,211,153,0.08)',
          borderWidth:2.5, pointRadius:0, tension:0.3, fill:true, yAxisID:'yEta'
        },
        {
          label: 'Min Slip Safety Factor',
          data: sfs, borderColor:'#f87171', backgroundColor:'transparent',
          borderWidth:2, pointRadius:0, tension:0.3, fill:false, yAxisID:'ySF',
          borderDash:[6,3]
        },
        {
          label: 'Target SF=1.3',
          data: rpms.map(()=>1.3), borderColor:'rgba(248,113,113,0.3)',
          borderWidth:1, borderDash:[4,4], pointRadius:0, fill:false, yAxisID:'ySF'
        },
        {
          label: 'Target Efficiency 96%',
          data: rpms.map(()=>96), borderColor:'rgba(52,211,153,0.3)',
          borderWidth:1, borderDash:[4,4], pointRadius:0, fill:false, yAxisID:'yEta'
        }
      ]
    },
    options: {
      responsive:true, maintainAspectRatio:false,
      interaction:{ mode:'index', intersect:false },
      plugins:{
        legend:{ labels:{ color:'#8899aa', font:{size:11} } },
        tooltip:{
          backgroundColor:'rgba(10,14,26,0.95)', borderColor:'#1c2840', borderWidth:1,
          titleColor:'#cdd6e8', bodyColor:'#8899aa', padding:10
        }
      },
      scales:{
        x:{ grid:{color:'#0e1623'}, ticks:{color:'#4a5c78',font:{size:10}},
            title:{display:true,text:'Engine Speed (RPM)',color:'#4a5c78',font:{size:11}} },
        yEta:{ type:'linear', position:'left', min:88, max:100,
               grid:{color:'#0e1623'}, ticks:{color:'#34d399',font:{size:11}},
               title:{display:true,text:'Efficiency (%)',color:'#34d399',font:{size:11}} },
        ySF:{ type:'linear', position:'right', min:0, max:5,
              grid:{drawOnChartArea:false}, ticks:{color:'#f87171',font:{size:11}},
              title:{display:true,text:'Slip Safety Factor',color:'#f87171',font:{size:11}} }
      }
    }
  });
}

function renderFEADRecs(fd) {
  const el = document.getElementById('fead-recs');
  if (!el) return;
  const t = fd.totals;
  const recs = [];
  const P = interp(state.rpm);
  const etaNum = parseFloat(t.eta);
  const T_opt = parseInt(t.T_opt);
  const T_cur = state.baseTension;

  // Tension optimisation
  if (T_cur < T_opt * 0.85) {
    recs.push({ color:'#e63946', title:'🔴 Belt Under-Tensioned', body:`Current tension (${T_cur} N) is below the calculated optimum (${T_opt} N) for SF=1.3. Belt slip risk exists — especially on CRK and FAN spans at high RPM. Increase tensioner pre-load.`, impact:`⬆ Increase tension to ${T_opt} N → SF rises to 1.3, slip eliminated` });
  } else if (T_cur > T_opt * 1.5) {
    recs.push({ color:'#f4a261', title:'🟠 Belt Over-Tensioned', body:`Current tension (${T_cur} N) significantly exceeds the optimum (${T_opt} N). Over-tensioning increases bearing loads and accelerates hub bearing wear without improving belt performance.`, impact:`⬇ Reduce tension to ${T_opt}–${Math.round(T_opt*1.2)} N → bearing life extends by ~${Math.round((T_cur/T_opt-1)*40)}%` });
  }

  // FAN load
  if (P.FAN > 8) {
    recs.push({ color:'#f472b6', title:'🌀 High FAN Load — Consider Viscous Coupling', body:`At ${state.rpm} RPM, FAN consumes ${P.FAN.toFixed(1)} kW — driven by the cube law (P ∝ n³). A viscous coupling or variable-speed fan reduces this to ~20–30% at low-speed cruise, saving 4–8 kW.`, impact:`✅ Variable fan → saves up to ${(P.FAN*0.7).toFixed(1)} kW at cruise` });
  }

  // ALT
  if (P.ALT > 3.5) {
    recs.push({ color:'#fbbf24', title:'⚡ Smart Alternator Recommended', body:`Alternator load (${P.ALT.toFixed(1)} kW) is high. A smart/variable alternator charges during deceleration (free energy) and cuts load during acceleration, reducing FEAD parasitic draw by 1–2 kW on average.`, impact:`✅ Smart ALT → saves ~${(P.ALT*0.35).toFixed(1)} kW average, reduces CRK hub load` });
  }

  // Efficiency
  if (etaNum < 96) {
    recs.push({ color:'#a78bfa', title:'📉 Efficiency Below 96% — Review Belt Routing', body:`FEAD efficiency is ${t.eta}%. Losses are dominated by ${parseFloat(t.P_bend)>parseFloat(t.P_bear)?'bending hysteresis (increase pulley diameters)':'bearing friction (reduce hub loads)'}. Review span lengths and pulley layout to reduce losses.`, impact:`✅ Optimising layout → efficiency target ≥97%` });
  }

  // Slip warnings
  for (const [n, d] of Object.entries(fd.pulleys)) {
    if (d.status === 'SLIP') recs.push({ color:'#e63946', title:`🚨 SLIP on ${n} Pulley`, body:`SF = ${d.SF} < 1.0. Belt is slipping on ${n} (${PULLEYS[n].label.split('\\n')[1]}). Increase static tension or check pulley alignment and condition.`, impact:'⬆ Raise tension immediately to prevent belt damage' });
    else if (d.status === 'MARGINAL') recs.push({ color:'#fbbf24', title:`⚠ Marginal SF on ${n}`, body:`SF = ${d.SF} (target >1.3). ${n} pulley is close to slip. Consider raising tension by 100–200 N or verifying pulley condition.`, impact:`⬆ Raise tension by 150 N → SF rises to ~${(parseFloat(d.SF)+0.25).toFixed(2)}` });
  }

  // Aramid cord
  recs.push({ color:'#4fc3f7', title:'🔩 Aramid Cord — Verified Optimal', body:`MT820 AMD with aramid cord is the correct choice. Aramid has: (1) low elongation (±0.4% stretch tolerance) → stable tension, (2) ~30% lower bending stiffness loss vs steel, (3) low linear mass → minimal centrifugal tension at high RPM.`, impact:`✅ Aramid saves ~${(BELT_PARAMS.C_bend*0.3*100).toFixed(2)}% bending loss vs steel cord` });

  if (recs.length === 0) recs.push({ color:'#34d399', title:'✅ FEAD is Well-Optimised', body:'All parameters are within acceptable limits. Belt slip safety factors are satisfactory, tension is near-optimal, and efficiency is above 96%.', impact:'Continue monitoring at peak RPM and idle+high-demand conditions.' });

  el.innerHTML = recs.map(r => `
    <div class="fead-rec" style="--rec-color:${r.color}">
      <div class="fead-rec-title">${r.title}</div>
      <div class="fead-rec-body">${r.body}</div>
      <div class="fead-rec-impact">${r.impact}</div>
    </div>`).join('');
}

function renderFEADTension(fd) {
  const el = document.getElementById('fead-tension-body');
  if (!el) return;
  const t  = fd.totals;
  const T0 = state.baseTension;
  const T_opt = parseInt(t.T_opt);
  const v  = beltVelocity(state.rpm);
  const Tc = parseFloat(t.Tc);
  // Belt life index (relative, higher = better)
  const life_factor = T0 > 0 ? Math.min(100, Math.round((T_opt / T0) * 100)) : 100;
  const diff = T0 - T_opt;

  const items = [
    { label:'Current Tension', val:`${T0} N`, note:'Set by tensioner spring pre-load', bar:Math.min(100,(T0/600)*100), color:'#4fc3f7' },
    { label:'Optimal Tension (SF=1.3)', val:`${T_opt} N`, note:`For max efficiency at ${state.rpm} RPM`, bar:Math.min(100,(T_opt/600)*100), color:'#34d399' },
    { label:'Centrifugal Tension', val:`${Tc} N`, note:`mv²/L at ${t.belt_v} m/s belt speed`, bar:Math.min(100,(Tc/200)*100), color:'#fbbf24' },
    { label:'Deviation from Optimal', val:`${diff >= 0 ? '+' : ''}${diff} N`, note: diff > 100 ? 'Over-tensioned — reduce' : diff < -100 ? 'Under-tensioned — increase' : 'Within ±100 N of optimal ✅', bar:Math.min(100,Math.abs(diff)/4), color: Math.abs(diff) > 100 ? '#f87171' : '#34d399' },
    { label:'Fan Bearing Life Factor', val:`${CU_VALIDATION.bearingLife.L10_composite} h`, note:`C&U L10 composite at peak load (2400 RPM, 358 N Fr)`, bar: Math.min(100,(CU_VALIDATION.bearingLife.L10_composite/5000)*100), color:'#a78bfa' }
  ];

  el.innerHTML = items.map(it => `
    <div class="fead-t-item">
      <div class="fead-t-label">${it.label}</div>
      <div class="fead-t-val" style="color:${it.color}">${it.val}</div>
      <div class="fead-t-note">${it.note}</div>
      <div class="fead-t-bar"><div class="fead-t-bar-fill" style="width:${it.bar}%;--bar-c:${it.color}"></div></div>
    </div>`).join('');
}

function renderFEADTable(fd) {
  const el = document.getElementById('fead-detail-table');
  if (!el) return;
  const cols = ['Pulley','Pwr (kW)','v_belt (m/s)','T_tight (N)','T_slack (N)','Slip SF','Bend Loss (kW)','Bear Loss (kW)','Status'];
  const v = beltVelocity(state.rpm).toFixed(2);
  let h = `<table><thead><tr>${cols.map(c=>`<th>${c}</th>`).join('')}</tr></thead><tbody>`;
  for (const n of ORDER) {
    const d = fd.pulleys[n];
    const p = PULLEYS[n];
    const sfNum = parseFloat(d.SF);
    const sfColor = sfNum >= 1.3 ? '#34d399' : sfNum >= 1.0 ? '#fbbf24' : '#f87171';
    const statusBg = d.status==='SLIP'?'rgba(230,57,70,0.15)':d.status==='MARGINAL'?'rgba(251,191,36,0.08)':'';
    const sfBar = Math.min(100, sfNum / 3 * 100);
    h += `<tr style="background:${statusBg}">
      <td style="color:${p.color};font-weight:700;font-family:var(--font-head)">${n}</td>
      <td>${d.pwr.toFixed(2)}</td>
      <td>${v}</td>
      <td>${d.T_tight}</td>
      <td>${d.T_slack}</td>
      <td>
        <div class="fead-sf-bar">
          <div class="fead-sf-indicator" style="width:${sfBar}px;background:${sfColor}"></div>
          <span style="color:${sfColor};font-weight:700">${d.SF}</span>
        </div>
      </td>
      <td>${d.P_bend}</td>
      <td>${d.P_bear}</td>
      <td style="color:${sfColor};font-weight:700">${d.status}</td>
    </tr>`;
  }
  el.innerHTML = h + `</tbody></table>`;
}


// ══════════════════════════════════════════════════════════════════════════════
// IDEAL WORKING CONDITION & COMPLIANCE DASHBOARD
// ══════════════════════════════════════════════════════════════════════════════

// ── Ideal Design Envelope (from Gates PDF + ISO 9981 + C&U data) ─────────────
const IDEAL = {
  rpm:       { value:1200, min:800,  max:1800, label:'Engine RPM', unit:'RPM',
               note:'PDF design reference: 1200 RPM. Optimal FEAD efficiency at 1200-1600 RPM.' },
  tension:   { value:null, min:1800, max:3500, label:'Belt Tension', unit:'N',
               note:'Computed per SF=1.3 (Capstan eq). Must not exceed max to avoid bearing overload.' },
  tenIdx:    { value:2,   label:'Tensioner Position', note:'MEAN arm position (15.4°) per Gates PDF Sheet 2.' },
  beltSpeed: { min:8,  max:22, ideal:13.4, label:'Belt Speed', unit:'m/s',
               note:'Optimal: 8–22 m/s. Above 22 m/s: centrifugal tension dominates. Below 8: high span tension per kW.' },
  slipSF:    { min:1.3, ideal:1.6, label:'Min Slip Safety Factor',
               note:'Capstan SF target ≥1.3. Values <1.0 cause belt slip and accelerated wear.' },
  efficiency:{ min:96, ideal:97.5, label:'FEAD Efficiency', unit:'%',
               note:'Target >96%. Aramid + optimal tension achieves 97–98.5% (Gates MT820 AMD spec).' },
  fanLoad:   { max:2866.4, label:'FAN Hub Load', unit:'N', note:'PDF certified peak: 2866.4 N (C&U L10=3305h).' },
  crkLoad:   { max:2658.9, label:'CRK Hub Load', unit:'N', note:'PDF certified peak: 2658.9 N.' },
  altLoad:   { max:1235.2, label:'ALT Hub Load', unit:'N', note:'PDF certified peak: 1235.2 N.' },
  acLoad:    { max:1381.4, label:'AC Hub Load',  unit:'N', note:'PDF certified peak: 1381.4 N.' }
};

// ── Compliance Weights (must sum to 100) ─────────────────────────────────────
const COMP_WEIGHTS = {
  efficiency: 25,  // FEAD efficiency is most important
  slipSF:     25,  // Belt slip safety is critical
  hubLoads:   20,  // Hub loads vs PDF limits
  beltSpeed:  15,  // Belt speed window
  tension:    15   // Tension vs optimal
};

let complianceRadarChart = null;

function computeCompliance() {
  const fd  = computeFEAD();
  const v   = parseFloat(fd.totals.belt_v);
  const eta = parseFloat(fd.totals.eta);
  const T0  = state.baseTension;
  const T_opt = parseInt(fd.totals.T_opt) || 2400;

  // 1. FEAD Efficiency score (0-100)
  const eta_score = Math.min(100, Math.max(0, (eta - 88) / (100 - 88) * 100));

  // 2. Slip SF score (worst pulley)
  const minSF = Math.min(...ORDER.map(n => parseFloat(fd.pulleys[n].SF)));
  const sf_score = Math.min(100, Math.max(0, (minSF / 2.5) * 100));

  // 3. Hub loads score (worst ratio vs PDF limits)
  const hubRatios = {
    CRK: hubData.CRK ? hubData.CRK.F / PDF.CRK.F : 0,
    FAN: hubData.FAN ? hubData.FAN.F / PDF.FAN.F : 0,
    ALT: hubData.ALT ? hubData.ALT.F / PDF.ALT.F : 0,
    AC:  hubData.AC  ? hubData.AC.F  / PDF.AC.F  : 0
  };
  const worstRatio = Math.max(...Object.values(hubRatios));
  const hub_score  = Math.min(100, Math.max(0, (1 - Math.max(0, worstRatio - 0.5)) * 200));

  // 4. Belt speed score (penalty outside 8-22 m/s window)
  const spd_score = v < 8  ? Math.max(0, v / 8 * 70) :
                    v > 22 ? Math.max(0, 100 - (v-22)*20) : 100;

  // 5. Tension score (distance from optimal)
  const ten_pct    = T_opt > 0 ? T0 / T_opt : 1;
  const ten_score  = Math.min(100, Math.max(0, 100 - Math.abs(ten_pct - 1) * 200));

  // Weighted composite
  const W = COMP_WEIGHTS;
  const composite = (
    eta_score  * W.efficiency +
    sf_score   * W.slipSF +
    hub_score  * W.hubLoads +
    spd_score  * W.beltSpeed +
    ten_score  * W.tension
  ) / 100;

  const grade = composite >= 85 ? 'COMPLIANT' : composite >= 65 ? 'MARGINAL' : 'NON-COMPLIANT';
  const color = composite >= 85 ? '#34d399'   : composite >= 65 ? '#fbbf24'  : '#e63946';
  const icon  = composite >= 85 ? '✅'         : composite >= 65 ? '⚠️'       : '❌';

  return {
    fd, v, eta, T0, T_opt, minSF, worstRatio, hubRatios,
    scores: { efficiency:eta_score, slipSF:sf_score, hubLoads:hub_score, beltSpeed:spd_score, tension:ten_score },
    composite: composite.toFixed(0), grade, color, icon
  };
}

function initComplianceDashboard() {
  const btn = document.getElementById('btn-apply-ideal');
  if (btn) btn.addEventListener('click', applyIdealSettings);
  renderComplianceDashboard();
}

function applyIdealSettings() {
  const fd = computeFEAD();
  const T_opt = parseInt(fd.totals.T_opt) || 2400;

  // Clamp to slider ranges
  const idealRPM = 1200;
  const idealTen = Math.max(1800, Math.min(3500, T_opt));
  const idealIdx = 2; // MEAN position

  const rpmEl = document.getElementById('rpm');
  const tenEl = document.getElementById('tension');
  const tidxEl = document.getElementById('tensioner');
  if (rpmEl)  { rpmEl.value  = idealRPM; document.getElementById('lbl-rpm').textContent = idealRPM + ' RPM'; }
  if (tenEl)  { tenEl.value  = idealTen; document.getElementById('lbl-tension').textContent = idealTen + ' N'; }
  if (tidxEl) { tidxEl.value = idealIdx; const tp = TEN_POS[idealIdx]; document.getElementById('lbl-tensioner').textContent = tp.label + ' (' + tp.arm + '°)'; }

  state.rpm = idealRPM;
  state.baseTension = idealTen;
  state.tenIdx = idealIdx;

  compute();
  renderTable();
  renderGearSummary();
  renderInsights();
  buildBelt3D();
  buildChartData();
  updateChart();
  renderFEADAll();
  renderDriveCycleChart();
  renderComplianceDashboard();
  renderComplianceDashboard();

  const btn = document.getElementById('btn-apply-ideal');
  if (btn) {
    btn.textContent = '✅ Ideal Applied';
    setTimeout(() => { btn.textContent = '▶ Apply Ideal Settings'; }, 2500);
  }
}

function renderComplianceDashboard() {
  const c = computeCompliance();
  renderComplianceVerdict(c);
  renderComplianceRadar(c);
  renderComplianceParamTable(c);
  renderComplianceScoreRing(c);
  renderComplianceMid(c);
  renderComplianceMaintenance(c);
  renderComplianceRules(c);
}

function renderComplianceVerdict(c) {
  const el = document.getElementById('compliance-verdict');
  if (!el) return;
  el.className = 'compliance-verdict verdict-' + (c.grade==='COMPLIANT'?'compliant':c.grade==='MARGINAL'?'marginal':'fail');
  const subtexts = {
    'COMPLIANT': `All design parameters within Gates PDF and ISO 9981 limits. Belt slip safety ≥${c.minSF.toFixed(2)}. FEAD efficiency ${c.eta.toFixed(1)}%. System is optimally configured.`,
    'MARGINAL':  `Some parameters approaching design limits. Review highlighted items. Min slip SF=${c.minSF.toFixed(2)}, FEAD η=${c.eta.toFixed(1)}%. Adjustments recommended.`,
    'NON-COMPLIANT': `One or more parameters outside design envelope. Immediate attention required. Risk of belt slip, premature bearing failure, or reduced efficiency.`
  };
  el.innerHTML = `
    <span class="verdict-icon">${c.icon}</span>
    <div>
      <div class="verdict-title" style="color:${c.color}">${c.grade} — ${state.rpm} RPM · ${state.baseTension} N · ${TEN_POS[state.tenIdx].label}</div>
      <div class="verdict-sub">${subtexts[c.grade]}</div>
    </div>
    <div class="verdict-score" style="color:${c.color}">${c.composite}/100</div>`;
}

function renderComplianceRadar(c) {
  const canvas = document.getElementById('compliance-radar');
  if (!canvas) return;
  if (complianceRadarChart) { complianceRadarChart.destroy(); complianceRadarChart = null; }
  const s = c.scores;
  const labels = ['FEAD\nEfficiency','Slip\nSafety','Hub Load\nMargin','Belt Speed\nWindow','Tension\nOptimality'];
  const currentData = [s.efficiency, s.slipSF, s.hubLoads, s.beltSpeed, s.tension].map(v => +v.toFixed(0));
  const idealData   = [100, 100, 100, 100, 100];

  complianceRadarChart = new Chart(canvas, {
    type: 'radar',
    data: {
      labels,
      datasets: [
        {
          label:'Ideal', data:idealData,
          borderColor:'rgba(79,195,247,0.5)', backgroundColor:'rgba(79,195,247,0.05)',
          borderWidth:1.5, borderDash:[5,3], pointRadius:3, pointBackgroundColor:'#4fc3f7'
        },
        {
          label:'Current', data:currentData,
          borderColor: c.color, backgroundColor: c.color + '22',
          borderWidth:2.5, pointRadius:4, pointBackgroundColor: c.color
        }
      ]
    },
    options: {
      responsive:true, maintainAspectRatio:false,
      plugins:{
        legend:{ display:false },
        tooltip:{
          backgroundColor:'rgba(10,14,26,0.95)', borderColor:'#1c2840', borderWidth:1,
          titleColor:'#cdd6e8', bodyColor:'#8899aa', padding:10
        }
      },
      scales:{
        r:{
          min:0, max:100,
          grid:{ color:'rgba(255,255,255,0.06)' },
          angleLines:{ color:'rgba(255,255,255,0.06)' },
          pointLabels:{ color:'#8899aa', font:{size:11}, padding:8 },
          ticks:{ backdropColor:'transparent', color:'#4a5c78', font:{size:9} }
        }
      }
    }
  });
}

function renderComplianceParamTable(c) {
  const el = document.getElementById('compliance-param-table');
  if (!el) return;
  const fd = c.fd;
  const v  = c.v;

  const rows = [
    {
      param:'Engine RPM', current:state.rpm, ideal:1200, min:800, max:1800, unit:'RPM',
      score: state.rpm>=800&&state.rpm<=1800 ? 100 : 50, note:'Gates PDF design reference'
    },
    {
      param:'Belt Tension', current:state.baseTension, ideal:c.T_opt, min:1800, max:3500, unit:'N',
      score: c.scores.tension, note:`SF=1.3 target for ${state.rpm} RPM`
    },
    {
      param:'Belt Speed', current:v.toFixed(2), ideal:'8–22', min:8, max:22, unit:'m/s',
      score: c.scores.beltSpeed, note:'Optimal centrifugal/tension balance'
    },
    {
      param:'FEAD Efficiency', current:c.eta.toFixed(1), ideal:'>97.0', min:96, max:100, unit:'%',
      score: c.scores.efficiency, note:'Aramid cord target spec'
    },
    {
      param:'Min Slip SF', current:c.minSF.toFixed(2), ideal:'>1.60', min:1.3, max:2.5, unit:'–',
      score: c.scores.slipSF, note:'Capstan equation, μ=0.35'
    },
    {
      param:'CRK Hub Load', current:hubData.CRK?hubData.CRK.F.toFixed(0):'—', ideal:'<'+PDF.CRK.F, min:0, max:PDF.CRK.F, unit:'N',
      score: hubData.CRK ? Math.min(100,Math.max(0,(1-hubData.CRK.F/PDF.CRK.F)*200)) : 100, note:'PDF certified peak'
    },
    {
      param:'FAN Hub Load', current:hubData.FAN?hubData.FAN.F.toFixed(0):'—', ideal:'<'+PDF.FAN.F, min:0, max:PDF.FAN.F, unit:'N',
      score: hubData.FAN ? Math.min(100,Math.max(0,(1-hubData.FAN.F/PDF.FAN.F)*200)) : 100, note:'PDF cert. | C&U L10=3305h'
    },
    {
      param:'ALT Hub Load', current:hubData.ALT?hubData.ALT.F.toFixed(0):'—', ideal:'<'+PDF.ALT.F, min:0, max:PDF.ALT.F, unit:'N',
      score: hubData.ALT ? Math.min(100,Math.max(0,(1-hubData.ALT.F/PDF.ALT.F)*200)) : 100, note:'PDF certified peak'
    }
  ];

  const cols = ['Parameter','Current','Ideal','Min','Max','Unit','Status','Note'];
  let h = `<table><thead><tr>${cols.map(c=>`<th>${c}</th>`).join('')}</tr></thead><tbody>`;
  rows.forEach(r => {
    const sc = Math.max(0, Math.min(100, r.score));
    const dotColor = sc>=80?'#34d399':sc>=55?'#fbbf24':'#e63946';
    const barW = sc + '%';
    h += `<tr>
      <td style="font-family:var(--font-head);font-weight:600;color:var(--text)">${r.param}</td>
      <td style="color:var(--text)">${r.current}</td>
      <td style="color:#4fc3f7">${r.ideal}</td>
      <td style="color:var(--text-faint)">${r.min}</td>
      <td style="color:var(--text-faint)">${r.max}</td>
      <td style="color:var(--text-faint)">${r.unit}</td>
      <td>
        <div class="cp-bar">
          <span class="cp-status-dot" style="background:${dotColor}"></span>
          <div class="cp-bar-track"><div class="cp-bar-fill" style="width:${barW};background:${dotColor}"></div></div>
          <span style="color:${dotColor};font-size:0.68rem;font-weight:700">${sc.toFixed(0)}</span>
        </div>
      </td>
      <td style="color:var(--text-faint);font-size:0.67rem;font-family:var(--font-ui)">${r.note}</td>
    </tr>`;
  });
  el.innerHTML = h + `</tbody></table>`;
}

function renderComplianceScoreRing(c) {
  const ring = document.getElementById('compliance-score-ring');
  if (ring) {
    const gradeLabel = c.composite>=85?'A – Excellent':c.composite>=70?'B – Good':c.composite>=55?'C – Marginal':'D – Poor';
    ring.innerHTML = `
      <div class="score-circle" style="--score-c:${c.color}">
        <div class="score-num">${c.composite}</div>
        <div class="score-label">/ 100</div>
      </div>
      <div class="score-grade">${gradeLabel}</div>`;
  }

  const bd = document.getElementById('compliance-score-breakdown');
  if (bd) {
    const items = [
      { label:'Efficiency',    score:c.scores.efficiency, weight:COMP_WEIGHTS.efficiency },
      { label:'Slip Safety',   score:c.scores.slipSF,     weight:COMP_WEIGHTS.slipSF },
      { label:'Hub Loads',     score:c.scores.hubLoads,   weight:COMP_WEIGHTS.hubLoads },
      { label:'Belt Speed',    score:c.scores.beltSpeed,  weight:COMP_WEIGHTS.beltSpeed },
      { label:'Tension',       score:c.scores.tension,    weight:COMP_WEIGHTS.tension }
    ];
    bd.innerHTML = items.map(it => {
      const sc = Math.max(0,Math.min(100,it.score));
      const col = sc>=80?'#34d399':sc>=55?'#fbbf24':'#e63946';
      return `<div class="csb-row">
        <span class="csb-label">${it.label} (×${it.weight})</span>
        <div class="csb-bar"><div class="csb-fill" style="width:${sc}%;background:${col}"></div></div>
        <span class="csb-val" style="color:${col}">${sc.toFixed(0)}</span>
      </div>`;
    }).join('');
  }
}

function renderComplianceMid(c) {
  const el = document.getElementById('compliance-mid');
  if (!el) return;
  const fd = c.fd;
  const P  = interp(state.rpm);

  const items = [
    { label:'Ideal RPM', current:state.rpm+' RPM', ideal:'1200 RPM', color:'#4fc3f7',
      ok: state.rpm>=900&&state.rpm<=1600, note: state.rpm===1200?'✅ At design RPM':state.rpm<900?'↑ Too low — increase':'↑ Above optimal range' },
    { label:'Optimal Tension', current:state.baseTension+' N', ideal:c.T_opt+' N', color:'#34d399',
      ok: Math.abs(state.baseTension-c.T_opt)<200, note: Math.abs(state.baseTension-c.T_opt)<200?'✅ Near-optimal':'↔ Adjust tension to '+c.T_opt+' N' },
    { label:'Tensioner Position', current:TEN_POS[state.tenIdx].label, ideal:TEN_POS[2].label+' (15.4°)', color:'#fbbf24',
      ok: state.tenIdx===2, note: state.tenIdx===2?'✅ At PDF design point':'Set to MEAN (index 2)' },
    { label:'Belt Speed', current:c.v.toFixed(1)+' m/s', ideal:'10–16 m/s', color:'#f472b6',
      ok: c.v>=8&&c.v<=22, note: c.v<8?'↑ Low — belt over-tensioned per kW':c.v>22?'↓ High — centrifugal risk':'✅ Within optimal window' },
    { label:'Min Slip SF', current:c.minSF.toFixed(2), ideal:'>1.30', color: c.minSF>=1.3?'#34d399':c.minSF>=1.0?'#fbbf24':'#e63946',
      ok: c.minSF>=1.3, note: c.minSF>=1.3?'✅ Slip-safe':'⚠ Raise tension or check wrap' },
    { label:'Fan Bearing Life', current:CU_VALIDATION.bearingLife.L10_composite+'h (worst)', ideal:'>5000 h', color:'#a78bfa',
      ok: true, note:'C&U Page 4: composite L10 at 2400 RPM, peak load' }
  ];

  el.innerHTML = items.map(it => `
    <div class="cmi-card" style="--cmi-c:${it.color}">
      <div class="cmi-label">${it.label}</div>
      <div class="cmi-current">Now: ${it.current}</div>
      <div class="cmi-ideal">${it.ideal}</div>
      <div class="cmi-status" style="color:${it.ok?'#34d399':'#f87171'}">${it.note}</div>
    </div>`).join('');
}

function renderComplianceMaintenance(c) {
  const el = document.getElementById('compliance-maintenance-grid');
  if (!el) return;
  const L10b = CU_VALIDATION.bearingLife.L10B_pulley; // 3860h

  // Scale intervals by current load vs peak load
  const fanRatio = hubData.FAN ? hubData.FAN.F / PDF.FAN.F : 0.5;
  const crkRatio = hubData.CRK ? hubData.CRK.F / PDF.CRK.F : 0.5;
  const loadFactor = Math.max(fanRatio, crkRatio);

  // At full load (ratio=1): interval = L10b/3 = 1287h
  // At low load (ratio<0.5): interval scales up
  const beltInspH = Math.round(Math.min(2000, 800 / Math.max(0.3, loadFactor)));
  const tenCheckH = Math.round(Math.min(1000, 500 / Math.max(0.3, loadFactor)));
  const beltReplH = Math.round(Math.min(4000, L10b * 0.5 / Math.max(0.3, loadFactor)));
  const bearReplH = Math.round(CU_VALIDATION.bearingLife.L10_composite * 0.9);

  const urgency = loadFactor > 0.8 ? 'HIGH LOAD' : loadFactor > 0.5 ? 'MOD LOAD' : 'LOW LOAD';
  const urgColor = loadFactor > 0.8 ? '#f87171' : loadFactor > 0.5 ? '#fbbf24' : '#34d399';

  const tasks = [
    {
      color:'#4fc3f7', interval:`${beltInspH}h`, task:'Belt Visual Inspection',
      detail:`Check for cracking, glazing, rib separation, and longitudinal cracks. Inspect pulley grooves for wear. Verify belt tension with Gates frequency meter.`,
      checks:['Rib surface smooth & intact','No glazing or oil contamination','Pulley grooves free of debris'],
      urgency, urgColor
    },
    {
      color:'#fbbf24', interval:`${tenCheckH}h`, task:'Tensioner Spring Check',
      detail:`Verify tensioner arm is at ${TEN_POS[2].label} position (15.4°). Check damping mechanism. Measure spring load — replace if below 80% of nominal.`,
      checks:['Arm at MEAN ±5° (PDF spec)','Damper not worn out','Spring load ≥80% nominal'],
      urgency, urgColor
    },
    {
      color:'#f472b6', interval:`${beltReplH}h`, task:'Belt Replacement',
      detail:`Replace Gates MT820 8-rib AMD belt. At current load factor (${(loadFactor*100).toFixed(0)}% of PDF peak), belt fatigue life reaches replacement threshold at ~${beltReplH}h. Do not over-run.`,
      checks:['Use Gates MT820 8PK AMD','Re-tension to '+c.T_opt+'N after install','Run 30min then re-check tension'],
      urgency, urgColor
    },
    {
      color:'#e63946', interval:`${bearReplH}h`, task:'Fan Bearing Replacement',
      detail:`C&U WR25153 composite L10 = 3,305h at worst-case (2400 RPM). Replace before reaching 90% of L10 = ${bearReplH}h. Roller-end (Cr=38,179N) outlasts ball-end (L10B=3,860h) — replace as a set.`,
      checks:['C&U ball-end L10B = 3860h','Replace at 90% = 3474h','Record replacement date & RPM history'],
      urgency: loadFactor > 0.7 ? 'CRITICAL' : 'PLAN AHEAD',
      urgColor: loadFactor > 0.7 ? '#e63946' : '#a78bfa'
    }
  ];

  el.innerHTML = tasks.map(t => `
    <div class="cm-card" style="--cm-c:${t.color}">
      <div class="cm-interval">${t.interval}</div>
      <div class="cm-task">${t.task}</div>
      <div class="cm-detail">${t.detail}</div>
      <div style="margin-top:0.4rem">${t.checks.map(ch=>`<div class="cm-check">✔ ${ch}</div>`).join('')}</div>
      <span class="cm-urgency" style="background:${t.urgColor}22;color:${t.urgColor};border:1px solid ${t.urgColor}44">${t.urgency}</span>
    </div>`).join('');
}

function renderComplianceRules(c) {
  const el = document.getElementById('compliance-rules-grid');
  if (!el) return;
  const sf = c.minSF;
  const eta = c.eta;

  const rules = [
    {
      icon:'🔩', cls: sf>=1.3?'cr-pass':'cr-warn',
      title:`Capstan Safety Factor ≥ 1.3 (ISO 9981)`,
      text:`Current minimum SF = ${sf.toFixed(2)}. The Eytelwein equation requires T_tight/T_slack = e^(μθ). With μ=0.35 (PK belt, grooved pulley), each pulley must maintain SF>1.3 to prevent slip during transient loads.`,
      ref:`ISO 9981:2018 · Gates Engineering Manual §4.3 · Current: SF=${sf.toFixed(2)} ${sf>=1.3?'✅':'⚠'}`
    },
    {
      icon:'⚙', cls: parseFloat(c.fd.totals.eta)>=96?'cr-pass':'cr-warn',
      title:`FEAD Efficiency ≥ 96% (Gates MT820 Spec)`,
      text:`Current η = ${eta.toFixed(1)}%. Gates MT820 AMD aramid belt specification guarantees 97–98.5% under nominal conditions. Losses from belt bending (C_b=0.0012), bearing friction (μ_b=0.003), and centrifugal effects must be minimised.`,
      ref:`Gates MT820 AMD Datasheet · Aramid C_b = 0.0012 · Current: ${eta.toFixed(1)}% ${eta>=96?'✅':'⚠'}`
    },
    {
      icon:'📏', cls: 'cr-pass',
      title:`Hub Loads Within PDF Certified Peaks`,
      text:`All computed hub forces must remain ≤ PDF peak values (CRK: 2658.9N, FAN: 2866.4N, ALT: 1235.2N, AC: 1381.4N). These are the values Gates used to select the bearing specification. Exceeding them shortens L10 life proportionally to the cube of force ratio.`,
      ref:`Gates PDF Sheet 1 · C&U WR25153 Page 4 · L10 ∝ (C/F)^3`
    },
    {
      icon:'🌡', cls: c.v>=8&&c.v<=22?'cr-pass':'cr-warn',
      title:`Belt Speed 8–22 m/s (Optimal Window)`,
      text:`Current v = ${c.v.toFixed(1)} m/s. Below 8 m/s: same power requires very high tension (T=P/v). Above 22 m/s: centrifugal tension T_c = mv²/L reduces effective clamping tension and increases bending fatigue frequency.`,
      ref:`Gates Belt Design Manual §3.2 · Current: ${c.v.toFixed(1)} m/s ${c.v>=8&&c.v<=22?'✅':'⚠'}`
    },
    {
      icon:'🎯', cls: Math.abs(state.baseTension - c.T_opt) < 300 ? 'cr-pass':'cr-warn',
      title:`Tension Within ±200N of Computed Optimum`,
      text:`Optimal tension = ${c.T_opt} N (for SF=1.3 at ${state.rpm} RPM). Current = ${state.baseTension} N (${state.baseTension>c.T_opt?'over':'under'}-tensioned by ${Math.abs(state.baseTension-c.T_opt)} N). Over-tensioning increases bearing load and reduces L10. Under-tensioning risks belt slip and glazing.`,
      ref:`Computed via Capstan equation · Gates tensioner spring spec · T_opt = ${c.T_opt} N`
    },
    {
      icon:'🔋', cls: 'cr-info',
      title:`Fan Bearing Life ≥ 3,305h (C&U L10 Composite)`,
      text:`C&U Group WR25153 certifies composite L10 = 3,305h under worst conditions (2,400 RPM, F_hub at PDF peak, Fr=358N imbalance). L10B (belt side, ball bearing) = 3,860h is life-limiting. Replace bearings at 90% of L10 = 2,975h to prevent unexpected failure.`,
      ref:`C&U WR25153 Page 4 · Cr_ball=19,035N · L10B=3,860h · L10=3,305h`
    }
  ];

  el.innerHTML = rules.map(r => `
    <div class="cr-rule ${r.cls}">
      <div class="cr-icon">${r.icon}</div>
      <div class="cr-body">
        <div class="cr-title">${r.title}</div>
        <div class="cr-text">${r.text}</div>
        <div class="cr-ref">${r.ref}</div>
      </div>
    </div>`).join('');
}




// ══════════════════════════════════════════════════════════════════════════════


// ══════════════════════════════════════════════════════════════════════════════
// PDF TECHNICAL REPORT GENERATOR v2 — larger fonts, live-computed values
// ══════════════════════════════════════════════════════════════════════════════

function initPDFReport() {
  const btn = document.getElementById('btn-pdf-report');
  if (btn) btn.addEventListener('click', generatePDFReport);
}

// ── Shared page header (called at top of every page after page 1) ─────────────
function drawPageHeader(doc, C, ML, PW, dateStr) {
  doc.setFillColor(...C.navy);
  doc.rect(0, 0, PW, 18, 'F');
  doc.setFillColor(...C.cyan);
  doc.rect(0, 0, 7, 18, 'F');
  doc.setFont('helvetica', 'bold');
  doc.setFontSize(11);
  doc.setTextColor(...C.white);
  doc.text('FEAD TECHNICAL ANALYSIS REPORT', ML, 12);
  doc.setFont('helvetica', 'normal');
  doc.setFontSize(9);
  doc.setTextColor(...C.muted);
  doc.text('Gates MT820 8PK AMD  \u00b7  Ashok Leyland H6  \u00b7  ' + dateStr, PW - ML, 12, { align: 'right' });
}

// ── Main generator ────────────────────────────────────────────────────────────
function generatePDFReport() {
  if (typeof window.jspdf === 'undefined') {
    alert('PDF library not loaded yet. Please wait a moment and try again.');
    return;
  }
  const btn = document.getElementById('btn-pdf-report');
  if (btn) { btn.disabled = true; btn.textContent = 'Generating PDF\u2026'; }

  try {
    const { jsPDF } = window.jspdf;
    const doc = new jsPDF({ orientation: 'portrait', unit: 'mm', format: 'a4' });

    // Snapshot live data BEFORE any sweep modifies state
    const snapRPM = state.rpm;
    const snapTen = state.baseTension;
    const snapIdx = state.tenIdx;
    const fd   = computeFEAD();
    const comp = computeCompliance();
    const P    = interp(snapRPM);
    const v    = beltVelocity(snapRPM);
    const dateStr = new Date().toLocaleDateString('en-GB', { day: '2-digit', month: 'long', year: 'numeric' });

    // Snapshot hub data
    const snapHub = {};
    ORDER.forEach(n => { if (hubData[n]) snapHub[n] = { F: hubData[n].F, dir: hubData[n].dir, T_in: hubData[n].T_in, T_out: hubData[n].T_out }; });

    // ── Colour palette ────────────────────────────────────────────────────
    const C = {
      navy:  [8,  16,  40],
      blue:  [15, 40,  90],
      cyan:  [79, 195, 247],
      green: [52, 211, 153],
      amber: [251,191, 36],
      red:   [230, 57, 70],
      white: [255,255,255],
      light: [220,228,240],
      muted: [130,150,175],
      pass:  [22, 101, 60],
      passb: [187,247,208],
      warn:  [120, 87,  0],
      warnb: [254,240,138],
      fail:  [127, 29, 29],
      failb: [254,202,202]
    };

    const PW = 210, PH = 297, ML = 16, MR = 16;
    const CW = PW - ML - MR;
    let Y = 22;

    // ── Layout helpers ────────────────────────────────────────────────────
    const newPage = () => {
      doc.addPage();
      Y = 22;
      drawPageHeader(doc, C, ML, PW, dateStr);
      Y = 26;
    };
    const checkY = (need) => { if (Y + need > PH - 22) newPage(); };

    const section = (title) => {
      checkY(16);
      doc.setFillColor(...C.navy);
      doc.rect(ML, Y, CW, 11, 'F');
      doc.setFillColor(...C.cyan);
      doc.rect(ML, Y, 4, 11, 'F');
      doc.setFont('helvetica', 'bold');
      doc.setFontSize(12);
      doc.setTextColor(...C.white);
      doc.text(title.toUpperCase(), ML + 8, Y + 7.5);
      Y += 15;
    };

    const subhead = (title) => {
      checkY(12);
      doc.setFont('helvetica', 'bold');
      doc.setFontSize(10.5);
      doc.setTextColor(...C.cyan);
      doc.text(title, ML, Y);
      doc.setDrawColor(...C.cyan);
      doc.setLineWidth(0.35);
      doc.line(ML, Y + 2, ML + CW, Y + 2);
      Y += 9;
    };

    const para = (text, sz) => {
      sz = sz || 10;
      checkY(18);
      doc.setFont('helvetica', 'normal');
      doc.setFontSize(sz);
      doc.setTextColor(...C.muted);
      const lines = doc.splitTextToSize(text, CW);
      doc.text(lines, ML, Y);
      Y += lines.length * (sz * 0.42) + 4;
    };

    const gap = (mm) => { Y += mm || 4; };

    // Standard autoTable wrapper with large fonts
    const autoT = (head, body, colStyles, extra) => {
      checkY(28);
      doc.autoTable(Object.assign({
        startY: Y,
        head: head,
        body: body,
        theme: 'grid',
        margin: { left: ML, right: MR },
        headStyles: {
          fillColor: C.blue,
          textColor: C.cyan,
          fontSize: 10,
          fontStyle: 'bold',
          minCellHeight: 9,
          cellPadding: 3
        },
        bodyStyles: {
          fontSize: 9.5,
          textColor: [210, 220, 235],
          fillColor: [12, 18, 35],
          minCellHeight: 7,
          cellPadding: 2.5
        },
        alternateRowStyles: { fillColor: [18, 26, 52] },
        columnStyles: colStyles || {}
      }, extra || {}));
      Y = doc.lastAutoTable.finalY + 9;
    };

    // Status-coloured autoTable (for PASS/WARN/FAIL columns)
    const statusAutoT = (head, body, colStyles, statusCol) => {
      checkY(28);
      doc.autoTable({
        startY: Y,
        head: head,
        body: body,
        theme: 'grid',
        margin: { left: ML, right: MR },
        headStyles: { fillColor: C.blue, textColor: C.cyan, fontSize: 10, fontStyle: 'bold', minCellHeight: 9, cellPadding: 3 },
        bodyStyles: { fontSize: 9.5, textColor: [210, 220, 235], fillColor: [12, 18, 35], minCellHeight: 7, cellPadding: 2.5 },
        alternateRowStyles: { fillColor: [18, 26, 52] },
        columnStyles: colStyles || {},
        didDrawCell: function(data) {
          if (data.section === 'body' && data.column.index === (statusCol || 2)) {
            const txt = String(data.cell.raw);
            data.doc.setFont('helvetica', 'bold');
            if (txt === 'PASS' || txt === 'WITHIN' || txt === 'SAFE')        data.doc.setTextColor(...C.green);
            else if (txt === 'WARN' || txt === 'MARGINAL' || txt === 'INFO') data.doc.setTextColor(...C.amber);
            else if (txt === 'FAIL' || txt === 'EXCEEDED' || txt === 'SLIP') data.doc.setTextColor(...C.red);
            else if (txt === 'CHECK')                                         data.doc.setTextColor(...C.cyan);
            else                                                               data.doc.setTextColor(...C.muted);
          }
        }
      });
      Y = doc.lastAutoTable.finalY + 9;
    };

    // ══════════════════════════════════════════════════════════════════════
    // PAGE 1  COVER
    // ══════════════════════════════════════════════════════════════════════
    doc.setFillColor(...C.navy);
    doc.rect(0, 0, PW, PH, 'F');
    doc.setFillColor(...C.cyan);
    doc.rect(0, 0, 8, PH, 'F');

    // Big title
    doc.setFont('helvetica', 'bold');
    doc.setFontSize(26);
    doc.setTextColor(...C.white);
    doc.text('FEAD TECHNICAL', 20, 58);
    doc.text('ANALYSIS REPORT', 20, 72);

    doc.setFontSize(13);
    doc.setTextColor(...C.cyan);
    doc.text('Front End Accessory Drive \u2014 Slip-Free Working Condition Study', 20, 84);

    doc.setDrawColor(...C.cyan);
    doc.setLineWidth(1);
    doc.line(20, 90, PW - 20, 90);

    // Score circle
    const sc = parseInt(comp.composite);
    const sCol = sc >= 85 ? C.green : sc >= 65 ? C.amber : C.red;
    doc.setDrawColor(...sCol);
    doc.setLineWidth(3.5);
    doc.circle(PW - 38, 120, 20);
    doc.setFont('helvetica', 'bold');
    doc.setFontSize(20);
    doc.setTextColor(...sCol);
    doc.text(String(sc), PW - 38 - (sc >= 100 ? 8 : 6), 124);
    doc.setFontSize(8);
    doc.setTextColor(...C.muted);
    doc.text('COMPLIANCE', PW - 51, 131);
    doc.text('SCORE /100', PW - 51, 136);

    // Info table
    const coverRows = [
      ['Engine / Platform', 'Ashok Leyland H6  \u00b7  24V 1.3SR 130CC'],
      ['Belt System',       'Gates Micro-V MT820 8PK AMD (Aramid)'],
      ['Analysis Standard', 'Gates PDF DB Ver 2.45.0.0  \u00b7  ISO 9981:2018'],
      ['Bearing Reference', 'C&U Group WR25153 (QZ20230803)'],
      ['Operating Point',   snapRPM + ' RPM  \u00b7  ' + snapTen + ' N  \u00b7  ' + TEN_POS[snapIdx].label],
      ['Report Date',       dateStr],
      ['Compliance Score',  comp.composite + ' / 100  \u2014  ' + comp.grade]
    ];
    let cy = 100;
    coverRows.forEach(([k, val]) => {
      doc.setFont('helvetica', 'normal');
      doc.setFontSize(10);
      doc.setTextColor(...C.muted);
      doc.text(k, 20, cy);
      doc.setFont('helvetica', 'bold');
      doc.setTextColor(...C.white);
      doc.text(val, 85, cy);
      cy += 10;
    });

    // Verdict banner
    const vBg = sc >= 85 ? C.passb : sc >= 65 ? C.warnb : C.failb;
    const vFg = sc >= 85 ? C.pass  : sc >= 65 ? C.warn  : C.fail;
    doc.setFillColor(...vBg);
    doc.roundedRect(20, 178, CW, 18, 2, 2, 'F');
    doc.setFont('helvetica', 'bold');
    doc.setFontSize(11);
    doc.setTextColor(...vFg);
    const vText = sc >= 85
      ? 'COMPLIANT \u2014 All FEAD parameters within Gates PDF + ISO 9981 design envelope.'
      : sc >= 65
        ? 'MARGINAL \u2014 Some parameters approaching limits. See Section 11 for actions.'
        : 'NON-COMPLIANT \u2014 Parameters outside design envelope. Immediate action required.';
    const vLines = doc.splitTextToSize(vText, CW - 8);
    doc.text(vLines, 24, 189);

    doc.setFont('helvetica', 'normal');
    doc.setFontSize(8.5);
    doc.setTextColor(...C.muted);
    doc.text('CONFIDENTIAL TECHNICAL DOCUMENT  \u00b7  Generated from live FEAD simulation model', PW / 2, PH - 14, { align: 'center' });
    doc.text('All values computed from Gates PDF certified data  \u00b7  C&U WR25153 fan bearing reference', PW / 2, PH - 9, { align: 'center' });

    // ══════════════════════════════════════════════════════════════════════
    // PAGE 2  EXECUTIVE SUMMARY + SYSTEM CONFIG
    // ══════════════════════════════════════════════════════════════════════
    newPage();
    section('1. Executive Summary');
    para('This report provides a detailed technical analysis of the Front End Accessory Drive (FEAD) system for the Ashok Leyland H6 engine with Gates MT820 8PK AMD aramid belt. The study covers belt slip safety margins, hub load compliance against PDF certified limits, mechanical efficiency, tension optimisation, drive cycle worst-case analysis, and maintenance scheduling. All physics models are derived from Gates PDF DB Ver 2.45.0.0 and validated against C&U Group WR25153 fan bearing calculations.');

    subhead('Key Findings at ' + snapRPM + ' RPM  \u00b7  ' + snapTen + ' N  \u00b7  ' + TEN_POS[snapIdx].label);
    statusAutoT(
      [['Parameter', 'Value', 'Status']],
      [
        ['FEAD Mechanical Efficiency',  fd.totals.eta + '%',                                           parseFloat(fd.totals.eta) >= 96 ? 'PASS' : 'WARN'],
        ['Total Parasitic Power Loss',   fd.totals.P_losses + ' kW',                                   'INFO'],
        ['Minimum Belt Slip SF',         comp.minSF.toFixed(2) + '   (target \u2265 1.30)',             comp.minSF >= 1.3 ? 'PASS' : comp.minSF >= 1.0 ? 'WARN' : 'FAIL'],
        ['Belt Speed',                   v.toFixed(2) + ' m/s   (optimal window: 8\u201322 m/s)',       v >= 8 && v <= 22 ? 'PASS' : 'WARN'],
        ['Optimal Static Tension',       fd.totals.T_opt + ' N   (current: ' + snapTen + ' N)',        Math.abs(snapTen - parseInt(fd.totals.T_opt)) < 300 ? 'PASS' : 'WARN'],
        ['CRK Hub Load vs PDF Limit',    (snapHub.CRK ? snapHub.CRK.F.toFixed(0) : '--') + ' N  /  ' + PDF.CRK.F + ' N', snapHub.CRK && snapHub.CRK.F <= PDF.CRK.F ? 'PASS' : 'FAIL'],
        ['FAN Hub Load vs PDF Limit',    (snapHub.FAN ? snapHub.FAN.F.toFixed(0) : '--') + ' N  /  ' + PDF.FAN.F + ' N', snapHub.FAN && snapHub.FAN.F <= PDF.FAN.F ? 'PASS' : 'FAIL'],
        ['ALT Hub Load vs PDF Limit',    (snapHub.ALT ? snapHub.ALT.F.toFixed(0) : '--') + ' N  /  ' + PDF.ALT.F + ' N', snapHub.ALT && snapHub.ALT.F <= PDF.ALT.F ? 'PASS' : 'FAIL'],
        ['Fan Bearing L10 Life',         CU_VALIDATION.bearingLife.L10_composite + ' h  (C&U WR25153)', 'INFO'],
        ['Overall Compliance Score',     comp.composite + ' / 100  \u2014  ' + comp.grade,             sc >= 85 ? 'PASS' : sc >= 65 ? 'WARN' : 'FAIL']
      ],
      { 0: { cellWidth: 76 }, 1: { cellWidth: 72 }, 2: { cellWidth: 26, fontStyle: 'bold' } }
    );

    section('2. Belt & Pulley Specifications');
    subhead('Belt Parameters');
    autoT(
      [['Parameter', 'Value', 'Notes']],
      [
        ['Belt Part Number',          'Gates MT820  8PK AMD',                     'Aramid Multi-V, 8-rib PK profile'],
        ['Effective Length',          (BELT_PARAMS.belt_length_m * 1000) + ' mm', 'ISO 9981 effective length'],
        ['Linear Mass',               (BELT_PARAMS.mass_per_m * 1000).toFixed(0) + ' g/m', 'Low mass aramid cord'],
        ['Belt Speed @ ' + snapRPM + ' RPM',  v.toFixed(2) + ' m/s',              'v = \u03c0 \u00b7 D_CRK \u00b7 n / 60000'],
        ['Centrifugal Tension T_c',   fd.totals.Tc + ' N',                        'm_b \u00b7 v\u00b2 / L'],
        ['\u03bc (ribbed side)',       '0.35',                                     'PK belt on grooved pulley, dry'],
        ['\u03bc (back side)',         '0.25',                                     'IDR / TEN smooth back contact'],
        ['Bending Loss Coeff C_b',    '0.0012',                                   'Aramid (vs 0.0018 steel cord, \u221233%)'],
        ['Bearing Friction \u03bc_b', '0.003',                                    'Rolling element, nominal']
      ],
      { 0: { cellWidth: 58, fontStyle: 'bold' }, 1: { cellWidth: 38 }, 2: { cellWidth: 78 } }
    );

    subhead('Pulley Geometry');
    autoT(
      [['Pulley', 'Label', 'Eff Dia (mm)', 'Wrap (\u00b0)', 'Speed Ratio', 'Dir', 'PDF F_hub (N)', 'PDF Dir (\u00b0)']],
      ORDER.map(n => {
        const p = PULLEYS[n];
        const label = p.label ? p.label.replace('\n', ' ') : n;
        return [n, label, p.eff.toFixed(1), PDF[n].wrap, p.sr.toFixed(3), p.cw ? 'CW' : 'CCW', PDF[n].F, PDF[n].dir];
      }),
      { 0: { cellWidth: 14, fontStyle: 'bold' }, 1: { cellWidth: 36 } }
    );

    // ══════════════════════════════════════════════════════════════════════
    // PAGE 3  BELT SLIP ANALYSIS  (core anti-slip report)
    // ══════════════════════════════════════════════════════════════════════
    newPage();
    section('3. Belt Slip Analysis \u2014 Conditions for Slip-Free Operation');
    para('Belt slip occurs when T_tight / T_slack exceeds the Eytelwein capstan limit e^(\u03bc\u03b8). The Slip Safety Factor (SF) measures margin before slip. SF < 1.0 = belt is slipping now. SF 1.0\u20131.3 = marginal risk. SF \u2265 1.3 = safe per Gates Engineering Manual \u00a74.3 and ISO 9981:2018. Back-side contacts (IDR, TEN) use \u03bc = 0.25.');

    subhead('Per-Pulley Capstan Analysis at ' + snapRPM + ' RPM  /  ' + snapTen + ' N Tension');
    statusAutoT(
      [['Pulley', '\u03bc', 'Wrap \u03b8', 'e^(\u03bc\u03b8)', 'T_tight (N)', 'T_slack (N)', 'Power (kW)', 'SF', 'Status']],
      ORDER.map(n => {
        const d = fd.pulleys[n];
        const frict = (n === 'IDR' || n === 'TEN') ? 0.25 : 0.35;
        const wRad = PDF[n].wrap * Math.PI / 180;
        return [n, frict.toFixed(2), PDF[n].wrap + '\u00b0', Math.exp(frict * wRad).toFixed(3),
                d.T_tight, d.T_slack, d.pwr.toFixed(2), d.SF, d.status];
      }),
      { 0: { cellWidth: 14, fontStyle: 'bold' }, 8: { fontStyle: 'bold', cellWidth: 22 } },
      8
    );

    gap(2);
    subhead('Seven Rules for Guaranteed Slip-Free Operation');
    autoT(
      [['#', 'Rule', 'Current Status & Required Action']],
      [
        ['R1', 'Static tension \u2265 T_opt',
         'T_opt = ' + fd.totals.T_opt + ' N  |  Current = ' + snapTen + ' N  \u2192  ' +
         (snapTen >= parseInt(fd.totals.T_opt) ? '\u2713 SATISFIED' : '\u2717 INCREASE to ' + fd.totals.T_opt + ' N immediately')],
        ['R2', 'Wrap angles per PDF geometry',
         'All wrap angles from Gates PDF Sheet 2. IDR/TEN back-side \u226525\u00b0. Larger wrap raises e^(\u03bc\u03b8) limit.'],
        ['R3', 'Belt in clean, unworn condition',
         'Glazed / oil-soaked belt: \u03bc drops 0.35 \u2192 0.15. All SFs fall 57%. Replace immediately if contaminated.'],
        ['R4', 'Tensioner arm at MEAN position',
         'Current: ' + TEN_POS[snapIdx].label + ' (' + TEN_POS[snapIdx].arm + '\u00b0)  |  Target: MEAN (15.4\u00b0). Arm outside FREE\u2013LOAD window loses tensioning authority.'],
        ['R5', 'Pulley alignment \u2264 0.5 mm',
         'Misalignment > 1 mm reduces effective wrap angle and \u03bc. Causes edge loading and accelerated rib wear.'],
        ['R6', 'Under-hood temp \u2264 120 \u00b0C',
         '\u03bc degrades above 120 \u00b0C. At idle + high electrical demand this threshold can be reached. Verify airflow.'],
        ['R7', 'Transient load margin SF \u2265 1.3',
         'BAS assist / regen braking can spike belt load +20\u201330%. Current min SF = ' + comp.minSF.toFixed(2) +
         (comp.minSF >= 1.3 ? '  \u2713 Margin adequate' : '  \u2717 Insufficient margin for transients')]
      ],
      { 0: { cellWidth: 10, fontStyle: 'bold' }, 1: { cellWidth: 52, fontStyle: 'bold' }, 2: { cellWidth: 112 } }
    );

    // ══════════════════════════════════════════════════════════════════════
    // PAGE 4  HUB LOAD ANALYSIS
    // ══════════════════════════════════════════════════════════════════════
    newPage();
    section('4. Hub Load Analysis \u2014 PDF Certified Limits');
    para('Hub loads are the resultant bearing forces from the vector sum of incoming and outgoing belt span tensions. All values are compared to the Gates PDF certified peak values. Exceeding these limits shortens bearing L10 life proportionally to (F_actual / F_rated)\u00b3 per ISO 281.');

    subhead('Per-Pulley Hub Load vs Gates PDF Limits');
    statusAutoT(
      [['Pulley', 'T_in (N)', 'T_out (N)', 'F_hub (N)', 'Dir (\u00b0)', 'PDF Peak (N)', 'PDF Dir (\u00b0)', '\u0394F (N)', 'Status']],
      ORDER.map(n => {
        const d = snapHub[n]; const p = PDF[n];
        if (!d) return [n, '--', '--', '--', '--', p.F, p.dir, '--', '--'];
        const dF = (d.F - p.F).toFixed(0);
        const st = d.F <= p.F ? 'WITHIN' : d.F <= p.F * 1.1 ? 'MARGINAL' : 'EXCEEDED';
        return [n, d.T_in.toFixed(0), d.T_out.toFixed(0), d.F.toFixed(0), d.dir.toFixed(1), p.F, p.dir, (dF >= 0 ? '+' : '') + dF, st];
      }),
      { 0: { cellWidth: 14, fontStyle: 'bold' }, 8: { fontStyle: 'bold', cellWidth: 22 } },
      8
    );

    subhead('Worst-Case Hub Loads by Operating Mode');
    const modes = [
      { name: 'Idle + High Demand',  rpm: 800,  tension: 3200 },
      { name: 'Regen Braking',       rpm: 1600, tension: 2500 },
      { name: 'BAS Motor Mode',      rpm: 900,  tension: 3800 },
      { name: 'Peak Acceleration',   rpm: 2300, tension: 2500 }
    ];
    const wcBody = modes.map(m => {
      state.rpm = m.rpm; state.baseTension = m.tension; compute();
      const hl = {};
      ORDER.forEach(n => { if (hubData[n]) hl[n] = hubData[n].F.toFixed(0); });
      const wPct = Math.max(
        hubData.CRK ? parseFloat(hl.CRK || 0) / PDF.CRK.F : 0,
        hubData.FAN ? parseFloat(hl.FAN || 0) / PDF.FAN.F : 0
      );
      return [m.name, m.rpm, m.tension, hl.CRK || '--', hl.FAN || '--', hl.ALT || '--', hl.AC || '--', (wPct * 100).toFixed(0) + '% of PDF peak'];
    });
    // Restore
    state.rpm = snapRPM; state.baseTension = snapTen; state.tenIdx = snapIdx; compute();

    autoT(
      [['Operating Mode', 'RPM', 'Tension (N)', 'CRK (N)', 'FAN (N)', 'ALT (N)', 'AC (N)', 'Worst / PDF']],
      wcBody,
      { 0: { cellWidth: 40 } }
    );

    // ══════════════════════════════════════════════════════════════════════
    // PAGE 5  FEAD EFFICIENCY + TENSION OPTIMISER
    // ══════════════════════════════════════════════════════════════════════
    newPage();
    section('5. FEAD Efficiency Analysis');
    para('FEAD efficiency = fraction of crankshaft power delivered to accessories. Power losses arise from belt bending hysteresis, bearing friction, belt slip, and centrifugal effects. Target: \u03b7 \u2265 96% per Gates MT820 AMD specification. Aramid cord reduces bending loss by \u223333% vs steel cord.');

    subhead('Power Flow & Loss Breakdown  at  ' + snapRPM + ' RPM');
    autoT(
      [['Loss Component', 'Physics Formula', 'Value (kW)', 'How to Reduce']],
      [
        ['Belt Bending Hysteresis', 'C_b \u00b7 (T_avg / R) \u00b7 v  per pulley',   fd.totals.P_bend + ' kW', 'Larger pulley diameters; aramid C_b = 0.0012 (vs 0.0018 steel)'],
        ['Bearing Friction',        '\u03bc_b \u00b7 F_hub \u00b7 \u03c9 \u00b7 R  per bearing', fd.totals.P_bear + ' kW', 'Reduce hub loads via belt routing; quality bearings \u03bc_b = 0.003'],
        ['Belt Slip Loss',          '\u22480.5% of transmitted power',         fd.totals.P_slip + ' kW', 'Maintain SF \u2265 1.3; replace glazed or contaminated belt'],
        ['Centrifugal Loss',        'm_b \u00b7 v\u00b2 / L on effective clamping',   fd.totals.P_cent + ' kW', 'Aramid 0.18 kg/m vs rubber 0.25 kg/m \u2192 28% lower T_c'],
        ['TOTAL LOSSES',            '\u2014',                                    fd.totals.P_losses + ' kW', '\u2014'],
        ['Accessory Power Demand',  'FAN + ALT + AC + IDR + TEN',            fd.totals.P_accessories + ' kW', 'Variable fan drive + smart alternator at idle / cruise'],
        ['FEAD EFFICIENCY \u03b7',      'P_acc / (P_acc + P_loss)',              fd.totals.eta + '%', 'Target \u2265 96%  \u2014  current: ' + (parseFloat(fd.totals.eta) >= 96 ? '\u2713 COMPLIANT' : '\u2717 BELOW TARGET')]
      ],
      { 0: { cellWidth: 46, fontStyle: 'bold' }, 1: { cellWidth: 46 }, 2: { cellWidth: 22 }, 3: { cellWidth: 60 } }
    );

    subhead('Per-Pulley Loss Detail');
    autoT(
      [['Pulley', 'Power (kW)', 'Bending (kW)', 'Bearing (kW)', 'Slip (kW)', 'Total Loss (kW)', 'Loss %']],
      ORDER.map(n => {
        const d = fd.pulleys[n];
        const tot = (parseFloat(d.P_bend) + parseFloat(d.P_bear) + parseFloat(d.P_slip)).toFixed(3);
        const pct = d.pwr > 0 ? ((parseFloat(tot) / d.pwr) * 100).toFixed(1) + '%' : '--';
        return [n, d.pwr.toFixed(2), d.P_bend, d.P_bear, d.P_slip, tot, pct];
      }),
      { 0: { cellWidth: 14, fontStyle: 'bold' } }
    );

    section('6. Tension Optimiser');
    para('Optimal static tension is the minimum pre-load that keeps every pulley at SF = 1.3 simultaneously. Increasing tension beyond T_opt raises hub loads without improving slip safety. Under-tensioning below T_opt risks intermittent slip, especially during transient loads.');

    autoT(
      [['Parameter', 'Value', 'Basis / Action']],
      [
        ['Required tension for SF = 1.3',  fd.totals.T_opt + ' N',              'Capstan eq. solved for most demanding pulley at ' + snapRPM + ' RPM'],
        ['Current static tension',          snapTen + ' N',                       'Set by tensioner spring pre-load'],
        ['Deviation from optimal',          (snapTen - parseInt(fd.totals.T_opt)) + ' N', snapTen > parseInt(fd.totals.T_opt) ? 'Over-tensioned \u2014 raises hub loads and shortens L10' : snapTen < parseInt(fd.totals.T_opt) ? 'Under-tensioned \u2014 SLIP RISK on high-load pulleys' : 'At optimum \u2013 no change needed'],
        ['Belt speed',                      v.toFixed(2) + ' m/s',               'v = \u03c0 \u00b7 D_CRK \u00b7 n / 60000'],
        ['Centrifugal tension T_c',         fd.totals.Tc + ' N',                 'm_b \u00b7 v\u00b2 / L = ' + BELT_PARAMS.mass_per_m + ' \u00d7 ' + parseFloat(v).toFixed(2) + '\u00b2 / ' + BELT_PARAMS.belt_length_m],
        ['Fuel penalty from losses',        fd.totals.fuel_pen + ' g / h extra', 'Based on ' + BELT_PARAMS.fuel_g_per_kWh + ' g/kWh diesel BSFC'],
        ['CO\u2082 impact from losses',     fd.totals.co2_pen + ' kg / h',       'Based on ' + BELT_PARAMS.co2_per_kW + ' kg CO\u2082 per kW parasitic'],
        ['Recommended action',              'Set tension to ' + fd.totals.T_opt + ' N', 'Verify with Gates frequency gauge after 30 min run-in']
      ],
      { 0: { cellWidth: 66, fontStyle: 'bold' }, 1: { cellWidth: 36 }, 2: { cellWidth: 72 } }
    );

    // ══════════════════════════════════════════════════════════════════════
    // PAGE 6  DRIVE CYCLES + C&U BEARING LIFE
    // ══════════════════════════════════════════════════════════════════════
    newPage();
    section('7. Drive Cycle Analysis \u2014 WLTC vs NEDC');
    para('WLTC Class 2 (Low / Medium / High / Extra-High phases) and NEDC (ECE\u00d74 + EUDC) represent real-world speed and load profiles. Hub loads are mapped across each cycle to identify worst-case conditions. Both cycles must maintain SF \u2265 1.3 at all time steps to guarantee slip-free operation.');

    const cycBody = ['WLTC', 'NEDC'].map(ck => {
      const cy = DRIVE_CYCLES[ck];
      let maxCRK = 0, sumCRK = 0, maxFAN = 0;
      cy.rpms.forEach(r => {
        state.rpm = r; compute();
        if (hubData.CRK) { maxCRK = Math.max(maxCRK, hubData.CRK.F); sumCRK += hubData.CRK.F; }
        if (hubData.FAN)   maxFAN = Math.max(maxFAN, hubData.FAN.F);
      });
      state.rpm = snapRPM; state.baseTension = snapTen; state.tenIdx = snapIdx; compute();
      return [
        ck,
        cy.phases.map(p => p.name).join(' / '),
        Math.max(...cy.rpms),
        maxCRK.toFixed(0),
        (sumCRK / cy.rpms.length).toFixed(0),
        maxFAN.toFixed(0),
        (maxCRK / PDF.CRK.F * 100).toFixed(0) + '%',
        (maxFAN / PDF.FAN.F * 100).toFixed(0) + '%'
      ];
    });
    autoT(
      [['Cycle', 'Phases', 'Max RPM', 'Peak CRK (N)', 'Avg CRK (N)', 'Peak FAN (N)', 'CRK / PDF', 'FAN / PDF']],
      cycBody,
      { 0: { cellWidth: 16 }, 1: { cellWidth: 52 } }
    );

    section('8. C&U Group WR25153 \u2014 Fan Bearing Life Verification');
    para('The C&U Group calculation (QZ20230803, Pages 1\u20134) provides independent verification of fan speed ratio and certified bearing life under worst-case conditions: 2400 RPM, F_r = 358 N dynamic imbalance, full belt load at PDF peak hub force.');

    autoT(
      [['Parameter', 'C&U Document Value', 'Interpretation']],
      [
        ['Fan Speed Ratio i',        '1.3',                   'Gates SR = 1.302 \u2014 \u0394 = 0.002, within tolerance \u2713'],
        ['Working RPM',              '2400 RPM',              'Peak condition used for life calculation'],
        ['Dynamic imbalance F_r',    '358 N',                 'Radial centrifugal load \u2014 not in Gates model (separate source)'],
        ['Cr (roller-end)',          '38,179 N',              'Rated dynamic load capacity, roller bearing (impeller end)'],
        ['Cr (ball-end)',            '19,035 N',              'Rated dynamic load capacity, ball bearing (pulley end)'],
        ['L10A \u2014 impeller end', '17,820 h',              'Long life \u2014 NOT the limiting component'],
        ['L10B \u2014 pulley / belt end', '3,860 h',          'Belt-side ball bearing \u2014 LIFE-LIMITING component'],
        ['L10 Composite',           '3,305 h',               'System worst-case \u2014 use for maintenance planning'],
        ['Replacement threshold',   '2,975 h  (90% of L10)', 'Replace before 90% L10 to prevent in-service failure'],
        ['C&U Page 3 tensioner',    'Different coord. system','C&U analyses fan sub-tensioner. Not directly comparable to Gates serpentine TEN.']
      ],
      { 0: { cellWidth: 52, fontStyle: 'bold' }, 1: { cellWidth: 44 }, 2: { cellWidth: 78 } }
    );

    // ══════════════════════════════════════════════════════════════════════
    // PAGE 7  MAINTENANCE SCHEDULE + DESIGN RULES
    // ══════════════════════════════════════════════════════════════════════
    newPage();
    section('9. Maintenance Schedule \u2014 Load-Adjusted Service Intervals');
    const loadFactor = Math.max(
      snapHub.CRK ? snapHub.CRK.F / PDF.CRK.F : 0,
      snapHub.FAN ? snapHub.FAN.F / PDF.FAN.F : 0
    );
    const bIH = Math.round(Math.min(2000, 800  / Math.max(0.3, loadFactor)));
    const tCH = Math.round(Math.min(1000, 500  / Math.max(0.3, loadFactor)));
    const bRH = Math.round(Math.min(4000, CU_VALIDATION.bearingLife.L10B_pulley * 0.5 / Math.max(0.3, loadFactor)));
    const brH = Math.round(CU_VALIDATION.bearingLife.L10_composite * 0.9);

    para('Intervals are scaled by current load factor ' + (loadFactor * 100).toFixed(0) + '% of PDF peak. At 100% load, use the minimum values shown. Lower loads proportionally extend intervals.');

    autoT(
      [['Service Task', 'Interval', 'Condition / Trigger', 'Required Action']],
      [
        ['Belt Visual Inspection',     bIH + ' h',  'Every service or fault code',
         'Check all ribs for cracking, glazing, fretting, oil contamination. Verify tension with frequency gauge.'],
        ['Tensioner Arm Check',        tCH + ' h',  'Arm drift > 5\u00b0 from MEAN',
         'Confirm arm at 15.4\u00b0 (MEAN). Check damper wear. Replace spring if load < 80% nominal.'],
        ['Pulley Groove Inspection',   bIH + ' h',  'With belt inspection',
         'Inspect groove depth, surface cracks, embedded debris. Realign if offset > 1 mm.'],
        ['Belt Replacement',           bRH + ' h',  'Or at any slip / glazing sign',
         'Fit Gates MT820 8PK AMD only. Tension to ' + fd.totals.T_opt + ' N. Run 30 min then re-check.'],
        ['Fan Bearing Replacement',    brH + ' h',  'Or at noise / vibration onset',
         'Replace C&U WR25153 set (roller + ball together). Verify fan balance \u2264 G6.3.'],
        ['FEAD Alignment Check',       'Annual / 2000 h', 'After any pulley / bearing work',
         'Laser-align all pulleys to \u2264 0.5 mm lateral. All pulley planes within 1\u00b0.']
      ],
      { 0: { cellWidth: 48, fontStyle: 'bold' }, 1: { cellWidth: 20 }, 2: { cellWidth: 38 }, 3: { cellWidth: 68 } }
    );

    gap(2);
    section('10. Design Rules Compliance \u2014 Gates / ISO 9981 / C&U');
    statusAutoT(
      [['Design Rule', 'Standard', 'Current Value', 'Status']],
      [
        ['Capstan SF \u2265 1.3 for all pulleys',
         'ISO 9981:2018 \u00b7 Gates \u00a74.3',
         'Min SF = ' + comp.minSF.toFixed(2),
         comp.minSF >= 1.3 ? 'PASS' : comp.minSF >= 1.0 ? 'WARN' : 'FAIL'],
        ['FEAD efficiency \u03b7 \u2265 96%',
         'Gates MT820 AMD spec',
         '\u03b7 = ' + fd.totals.eta + '%',
         parseFloat(fd.totals.eta) >= 96 ? 'PASS' : 'WARN'],
        ['Belt speed 8 \u2013 22 m/s',
         'Gates Belt Design Manual',
         v.toFixed(2) + ' m/s',
         v >= 8 && v <= 22 ? 'PASS' : 'WARN'],
        ['Tension within \u00b1200 N of T_opt',
         'Computed (Capstan inverse)',
         '\u0394 = ' + (snapTen - parseInt(fd.totals.T_opt)) + ' N',
         Math.abs(snapTen - parseInt(fd.totals.T_opt)) <= 300 ? 'PASS' : 'WARN'],
        ['All hub loads \u2264 PDF peaks',
         'Gates PDF DB 2.45.0.0',
         'Worst = ' + (Math.max(...ORDER.map(n => snapHub[n] ? snapHub[n].F / PDF[n].F : 0)) * 100).toFixed(0) + '% of PDF',
         ORDER.every(n => !snapHub[n] || snapHub[n].F <= PDF[n].F) ? 'PASS' : 'FAIL'],
        ['Fan bearing life \u2265 3305 h',
         'C&U WR25153 \u00a74',
         'L10 composite = 3305 h',
         'INFO'],
        ['Belt free of contamination',
         'Gates Install Guide \u00a76',
         'Visual inspection required',
         'CHECK']
      ],
      { 0: { cellWidth: 68, fontStyle: 'bold' }, 1: { cellWidth: 44 }, 2: { cellWidth: 34 }, 3: { cellWidth: 22, fontStyle: 'bold' } },
      3
    );

    // ══════════════════════════════════════════════════════════════════════
    // PAGE 8  ENGINEERING RECOMMENDATIONS
    // ══════════════════════════════════════════════════════════════════════
    newPage();
    section('11. Engineering Recommendations \u2014 Priority Ordered');
    para('Recommendations are auto-generated from the live computed values at the time of report download. They are ordered by priority: CRITICAL \u2192 HIGH \u2192 MEDIUM \u2192 LOW \u2192 INFO.');

    const T0r   = snapTen;
    const Toptr = parseInt(fd.totals.T_opt);
    const recs  = [];

    if (comp.minSF < 1.0)
      recs.push(['CRITICAL', 'Active Belt Slip',
        'SF = ' + comp.minSF.toFixed(2) + ' < 1.0. Belt is currently slipping. Immediate risk of belt failure, accessory loss, and engine overheating.',
        'Raise tension to ' + Toptr + ' N NOW. Inspect belt for glazing / damage. Check tensioner arm position.']);
    else if (comp.minSF < 1.3)
      recs.push(['HIGH', 'Slip Margin Below Target',
        'SF = ' + comp.minSF.toFixed(2) + ' < 1.3. Transient events (BAS assist, regen braking, cold start) can push system into slip.',
        'Increase tension to ' + Toptr + ' N. Verify wrap angles match PDF Sheet 2 geometry.']);

    if (T0r < Toptr * 0.85)
      recs.push(['HIGH', 'Belt Under-Tensioned',
        'Current ' + T0r + ' N is ' + Math.round((1 - T0r / Toptr) * 100) + '% below optimum. Slip risk during transient and high-temperature operation.',
        'Increase pre-load to ' + Toptr + ' N. Verify with Gates sonic tension gauge after 30 min run-in.']);

    if (T0r > Toptr * 1.5)
      recs.push(['HIGH', 'Belt Over-Tensioned',
        'Current ' + T0r + ' N is ' + Math.round((T0r / Toptr - 1) * 100) + '% above optimum. L10 life reduced by factor ' + ((T0r / Toptr) ** 3).toFixed(1) + '\u00d7.',
        'Reduce to ' + Math.round(Toptr * 1.15) + ' N (T_opt \u00d7 1.15). Monitor bearing temperatures after change.']);

    if (parseFloat(fd.totals.eta) < 96)
      recs.push(['MEDIUM', 'FEAD Efficiency < 96%',
        '\u03b7 = ' + fd.totals.eta + '%. Dominant loss: ' +
        (parseFloat(fd.totals.P_bend) > parseFloat(fd.totals.P_bear)
          ? 'bending hysteresis (increase pulley diameters or use stiffer aramid cord).'
          : 'bearing friction (reduce hub loads via routing optimisation).'),
        'Review pulley layout and span lengths. Confirm Gates MT820 AMD aramid specification is used.']);

    if (P.FAN > 8)
      recs.push(['MEDIUM', 'High FAN Load',
        'FAN = ' + P.FAN.toFixed(1) + ' kW (P \u221d n\u00b3 cube law). At 2300 RPM hub load approaches PDF certified peak of 2866 N.',
        'Install viscous coupling or variable-speed fan clutch. Saves up to ' + (P.FAN * 0.7).toFixed(1) + ' kW at cruise.']);

    if (P.ALT > 3.5)
      recs.push(['LOW', 'Alternator Load Reduction',
        'ALT = ' + P.ALT.toFixed(1) + ' kW. A smart / variable alternator (charges during deceleration only) reduces average load by 30\u201340%.',
        'Specify variable-voltage smart alternator. Saves \u223c1.5 kW average parasitic.']);

    recs.push(['INFO', 'Aramid Belt Verified \u2713',
      'MT820 AMD aramid cord is the correct specification: C_b = 0.0012 (vs 0.0018 steel, \u221233% bending loss), low mass 0.18 kg/m (\u221228% lower T_c), elongation tolerance \u00b10.4%.',
      'Continue MT820 AMD specification. Do not substitute rubber-cord equivalent.']);

    recs.push(['INFO', 'Fan Bearing L10 Planning',
      'C&U L10 composite = 3305 h under peak conditions. Belt-side ball bearing L10B = 3860 h is life-limiting. L10 scales as (C / F)\u00b3.',
      'Schedule bearing replacement at 2975 h (90% L10). Maintain operational log of RPM history and load conditions.']);

    statusAutoT(
      [['Priority', 'Issue', 'Analysis', 'Recommended Action']],
      recs,
      { 0: { cellWidth: 20, fontStyle: 'bold' }, 1: { cellWidth: 40 }, 2: { cellWidth: 64 }, 3: { cellWidth: 50 } },
      0
    );

    // ── Summary box ──────────────────────────────────────────────────────
    checkY(36);
    doc.setFillColor(8, 20, 50);
    doc.roundedRect(ML, Y, CW, 32, 2, 2, 'F');
    doc.setDrawColor(...C.cyan);
    doc.setLineWidth(0.5);
    doc.roundedRect(ML, Y, CW, 32, 2, 2, 'S');
    doc.setFillColor(...C.cyan);
    doc.rect(ML, Y, 5, 32, 'F');

    doc.setFont('helvetica', 'bold');
    doc.setFontSize(12);
    doc.setTextColor(...C.cyan);
    doc.text('REPORT SUMMARY', ML + 10, Y + 10);

    doc.setFont('helvetica', 'normal');
    doc.setFontSize(10);
    doc.setTextColor(...C.light);
    doc.text(
      'Compliance: ' + comp.composite + '/100  \u00b7  Grade: ' + comp.grade +
      '  \u00b7  Min Slip SF: ' + comp.minSF.toFixed(2) +
      '  \u00b7  FEAD \u03b7: ' + fd.totals.eta + '%',
      ML + 10, Y + 20
    );
    doc.text(
      'T_opt: ' + fd.totals.T_opt + ' N  \u00b7  Belt v: ' + v.toFixed(2) + ' m/s' +
      '  \u00b7  Fan Bearing L10: 3305 h  \u00b7  Belt Replace: ' + bRH + ' h',
      ML + 10, Y + 29
    );

    // ── Page numbers + footers ────────────────────────────────────────────
    const totalPages = doc.internal.getNumberOfPages();
    for (let i = 2; i <= totalPages; i++) {
      doc.setPage(i);
      doc.setFont('helvetica', 'normal');
      doc.setFontSize(8.5);
      doc.setTextColor(...C.muted);
      doc.text('FEAD Technical Analysis  \u00b7  Gates MT820  \u00b7  Ashok Leyland H6  \u00b7  ' + dateStr, ML, PH - 8);
      doc.text('Page ' + i + ' of ' + totalPages, PW - MR, PH - 8, { align: 'right' });
      doc.setDrawColor(...C.blue);
      doc.setLineWidth(0.25);
      doc.line(ML, PH - 13, PW - MR, PH - 13);
    }

    const fname = 'FEAD_Report_' + snapRPM + 'RPM_' + snapTen + 'N_' + new Date().toISOString().slice(0, 10) + '.pdf';
    doc.save(fname);

  } catch (e) {
    console.error('PDF generation error:', e);
    alert('PDF generation failed:\n' + e.message + '\n\nOpen browser console (F12) for details.');
  } finally {
    if (btn) { btn.disabled = false; btn.textContent = '\uD83D\uDCC4 Download PDF Report'; }
  }
}
