"""Embedded HTML page for the admin dashboard (single file, no assets).

Apple "liquid glass" design: frosted translucent cards (backdrop blur) over a
soft aurora gradient, pill buttons, system font stack. Follows the device
light/dark setting automatically and is fully responsive for phones.

Screens (glass sidebar on desktop, bottom tab bar on phones):
  Control - live status, mode switch, e-stop, return-to-base, virtual joystick
  Map     - live SLAM map with robot pose + violation target, tap-and-drag
            relocalise (publishes /initialpose), clear-costmaps recovery
  History - violation statistics and metadata-only incident table
"""

PAGE_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="theme-color" media="(prefers-color-scheme: light)" content="#eef1f8">
<meta name="theme-color" media="(prefers-color-scheme: dark)" content="#0b0e14">
<title>Compliance Robot</title>
<style>
  :root {
    --bg1:#dfe7ff; --bg2:#ffe9f3; --bg3:#e3fff3; --base:#eef1f8;
    --glass:rgba(255,255,255,.58); --glass2:rgba(255,255,255,.42);
    --stroke:rgba(255,255,255,.65); --hairline:rgba(60,60,67,.12);
    --txt:#1c1c1e; --dim:rgba(60,60,67,.6);
    --blue:#007aff; --green:#34c759; --orange:#ff9500; --red:#ff3b30;
    --knob:rgba(255,255,255,.85); --shadow:0 8px 32px rgba(31,38,60,.12);
    --mapbg:rgba(255,255,255,.35);
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --bg1:#1b2347; --bg2:#3a1d3f; --bg3:#0f2e33; --base:#0b0e14;
      --glass:rgba(28,32,44,.55); --glass2:rgba(28,32,44,.4);
      --stroke:rgba(255,255,255,.14); --hairline:rgba(255,255,255,.1);
      --txt:#f2f3f7; --dim:rgba(235,235,245,.55);
      --blue:#0a84ff; --green:#30d158; --orange:#ff9f0a; --red:#ff453a;
      --knob:rgba(255,255,255,.22); --shadow:0 8px 32px rgba(0,0,0,.45);
      --mapbg:rgba(255,255,255,.07);
    }
  }
  * { box-sizing:border-box; margin:0; min-width:0;
      -webkit-tap-highlight-color:transparent; }
  html { font-size:16px; }
  body {
    min-height:100vh; color:var(--txt); background:var(--base);
    overflow-x:hidden; max-width:100vw;
    font-family:-apple-system,BlinkMacSystemFont,'SF Pro Display','SF Pro Text',
                'Segoe UI',Roboto,'Helvetica Neue',sans-serif;
  }
  body::before {
    content:''; position:fixed; inset:-20%; z-index:-1;
    background:
      radial-gradient(40% 50% at 15% 15%, var(--bg1) 0%, transparent 70%),
      radial-gradient(45% 55% at 85% 20%, var(--bg2) 0%, transparent 70%),
      radial-gradient(50% 60% at 50% 95%, var(--bg3) 0%, transparent 70%),
      var(--base);
    filter:blur(40px); animation:drift 24s ease-in-out infinite alternate;
  }
  @keyframes drift {
    from { transform:translate(0,0) scale(1); }
    to   { transform:translate(2%,3%) scale(1.06); }
  }

  .glass {
    background:var(--glass);
    -webkit-backdrop-filter:blur(28px) saturate(180%);
    backdrop-filter:blur(28px) saturate(180%);
    border:1px solid var(--stroke); border-radius:24px;
    box-shadow:var(--shadow);
  }

  /* ---------- app shell: sidebar + content ---------- */
  #app { display:flex; min-height:100vh; }
  aside {
    width:218px; flex:none; margin:14px 0 14px 14px; padding:18px 12px;
    display:flex; flex-direction:column; gap:6px;
    position:sticky; top:14px; height:calc(100vh - 28px);
    border-radius:26px;
  }
  aside .brand { display:flex; align-items:center; gap:10px;
                 padding:6px 10px 16px; }
  aside .brand .dot { width:36px; height:36px; border-radius:11px; flex:none;
    background:linear-gradient(135deg,var(--blue),#5e5ce6);
    display:flex; align-items:center; justify-content:center;
    color:#fff; font-size:19px; }
  aside .brand h1 { font-size:15px; font-weight:700; letter-spacing:-.02em; }
  aside .brand p { font-size:11px; color:var(--dim); }
  .navbtn {
    display:flex; align-items:center; gap:12px; width:100%;
    padding:12px 14px; border-radius:14px; border:none; background:none;
    font-family:inherit; font-size:14.5px; font-weight:600; color:var(--dim);
    cursor:pointer; text-align:left; transition:background .15s;
  }
  .navbtn .ic { font-size:18px; width:24px; text-align:center; }
  .navbtn.active { background:var(--glass2); color:var(--txt);
                   border:1px solid var(--stroke); }
  aside .spacer { flex:1; }
  aside .modechip { text-align:center; }

  main { flex:1; padding:14px; max-width:1100px;
         padding-bottom:max(20px, env(safe-area-inset-bottom)); }
  .topbar { display:flex; align-items:center; gap:12px; padding:14px 20px;
            margin-bottom:12px; }
  .topbar h2 { font-size:17px; font-weight:700; letter-spacing:-.02em; flex:1; }

  .badge { padding:6px 14px; border-radius:100px; font-size:13px;
           font-weight:600; letter-spacing:.01em; display:inline-block; }
  .b-auto   { background:rgba(52,199,89,.16); color:var(--green); }
  .b-manual { background:rgba(255,149,0,.18); color:var(--orange); }

  .grid { display:grid; gap:10px;
          grid-template-columns:repeat(auto-fit,minmax(min(150px,100%),1fr)); }
  .stat { padding:16px 18px; border-radius:20px; }
  .stat .label { font-size:12px; color:var(--dim); font-weight:500;
                 margin-bottom:5px; }
  .stat .value { font-size:22px; font-weight:700; letter-spacing:-.02em; }
  .stat .value.small { font-size:17px; }
  .ok { color:var(--green); } .off { color:var(--dim); }

  .row { display:grid; gap:12px; margin-top:12px;
         grid-template-columns:repeat(auto-fit,minmax(min(310px,100%),1fr)); }
  .panel { padding:20px; }
  .panel h2 { font-size:13px; font-weight:600; color:var(--dim);
              text-transform:uppercase; letter-spacing:.06em;
              margin-bottom:14px; }

  button {
    font-family:inherit; font-size:16px; font-weight:600; color:var(--txt);
    background:var(--glass2); border:1px solid var(--stroke);
    border-radius:100px; padding:14px 20px; cursor:pointer;
    min-height:48px; transition:transform .12s ease, filter .15s ease;
    -webkit-backdrop-filter:blur(10px); backdrop-filter:blur(10px);
  }
  button:active { transform:scale(.96); filter:brightness(1.08); }
  button.primary { background:var(--blue); border-color:transparent; color:#fff;
                   box-shadow:0 6px 20px rgba(10,132,255,.35); }
  button.danger  { background:var(--red); border-color:transparent; color:#fff;
                   box-shadow:0 6px 20px rgba(255,69,58,.35); }
  button.warn.on { background:var(--orange); border-color:transparent;
                   color:#fff; box-shadow:0 6px 20px rgba(255,149,0,.35); }
  .btnrow { display:flex; flex-direction:column; gap:10px; }
  .btnrow button { width:100%; }
  .note { font-size:12.5px; color:var(--dim); margin-top:12px; line-height:1.5; }

  /* Virtual joystick */
  .stickwrap { display:flex; flex-direction:column; align-items:center; }
  #stick {
    width:200px; height:200px; border-radius:50%; position:relative;
    background:var(--glass2); border:1px solid var(--stroke);
    -webkit-backdrop-filter:blur(16px); backdrop-filter:blur(16px);
    touch-action:none; box-shadow:inset 0 2px 14px rgba(0,0,0,.08);
  }
  #stick::before { content:''; position:absolute; inset:14px;
    border-radius:50%; border:1.5px dashed var(--hairline); }
  #knob {
    width:84px; height:84px; border-radius:50%; position:absolute;
    left:58px; top:58px; background:var(--knob);
    border:1px solid var(--stroke);
    box-shadow:0 6px 18px rgba(0,0,0,.22);
    -webkit-backdrop-filter:blur(8px); backdrop-filter:blur(8px);
    transition:left .25s cubic-bezier(.3,1.4,.5,1), top .25s cubic-bezier(.3,1.4,.5,1);
    display:flex; align-items:center; justify-content:center;
    color:var(--dim); font-size:24px; user-select:none;
  }
  #knob.live { transition:none; }

  /* Map screen */
  #mapbox { position:relative; border-radius:18px; overflow:hidden;
            background:var(--mapbg); touch-action:none; }
  #mapcanvas { display:block; width:100%; }
  #maphint { position:absolute; left:0; right:0; top:10px; text-align:center;
             pointer-events:none; }
  #maphint span { background:var(--glass); border:1px solid var(--stroke);
    border-radius:100px; padding:7px 16px; font-size:13px; font-weight:600;
    -webkit-backdrop-filter:blur(14px); backdrop-filter:blur(14px); }
  .maptools { display:flex; gap:10px; margin-top:12px; flex-wrap:wrap; }
  .maptools button { flex:1; min-width:150px; font-size:14.5px; }
  #toast { position:fixed; left:50%; bottom:90px; transform:translateX(-50%);
    background:var(--glass); border:1px solid var(--stroke);
    -webkit-backdrop-filter:blur(20px); backdrop-filter:blur(20px);
    border-radius:100px; padding:11px 20px; font-size:14px; font-weight:600;
    box-shadow:var(--shadow); opacity:0; transition:opacity .25s;
    pointer-events:none; z-index:60; max-width:90vw; text-align:center; }
  #toast.show { opacity:1; }

  .tablewrap { overflow-x:auto; -webkit-overflow-scrolling:touch;
               border-radius:16px; }
  table { width:100%; border-collapse:collapse; font-size:13.5px;
          min-width:620px; }
  th, td { text-align:left; padding:10px 12px;
           border-bottom:1px solid var(--hairline); white-space:nowrap; }
  th { color:var(--dim); font-weight:600; font-size:12px;
       text-transform:uppercase; letter-spacing:.05em; }
  tr:last-child td { border-bottom:none; }
  td.outcome { font-weight:600; }
  tr.complied td.outcome { color:var(--green); }
  tr.failed   td.outcome { color:var(--red); }

  select {
    font-family:inherit; font-size:14px; color:var(--txt);
    background:var(--glass2); border:1px solid var(--stroke);
    border-radius:100px; padding:8px 14px;
    -webkit-backdrop-filter:blur(10px); backdrop-filter:blur(10px);
  }
  .filterbar { display:flex; align-items:center; justify-content:space-between;
               margin-bottom:14px; gap:10px; flex-wrap:wrap; }

  /* Login */
  #login { max-width:380px; margin:14vh auto 0; padding:30px 26px;
           text-align:center; }
  #login .dot { width:64px; height:64px; border-radius:20px; margin:0 auto 16px;
    background:linear-gradient(135deg,var(--blue),#5e5ce6);
    display:flex; align-items:center; justify-content:center;
    color:#fff; font-size:34px; }
  #login h2 { font-size:21px; font-weight:700; letter-spacing:-.02em; }
  #login p  { font-size:13px; color:var(--dim); margin-top:6px; }
  #login input {
    width:100%; margin:20px 0 6px; padding:15px 18px; font-size:16px;
    font-family:inherit; color:var(--txt);
    background:var(--glass2); border:1px solid var(--stroke);
    border-radius:16px; outline:none;
    -webkit-backdrop-filter:blur(10px); backdrop-filter:blur(10px);
  }
  #login input:focus { border-color:var(--blue); }
  #loginErr { color:var(--red); font-size:13px; min-height:18px; }
  #login button { width:100%; margin-top:8px; }

  .hidden { display:none !important; }
  .view { display:none; }
  .view.active { display:block; }
  .section { margin-top:12px; padding:20px; }

  /* Camera feeds */
  .cam-grid { display:grid; gap:12px; margin-top:12px;
              grid-template-columns:repeat(auto-fit,minmax(min(340px,100%),1fr)); }
  .cam-card { padding:18px; }
  .cam-header { display:flex; align-items:center; justify-content:space-between;
                margin-bottom:12px; }
  .cam-header h2 { font-size:13px; font-weight:600; color:var(--dim);
                   text-transform:uppercase; letter-spacing:.06em; margin:0; }
  .cam-dot { width:9px; height:9px; border-radius:50%; flex:none;
             background:var(--dim); transition:background .4s; }
  .cam-dot.on  { background:var(--green);
                 box-shadow:0 0 6px rgba(52,199,89,.7); }
  .cam-dot.off { background:var(--red); }
  .cam-status { font-size:12px; font-weight:600; color:var(--dim);
                display:flex; align-items:center; gap:6px; }
  .cam-status.on  { color:var(--green); }
  .cam-status.off { color:var(--red); }
  .cam-feed { width:100%; border-radius:14px; background:#000;
              aspect-ratio:16/9; object-fit:contain; display:block;
              border:1px solid var(--hairline); }

  /* Bottom tab bar on phones */
  #tabbar { display:none; }
  @media (max-width:760px) {
    aside { display:none; }
    main { padding:10px 10px 92px; }
    #tabbar {
      display:flex; position:fixed; left:10px; right:10px;
      bottom:max(10px, env(safe-area-inset-bottom)); z-index:50;
      border-radius:24px; padding:6px;
    }
    #tabbar button {
      flex:1; display:flex; flex-direction:column; align-items:center; gap:2px;
      border:none; background:none; border-radius:18px; padding:8px 4px;
      font-size:11px; font-weight:600; color:var(--dim); min-height:54px;
    }
    #tabbar button .ic { font-size:21px; }
    #tabbar button.active { background:var(--glass2); color:var(--txt); }
    html { font-size:15px; }
    .grid { grid-template-columns:repeat(2,1fr); }
    .stat { padding:13px 14px; }
    .stat .value { font-size:19px; }
    .topbar { padding:12px 14px; }
    .panel { padding:16px; }
    .cam-grid { grid-template-columns:1fr; }
  }
</style>
</head>
<body>

<div id="login" class="glass">
  <div class="dot">&#129302;</div>
  <h2>Compliance Robot</h2>
  <p>Admin console &middot; sign in to continue</p>
  <input id="pw" type="password" placeholder="Admin password" autocomplete="current-password"
         onkeydown="if(event.key==='Enter')login()">
  <div id="loginErr"></div>
  <button class="primary" onclick="login()">Sign In</button>
  <p class="note">Privacy: this console shows violation metadata only.<br>
     No video or photographs are accessible here.</p>
</div>

<div id="app" class="hidden">
  <aside class="glass">
    <div class="brand">
      <div class="dot">&#129302;</div>
      <div><h1>Compliance Robot</h1><p>Admin console</p></div>
    </div>
    <button class="navbtn active" data-view="control" onclick="go('control')">
      <span class="ic">&#127918;</span> Control</button>
    <button class="navbtn" data-view="cameras" onclick="go('cameras')">
      <span class="ic">&#128247;</span> Cameras</button>
    <button class="navbtn" data-view="map" onclick="go('map')">
      <span class="ic">&#128506;</span> Map</button>
    <button class="navbtn" data-view="history" onclick="go('history')">
      <span class="ic">&#128203;</span> History</button>
    <div class="spacer"></div>
    <div class="modechip"><span id="modeSide" class="badge b-auto">&mdash;</span></div>
  </aside>

  <main>
    <div class="topbar glass">
      <h2 id="viewTitle">Control</h2>
      <span id="mode" class="badge b-auto">&mdash;</span>
    </div>

    <!-- ================= CONTROL ================= -->
    <div id="view-control" class="view active">
      <div class="grid">
        <div class="stat glass"><div class="label">Robot activity</div>
          <div class="value small" id="fsm">&mdash;</div></div>
        <div class="stat glass"><div class="label">Active room</div>
          <div class="value small" id="room">&mdash;</div></div>
        <div class="stat glass"><div class="label">Joystick</div>
          <div class="value small" id="joy">&mdash;</div></div>
        <div class="stat glass"><div class="label">Violations 24 h</div>
          <div class="value" id="v24">&mdash;</div></div>
        <div class="stat glass"><div class="label">Compliance rate</div>
          <div class="value" id="crate">&mdash;</div></div>
      </div>

      <div class="row">
        <div class="panel glass">
          <h2>Robot Control</h2>
          <div class="btnrow">
            <button id="modeBtn" class="primary" onclick="toggleMode()">&mdash;</button>
            <button onclick="goHome()">&#8962;&nbsp; Return to Base</button>
            <button class="danger" onclick="estop()">&#9632;&nbsp; Emergency Stop</button>
          </div>
          <p class="note" id="modeNote">&mdash;</p>
        </div>

        <div class="panel glass">
          <h2>Manual Drive</h2>
          <div class="stickwrap">
            <div id="stick"><div id="knob">&#10021;</div></div>
          </div>
          <p class="note" style="text-align:center">
            Drag to drive &mdash; up/down moves, left/right turns.<br>
            Bluetooth joystick (L2 + stick) always overrides.</p>
        </div>
      </div>
    </div>

    <!-- ================= CAMERAS ================= -->
    <div id="view-cameras" class="view">
      <div class="cam-grid">

        <div class="cam-card glass">
          <div class="cam-header">
            <h2>&#128249;&nbsp; CCTV &mdash; Room Camera (Laptop)</h2>
            <div class="cam-status off" id="cctv-status">
              <div class="cam-dot off" id="cctv-dot"></div>
              <span id="cctv-label">Disconnected</span>
            </div>
          </div>
          <img id="cctv-img" class="cam-feed" alt="CCTV feed" src="">
        </div>

        <div class="cam-card glass">
          <div class="cam-header">
            <h2>&#129302;&nbsp; Robot Camera (Pi HQ)</h2>
            <div class="cam-status off" id="robot-status">
              <div class="cam-dot off" id="robot-dot"></div>
              <span id="robot-label">Disconnected</span>
            </div>
          </div>
          <img id="robot-img" class="cam-feed" alt="Robot camera" src="">
        </div>

      </div>
    </div>

    <!-- ================= MAP ================= -->
    <div id="view-map" class="view">
      <div class="panel glass">
        <h2>Live Map</h2>
        <div id="mapbox">
          <canvas id="mapcanvas"></canvas>
          <div id="maphint" class="hidden"><span>Tap the robot's true position,
            then drag towards where it is facing</span></div>
        </div>
        <div class="maptools">
          <button id="relocBtn" class="warn" onclick="toggleReloc()">
            &#10166;&nbsp; Relocalise</button>
          <button onclick="clearCostmaps()">&#129529;&nbsp; Clear costmaps</button>
        </div>
        <p class="note">Blue arrow = robot. Red dot = latest violation target.
          Relocalise tells the localisation system where the robot really is
          (use it when the robot is lost on the map). On the real robot this
          feeds AMCL; in simulation mapping mode SLAM localises itself.</p>
      </div>
    </div>

    <!-- ================= HISTORY ================= -->
    <div id="view-history" class="view">
      <div class="section glass" style="margin-top:0">
        <div class="filterbar">
          <h2 style="margin:0">Violation History</h2>
          <select id="roomFilter" onchange="renderTable()">
            <option value="">All rooms</option>
          </select>
        </div>
        <div class="grid" id="roomStats" style="margin-bottom:14px"></div>
        <div class="tablewrap">
          <table>
            <thead><tr><th>Time</th><th>Room</th><th>Type</th><th>Stage</th>
                       <th>Outcome</th><th>Conf.</th><th>Location</th></tr></thead>
            <tbody id="tbody"></tbody>
          </table>
        </div>
        <p class="note">Metadata-only log (privacy by design). Track IDs are
           anonymous and reset on robot restart.</p>
      </div>
    </div>
  </main>

  <nav id="tabbar" class="glass">
    <button class="active" data-view="control" onclick="go('control')">
      <span class="ic">&#127918;</span>Control</button>
    <button data-view="cameras" onclick="go('cameras')">
      <span class="ic">&#128247;</span>Cameras</button>
    <button data-view="map" onclick="go('map')">
      <span class="ic">&#128506;</span>Map</button>
    <button data-view="history" onclick="go('history')">
      <span class="ic">&#128203;</span>History</button>
  </nav>
</div>

<div id="toast"></div>

<script>
let token = localStorage.getItem('crtoken') || '';
let incidents = [];

async function api(path, body) {
  const opts = { headers: { 'X-Auth': token } };
  if (body !== undefined) {
    opts.method = 'POST';
    opts.body = JSON.stringify(body);
  }
  const r = await fetch(path, opts);
  if (r.status === 401) { showLogin(); throw new Error('auth'); }
  return r.json();
}

function toast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  clearTimeout(t._h);
  t._h = setTimeout(() => t.classList.remove('show'), 2600);
}

function showLogin() {
  document.getElementById('login').classList.remove('hidden');
  document.getElementById('app').classList.add('hidden');
}
function showApp() {
  document.getElementById('login').classList.add('hidden');
  document.getElementById('app').classList.remove('hidden');
}

async function login(pwArg) {
  const pw = pwArg !== undefined ? pwArg : document.getElementById('pw').value;
  const r = await fetch('/api/login', { method:'POST',
      body: JSON.stringify({ password: pw }) });
  if (r.ok) {
    token = (await r.json()).token;
    localStorage.setItem('crtoken', token);
    showApp(); refresh(); loadIncidents();
  } else {
    document.getElementById('loginErr').textContent = 'Wrong password';
  }
}

/* ------------- navigation ------------- */
const TITLES = { control:'Control', cameras:'Cameras',
                 map:'Live Map', history:'Violation History' };
function go(view) {
  document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
  document.getElementById('view-' + view).classList.add('active');
  document.querySelectorAll('[data-view]').forEach(b =>
    b.classList.toggle('active', b.dataset.view === view));
  document.getElementById('viewTitle').textContent = TITLES[view];
  if (view === 'map') { mapVisible = true; loadMapInfo(true); }
  else mapVisible = false;
  camVisible = (view === 'cameras');
}

/* ------------- status ------------- */
let paused = false;
async function refresh() {
  try {
    const s = await api('/api/status');
    paused = s.mode === 'MANUAL';
    for (const id of ['mode', 'modeSide']) {
      const el = document.getElementById(id);
      el.textContent = paused ? 'Admin Control' : 'Autonomous';
      el.className = 'badge ' + (paused ? 'b-manual' : 'b-auto');
    }
    document.getElementById('fsm').textContent = s.fsm_state;
    document.getElementById('room').textContent = s.room;
    const joy = document.getElementById('joy');
    joy.textContent = s.joystick ? 'Connected' : 'Not seen';
    joy.className = 'value small ' + (s.joystick ? 'ok' : 'off');
    document.getElementById('modeBtn').textContent =
      paused ? '▶︎  Resume Autonomy' : '⏸︎  Take Admin Control';
    document.getElementById('modeNote').textContent = paused
      ? 'Robot is under admin control: patrol and escalation are paused. ' +
        'Drive with the joystick or the pad, then resume autonomy.'
      : 'Robot is doing its job: patrolling and responding to violations. ' +
        'Take admin control to drive it manually.';
  } catch (e) { /* logged out */ }
}

/* ------------- incidents ------------- */
async function loadIncidents() {
  try {
    const d = await api('/api/incidents');
    incidents = d.incidents;
    document.getElementById('v24').textContent = d.stats.last24h;
    document.getElementById('crate').textContent =
        d.stats.total ? d.stats.compliance_rate + '%' : '—';
    const rs = document.getElementById('roomStats');
    rs.innerHTML = '';
    const filt = document.getElementById('roomFilter');
    const current = filt.value;
    filt.innerHTML = '<option value="">All rooms</option>';
    Object.entries(d.stats.per_room).forEach(([room, n]) => {
      rs.innerHTML += `<div class="stat glass"><div class="label">${room}</div>
                       <div class="value">${n}</div></div>`;
      filt.innerHTML += `<option value="${room}">${room}</option>`;
    });
    rs.innerHTML += `<div class="stat glass"><div class="label">total</div>
                     <div class="value">${d.stats.total}</div></div>`;
    filt.value = current;
    renderTable();
  } catch (e) { /* logged out */ }
}

function renderTable() {
  const room = document.getElementById('roomFilter').value;
  const rows = incidents
    .filter(i => !room || (i.room || i.room_id) === room)
    .slice(0, 200)
    .map(i => {
      const t = (i.timestamp || '').replace('T', ' ').slice(0, 19);
      const loc = i.approx_location && i.approx_location.x != null
        ? `(${i.approx_location.x}, ${i.approx_location.y})` : '—';
      const cls = i.outcome === 'complied' ? 'complied'
                : i.outcome === 'logged_no_compliance' ? 'failed' : '';
      return `<tr class="${cls}"><td>${t}</td><td>${i.room || i.room_id || '—'}</td>
        <td>${i.event_class || '—'}</td><td>${i.stage_reached || '—'}</td>
        <td class="outcome">${(i.outcome || '—').replaceAll('_',' ')}</td>
        <td>${i.confidence != null ? i.confidence : '—'}</td><td>${loc}</td></tr>`;
    });
  document.getElementById('tbody').innerHTML =
    rows.join('') || '<tr><td colspan="7">No violations recorded.</td></tr>';
}

async function toggleMode() { await api('/api/mode', { paused: !paused }); refresh(); }
async function goHome()    { await api('/api/home', {}); toast('Returning to base'); }
async function estop()     { await api('/api/stop', {}); refresh();
                             toast('EMERGENCY STOP - autonomy paused'); }

/* ------------- virtual joystick ------------- */
const stick = document.getElementById('stick');
const knob = document.getElementById('knob');
let dragging = false, sendTimer = null, cur = { lx: 0, az: 0 };

function setKnob(dx, dy) {
  knob.style.left = (58 + dx) + 'px';
  knob.style.top  = (58 + dy) + 'px';
}
function stickMove(e) {
  const r = stick.getBoundingClientRect();
  const R = r.width / 2 - 42;
  let dx = e.clientX - (r.left + r.width / 2);
  let dy = e.clientY - (r.top + r.height / 2);
  const d = Math.hypot(dx, dy);
  if (d > R) { dx *= R / d; dy *= R / d; }
  setKnob(dx, dy);
  cur = { lx: +(-dy / R).toFixed(2), az: +(-dx / R).toFixed(2) };
}
stick.addEventListener('pointerdown', e => {
  dragging = true;
  knob.classList.add('live');
  stick.setPointerCapture(e.pointerId);
  stickMove(e);
  api('/api/drive', cur);
  sendTimer = setInterval(() => api('/api/drive', cur), 180);
});
stick.addEventListener('pointermove', e => { if (dragging) stickMove(e); });
function stickEnd() {
  if (!dragging) return;
  dragging = false;
  clearInterval(sendTimer);
  knob.classList.remove('live');
  setKnob(0, 0);
  cur = { lx: 0, az: 0 };
  api('/api/drive', cur);
}
stick.addEventListener('pointerup', stickEnd);
stick.addEventListener('pointercancel', stickEnd);

/* ------------- map ------------- */
const canvas = document.getElementById('mapcanvas');
const ctx = canvas.getContext('2d');
const mapImg = new Image();
let mapVisible = false, mapInfo = null, mapStamp = -1, imgReady = false;
let relocMode = false, relocStart = null, relocDrag = null;

function worldToCanvas(wx, wy) {
  const s = canvas.width / mapImg.width;
  const px = (wx - mapInfo.origin_x) / mapInfo.resolution;
  const py = mapInfo.height - ((wy - mapInfo.origin_y) / mapInfo.resolution);
  return [px * s, py * s];
}
function canvasToWorld(cx, cy) {
  const s = canvas.width / mapImg.width;
  const wx = (cx / s) * mapInfo.resolution + mapInfo.origin_x;
  const wy = (mapInfo.height - cy / s) * mapInfo.resolution + mapInfo.origin_y;
  return [wx, wy];
}

async function loadMapInfo(force) {
  if (!mapVisible && !force) return;
  try {
    mapInfo = await api('/api/map_info');
    if (!mapInfo.has_map) { drawNoMap(); return; }
    if (mapInfo.stamp !== mapStamp) {
      // <img> cannot send the auth header: fetch the PNG and use a blob URL
      const r = await fetch('/api/map.png?t=' + Date.now(),
                            { headers: { 'X-Auth': token } });
      if (!r.ok) return;
      const url = URL.createObjectURL(await r.blob());
      mapImg.onload = () => {
        imgReady = true;
        URL.revokeObjectURL(url);
        drawMap();
      };
      mapImg.src = url;
      mapStamp = mapInfo.stamp;
    } else if (imgReady) drawMap();
  } catch (e) { /* logged out */ }
}

function fitCanvas() {
  const box = document.getElementById('mapbox');
  const w = box.clientWidth;
  if (!imgReady || !w) return false;
  const h = Math.round(w * mapImg.height / mapImg.width);
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w; canvas.height = h;
  }
  return true;
}

function drawNoMap() {
  const box = document.getElementById('mapbox');
  canvas.width = box.clientWidth || 600;
  canvas.height = 240;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = getComputedStyle(document.body).getPropertyValue('color');
  ctx.globalAlpha = 0.5;
  ctx.font = '14px -apple-system, sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText('Waiting for the map (SLAM starting up)...',
               canvas.width / 2, 120);
  ctx.globalAlpha = 1;
}

function drawMap() {
  if (!fitCanvas()) return;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImg, 0, 0, canvas.width, canvas.height);

  if (mapInfo.target) {
    const [tx, ty] = worldToCanvas(mapInfo.target[0], mapInfo.target[1]);
    ctx.fillStyle = 'rgba(255,69,58,.9)';
    ctx.beginPath(); ctx.arc(tx, ty, 7, 0, 7); ctx.fill();
  }
  if (mapInfo.robot) {
    const [rx, ry] = worldToCanvas(mapInfo.robot.x, mapInfo.robot.y);
    drawArrow(rx, ry, -mapInfo.robot.yaw, '#0a84ff');
  }
  if (relocStart && relocDrag) {
    const ang = Math.atan2(relocDrag[1] - relocStart[1],
                           relocDrag[0] - relocStart[0]);
    drawArrow(relocStart[0], relocStart[1], ang, '#ff9f0a');
  }
}

function drawArrow(x, y, ang, color) {
  ctx.save();
  ctx.translate(x, y); ctx.rotate(ang);
  ctx.fillStyle = color;
  ctx.strokeStyle = 'rgba(255,255,255,.9)'; ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(13, 0); ctx.lineTo(-8, -8); ctx.lineTo(-4, 0); ctx.lineTo(-8, 8);
  ctx.closePath(); ctx.fill(); ctx.stroke();
  ctx.restore();
}

function toggleReloc(forceOff) {
  relocMode = forceOff ? false : !relocMode;
  relocStart = relocDrag = null;
  document.getElementById('relocBtn').classList.toggle('on', relocMode);
  document.getElementById('maphint').classList.toggle('hidden', !relocMode);
  if (imgReady) drawMap();
}

function canvasPoint(e) {
  const r = canvas.getBoundingClientRect();
  return [(e.clientX - r.left) * canvas.width / r.width,
          (e.clientY - r.top) * canvas.height / r.height];
}
canvas.addEventListener('pointerdown', e => {
  if (!relocMode || !imgReady) return;
  relocStart = canvasPoint(e);
  relocDrag = relocStart;
  canvas.setPointerCapture(e.pointerId);
  drawMap();
});
canvas.addEventListener('pointermove', e => {
  if (!relocMode || !relocStart) return;
  relocDrag = canvasPoint(e);
  drawMap();
});
canvas.addEventListener('pointerup', async e => {
  if (!relocMode || !relocStart) return;
  const end = canvasPoint(e);
  const [wx, wy] = canvasToWorld(relocStart[0], relocStart[1]);
  // canvas y grows downward; world yaw grows counter-clockwise
  const yaw = Math.atan2(-(end[1] - relocStart[1]), end[0] - relocStart[0]);
  toggleReloc(true);
  const r = await api('/api/relocalise', { x: wx, y: wy, yaw: yaw });
  toast(r.note || `Relocalised to (${wx.toFixed(2)}, ${wy.toFixed(2)})`);
});

async function clearCostmaps() {
  const r = await api('/api/clear_costmaps', {});
  toast(r.cleared ? `Costmaps cleared (${r.cleared})`
                  : 'Costmap services not available');
}

window.addEventListener('resize', () => { if (imgReady && mapVisible) drawMap(); });

/* ------------- camera feeds ------------- */
let camVisible = false;

function setCamStatus(id, online) {
  const dot = document.getElementById(id + '-dot');
  const lbl = document.getElementById(id + '-label');
  const bar = document.getElementById(id + '-status');
  dot.className = 'cam-dot ' + (online ? 'on' : 'off');
  bar.className = 'cam-status ' + (online ? 'on' : 'off');
  lbl.textContent = online ? 'Connected' : 'Disconnected';
}

async function loadCamFrame(imgId, path) {
  // <img> cannot send the auth header: fetch the JPEG and use a blob URL
  try {
    const r = await fetch(path + '?t=' + Date.now(),
                          { headers: { 'X-Auth': token } });
    if (!r.ok) return;
    const url = URL.createObjectURL(await r.blob());
    const img = document.getElementById(imgId);
    const old = img.dataset.blobUrl;
    img.onload = () => { if (old) URL.revokeObjectURL(old); };
    img.dataset.blobUrl = url;
    img.src = url;
  } catch (e) { /* ignore */ }
}

async function refreshCams() {
  if (!camVisible) return;
  try {
    const s = await api('/api/cam/status');
    setCamStatus('cctv', s.cctv);
    setCamStatus('robot', s.robot);
  } catch (e) { /* ignore */ }
  loadCamFrame('cctv-img', '/api/cam/cctv.jpg');
  loadCamFrame('robot-img', '/api/cam/robot.jpg');
}

setInterval(refresh, 2000);
setInterval(loadIncidents, 5000);
setInterval(() => loadMapInfo(false), 1000);
setInterval(refreshCams, 350);

const hashParams = new URLSearchParams(location.hash.slice(1));
const startView = hashParams.get('view');
if (startView && TITLES[startView]) go(startView);
if (hashParams.get('pw') !== null) {
  login(hashParams.get('pw'));
} else if (token) {
  showApp(); refresh(); loadIncidents();
} else {
  showLogin();
}
</script>
</body>
</html>
"""
