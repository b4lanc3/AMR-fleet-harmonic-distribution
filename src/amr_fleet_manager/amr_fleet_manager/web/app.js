/* AMR Fleet dashboard - vanilla JS, no external dependencies. */
'use strict';

const STATE_VI = {
  OFFLINE: 'Mất kết nối', IDLE: 'Rảnh', PARKED: 'Đang đỗ', TO_PARK: 'Về bãi đỗ',
  TO_PICKUP: 'Đi lấy hàng', LOADING: 'Đang nhận hàng', TO_DROPOFF: 'Đang chở hàng',
  UNLOADING: 'Đang trả hàng', TO_CHARGER: 'Đi sạc', CHARGING: 'Đang sạc',
  WAIT_CHARGER: 'Chờ dock sạc', YIELDING: 'Nhường đường', DEMO: 'Demo đối đầu',
};
// [status role, icon]  - status colours always come with an icon + label
const STATE_BADGE = {
  OFFLINE: ['critical', '✕'], IDLE: ['info', '○'], PARKED: ['info', 'P'], TO_PARK: ['info', '↩'],
  TO_PICKUP: ['info', '→'], LOADING: ['good', '⇩'], TO_DROPOFF: ['good', '▶'],
  UNLOADING: ['good', '⇧'], TO_CHARGER: ['serious', '⚡'], CHARGING: ['good', '⚡'],
  WAIT_CHARGER: ['warning', '⏳'], YIELDING: ['warning', '⤳'], DEMO: ['serious', '⇄'],
};
const LINE_BADGE = {
  RUNNING: ['good', '▶', 'Đang chạy'], STARVED: ['warning', '◌', 'Thiếu đầu vào'],
  BLOCKED: ['critical', '■', 'Nghẽn đầu ra'], IDLE: ['info', '○', 'Lưu kho'],
};
const ZONE_VI = {
  A: 'Khu A · Kho linh kiện', B: 'Khu B · Dây chuyền sản xuất', C: 'Khu C · Dây chuyền đóng gói',
  D: 'Khu D · Kho thành phẩm', CHG: 'Trạm sạc (5 dock)', PK: 'Bãi đỗ', PK2: 'Bãi đỗ',
};
// where the zone names are drawn (free floor between racks / lines), metres
const LABEL_AT = { A: [20.8, 53.25], B: [74.5, 49.0], C: [74.5, 20.5], D: [20.8, 22.75], CHG: [72.0, 37.6] };
const TASK_VI = { KIT_DELIVERY: 'Linh kiện', BOX_TRANSFER: 'Thùng SP', FG_DELIVERY: 'Thành phẩm', MANUAL: 'Thủ công' };
const TASK_STATUS_VI = {
  QUEUED: 'Chờ robot', NEXT: 'Đã xếp (task kế)', ASSIGNED: 'Đã giao', TO_PICKUP: 'Đi lấy',
  LOADING: 'Nhận hàng', IN_TRANSIT: 'Đang chở', UNLOADING: 'Trả hàng',
};
const CAT_VI = { YIELD: 'Né đường', TRAFFIC: 'Giao thông', TASK: 'Task', CHARGE: 'Sạc pin', SYSTEM: 'Hệ thống', PRODUCTION: 'Sản xuất' };

let layout = null, graph = null, state = null, selected = null;
const robotIndex = {};
const poses = {};          // robot -> {from, to, t0, t1} for smooth animation
let staticLayer = null, staticKey = '';

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const rname = (id) => (id || '').toUpperCase().replace('_', '-');
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const fmtS = (s) => {
  if (s == null || !isFinite(s)) return '—';
  s = Math.max(0, Math.round(s));
  return s >= 60 ? `${Math.floor(s / 60)}p ${String(s % 60).padStart(2, '0')}s` : `${s}s`;
};
function robotSlot(id) {
  if (!(id in robotIndex)) robotIndex[id] = Object.keys(robotIndex).length;
  return robotIndex[id];
}
// 8 categorical slots in fixed order; robots 9..16 reuse a slot with a hollow marker
// (composite encoding) and every robot is always labelled with its id.
function robotColor(id) { return css(`--s${(robotSlot(id) % 8) + 1}`); }
function robotHollow(id) { return robotSlot(id) >= 8; }
function dotHtml(id) {
  const c = robotColor(id);
  return robotHollow(id) ? `<i class="dot hollow" style="border-color:${c}"></i>` : `<i class="dot" style="background:${c}"></i>`;
}
function badge(role, icon, text) { return `<span class="badge st-${role}" data-icon="${icon}">${esc(text)}</span>`; }

/* ------------------------------------------------------------------ theme */
(function initTheme() {
  let saved = null;
  try { saved = localStorage.getItem('amr-theme'); } catch (e) { /* storage blocked */ }
  if (saved) document.documentElement.dataset.theme = saved;
  $('#btnTheme').addEventListener('click', () => {
    const dark = document.documentElement.dataset.theme
      ? document.documentElement.dataset.theme === 'dark'
      : matchMedia('(prefers-color-scheme: dark)').matches;
    const next = dark ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    try { localStorage.setItem('amr-theme', next); } catch (e) { /* ignore */ }
    staticKey = '';
    render();
  });
})();

/* ------------------------------------------------------------------- data */
async function loadLayout() {
  try {
    const r = await fetch('api/layout');
    const j = await r.json();
    if (!j.layout) throw new Error('no layout yet');
    layout = j.layout; graph = j.graph;
    (j.robots || []).forEach(robotSlot);
    fillForm();
    staticKey = '';
  } catch (e) {
    setTimeout(loadLayout, 1500);
  }
}

function onState(s) {
  const now = performance.now();
  for (const r of s.robots) {
    const p = poses[r.robot_id];
    const to = { x: r.pose.x, y: r.pose.y, th: r.pose.theta };
    if (!p) poses[r.robot_id] = { from: to, to, t0: now, t1: now };
    else poses[r.robot_id] = { from: curPose(r.robot_id, now), to, t0: now, t1: now + 520 };
  }
  state = s;
  render();
}

function connect() {
  if (!window.EventSource) return poll();
  const es = new EventSource('api/stream');
  es.onopen = () => setConn(true);
  es.onmessage = (ev) => { try { onState(JSON.parse(ev.data)); setConn(true); } catch (e) { /* skip */ } };
  es.onerror = () => { setConn(false); };
}
async function poll() {
  try { const r = await fetch('api/state'); const j = await r.json(); if (j.robots) { onState(j); setConn(true); } }
  catch (e) { setConn(false); }
  setTimeout(poll, 1000);
}
function setConn(ok) {
  const c = $('#conn');
  c.textContent = ok ? '● đã kết nối' : '● mất kết nối';
  c.style.color = ok ? css('--good-text') : css('--critical');
}

async function post(url, body) {
  try {
    const r = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    return await r.json();
  } catch (e) { return { ok: false, message: String(e) }; }
}

/* ----------------------------------------------------------------- render */
function render() {
  if (!state) return;
  $('#clock').textContent = `t = ${fmtS(state.sim_time_s)}`;
  const fs = $('#fleetState');
  fs.innerHTML = state.paused ? badge('critical', '❚❚', 'Fleet tạm dừng') : badge('good', '▶', 'Fleet đang chạy');
  $('#btnPause').textContent = state.paused ? 'Tiếp tục fleet' : 'Tạm dừng fleet';
  renderKpis(); renderRobots(); renderDetail(); renderLines(); renderChargers(); renderTasks(); renderEvents();
}

function renderKpis() {
  const s = state, p = s.production || {};
  const online = s.robots.filter((r) => r.online).length;
  const working = s.robots.filter((r) => r.task_id).length;
  const queued = s.tasks.filter((t) => t.status === 'QUEUED').length;
  const inUse = s.charger_occupants.filter((o) => o).length;
  const tiles = [
    ['Robot online', `${online}/${s.robots.length}`, `${working} robot đang làm task`],
    ['Task hoàn thành', s.tasks_completed, `${queued} task chờ robot`],
    ['Thành phẩm nhập kho', p.finished_stored ?? 0, `${p.products_made ?? 0} SP đã sản xuất`],
    ['Thông lượng', `${Math.round(p.throughput_per_hour ?? 0)}`, 'thành phẩm / giờ'],
    ['Lượt né đường', s.yield_maneuvers, `${s.deadlocks_resolved} đối đầu / deadlock đã gỡ`],
    ['Trạm sạc', `${inUse}/${s.charger_ids.length}`, `${s.charger_queue.length} robot chờ sạc`],
  ];
  $('#kpis').innerHTML = tiles.map(([l, v, f]) =>
    `<div class="kpi"><div class="label">${l}</div><div class="value">${esc(v)}</div><div class="foot">${esc(f)}</div></div>`).join('');
}

function battHtml(b, charging) {
  const role = b < 15 ? 'critical' : b < 30 ? 'serious' : b < 50 ? 'warning' : 'good';
  return `<span class="batt"><span class="bar"><i style="width:${Math.max(3, b)}%;background:var(--${role})"></i></span>${b.toFixed(0)}%${charging ? ' ⚡' : ''}</span>`;
}

function waitHtml(r) {
  if (r.yielding_to) return badge('warning', '⤳', `Nhường ${r.yielding_to.split(', ').map(rname).join(', ')}`);
  if (r.blocked_by) return badge('serious', '⏸', `Chờ ${rname(r.blocked_by)} ${fmtS(r.waiting_time_s)}`);
  return '<span class="small">—</span>';
}

function renderRobots() {
  const rows = state.robots.map((r) => {
    const [role, icon] = STATE_BADGE[r.state] || ['info', '•'];
    const eta = r.route_node_ids.length ? fmtS(r.eta_goal_s) : '—';
    const etaTask = r.task_id ? fmtS(r.eta_task_s) : '—';
    const next = r.next_task_id
      ? `<b>${esc(r.next_task_id)}</b> <span class="small">${esc(r.next_task_description.replace(r.next_task_id + ': ', ''))} · bắt đầu sau ~${fmtS(r.next_task_eta_start_s)}</span>`
      : `<span class="small">${esc(r.next_task_description)}</span>`;
    const what = r.task_id ? `<b>${esc(r.task_id)}</b> ${esc(r.state_detail)}` : esc(r.state_detail);
    const goal = r.current_goal ? `<b>→ ${esc(r.current_goal)}</b><br>` : '';
    return `<tr data-id="${r.robot_id}" class="${selected === r.robot_id ? 'sel' : ''}">
      <td><span class="rid">${dotHtml(r.robot_id)}${rname(r.robot_id)}</span></td>
      <td class="num"><b>P${r.priority_rank}</b> <span class="small">(${Math.round(r.priority_score)})</span></td>
      <td>${badge(role, icon, STATE_VI[r.state] || r.state)}</td>
      <td>${what}${r.cargo ? `<br><span class="small">📦 ${esc(r.cargo)}</span>` : ''}</td>
      <td class="route">${goal}${esc(r.route_summary || '—')}</td>
      <td class="num">${eta}</td><td class="num">${etaTask}</td>
      <td>${next}</td>
      <td>${battHtml(r.battery_percent, r.charging)}</td>
      <td>${waitHtml(r)}</td></tr>`;
  });
  const tb = $('#robots tbody');
  tb.innerHTML = rows.join('');
  tb.querySelectorAll('tr').forEach((tr) => tr.addEventListener('click', () => { selected = tr.dataset.id; render(); }));
}

function renderDetail() {
  const box = $('#detailBody');
  const r = state.robots.find((x) => x.robot_id === selected);
  if (!r) { box.className = 'detail-empty'; box.textContent = 'Chọn một robot trên bản đồ hoặc trong bảng.'; return; }
  if (document.activeElement && document.activeElement.id === 'dRank') return;  // don't clobber typing
  const [role, icon] = STATE_BADGE[r.state] || ['info', '•'];
  box.className = 'detail';
  box.innerHTML = `
    <div class="title">${dotHtml(r.robot_id)}<h3>${rname(r.robot_id)}</h3>${badge(role, icon, STATE_VI[r.state] || r.state)}</div>
    <dl>
      <dt>Đang làm gì</dt><dd>${esc(r.state_detail)}</dd>
      <dt>Task</dt><dd>${r.task_id ? `<b>${esc(r.task_id)}</b> ${esc(r.task_description)}<br><span class="small">${esc(r.pickup)} → ${esc(r.dropoff)}</span>` : '—'}</dd>
      <dt>Hàng trên xe</dt><dd>${esc(r.cargo || 'Không')}</dd>
      <dt>Đích hiện tại</dt><dd>${esc(r.current_goal || '—')}</dd>
      <dt>Lộ trình</dt><dd>${esc(r.route_summary || '—')}<div class="nodes">${esc(r.route_node_ids.join(' › '))}</div></dd>
      <dt>Quãng đường còn</dt><dd>${r.distance_remaining_m.toFixed(1)} m</dd>
      <dt>ETA đích</dt><dd>${fmtS(r.eta_goal_s)}</dd>
      <dt>ETA xong task</dt><dd>${r.task_id ? fmtS(r.eta_task_s) : '—'}</dd>
      <dt>Task tiếp theo</dt><dd>${esc(r.next_task_description)}</dd>
      <dt>Hàng ưu tiên</dt><dd><b>P${r.priority_rank}</b> · điểm ${Math.round(r.priority_score)}<br><span class="small">${esc(prioWhy(r))}</span></dd>
      <dt>Chờ / nhường</dt><dd>${waitHtml(r)}</dd>
      <dt>Pin</dt><dd>${battHtml(r.battery_percent, r.charging)}</dd>
      <dt>Vị trí</dt><dd>x ${r.pose.x.toFixed(1)}, y ${r.pose.y.toFixed(1)} · ${(r.speed).toFixed(2)} m/s</dd>
      <dt>Ô đang giữ</dt><dd class="nodes">${esc((r.held_nodes || []).join(', '))}</dd>
      <dt>Thống kê</dt><dd>${r.tasks_completed} task · ${r.odometer_m.toFixed(0)} m</dd>
    </dl>
    <div class="controls">
      <label class="small">Hạng cơ bản <input id="dRank" type="number" min="1" max="${state.robots.length}" value="${r.base_rank}"></label>
      <button class="btn" id="dSetRank">Đặt hạng</button>
      <button class="btn ${r.paused ? '' : 'btn-danger'}" id="dPause">${r.paused ? 'Cho chạy tiếp' : 'Tạm dừng robot'}</button>
    </div>`;
  $('#dSetRank').onclick = async () => {
    const res = await post('api/priority', { robot_id: r.robot_id, base_rank: parseInt($('#dRank').value, 10) });
    if (!res.ok) alert(res.message);
  };
  $('#dPause').onclick = async () => { await post('api/priority', { robot_id: r.robot_id, base_rank: 0, paused: !r.paused }); };
}

function meter(n, cap, inbound) {
  let h = '';
  for (let i = 0; i < cap; i++) h += `<i class="${i < n ? 'on' : i < n + inbound ? 'in' : ''}"></i>`;
  return `<span class="meter">${h}</span>`;
}

function renderLines() {
  const lines = (state.production || {}).lines || [];
  const out = [];
  for (const ln of lines) {
    if (ln.line_type === 'supply') continue;
    const [role, icon, text] = LINE_BADGE[ln.status] || ['info', '•', ln.status];
    if (ln.line_type === 'storage') {
      out.push(`<div class="line"><span class="name">${esc(ln.line_id)} · Kho thành phẩm</span>${badge('info', '▣', `${ln.input_buffer} SP`)}</div>`);
      continue;
    }
    const kind = ln.line_type === 'production' ? 'Sản xuất' : 'Đóng gói';
    const inUnit = ln.line_type === 'production' ? 'kit' : 'thùng';
    const outUnit = ln.line_type === 'production' ? 'thùng' : 'pallet';
    out.push(`<div class="line">
      <span class="name">${esc(ln.line_id)} · ${kind}</span>${badge(role, icon, text)}
      <div class="meta">
        <span>Vào (${inUnit}) ${meter(ln.input_buffer, ln.input_capacity, ln.inbound)} ${ln.input_buffer}/${ln.input_capacity}${ln.inbound ? ` +${ln.inbound} đang tới` : ''}</span>
        <span>Ra (${outUnit}) ${meter(ln.output_buffer, ln.output_capacity, 0)} ${ln.output_buffer}/${ln.output_capacity}</span>
        <span>Đang xếp ${ln.work_in_progress} SP</span>
        <span>${ln.line_type === 'production' ? 'Đã SX' : 'Đã đóng gói'} <b>${ln.produced}</b></span>
        <span>Hiệu suất ${(ln.utilization * 100).toFixed(0)}%</span>
      </div></div>`);
  }
  $('#lines').innerHTML = out.join('') || '<p class="hint">Chưa có dữ liệu.</p>';
}

function renderChargers() {
  const byId = Object.fromEntries(state.robots.map((r) => [r.robot_id, r]));
  const docks = state.charger_ids.map((cid, i) => {
    const occ = state.charger_occupants[i];
    const r = byId[occ];
    const status = !occ ? badge('good', '○', 'Trống')
      : r && r.state === 'CHARGING' ? badge('serious', '⚡', 'Đang sạc') : badge('info', '→', 'Đang tới');
    return `<div class="dock"><div class="id">${esc(cid)}</div>${status}
      <div class="who">${occ ? `${rname(occ)}${r ? ` · ${r.battery_percent.toFixed(0)}%` : ''}` : ''}</div></div>`;
  });
  const q = state.charger_queue.length
    ? `Hàng đợi sạc: ${state.charger_queue.map((id) => `${rname(id)} (${(byId[id]?.battery_percent ?? 0).toFixed(0)}%)`).join(' → ')}`
    : 'Không có robot chờ sạc.';
  $('#chargers').innerHTML = `<div class="docks">${docks.join('')}</div><p class="queue">${q}</p>
    <p class="hint">Robot đi sạc khi pin &lt; 30%, sạc tranh thủ khi rảnh và pin &lt; 50%, rời dock khi ≥ 95% (hoặc sớm hơn nếu có task).</p>`;
}

function renderTasks() {
  const rows = state.tasks.map((t) => `<tr>
    <td><b>${esc(t.task_id)}</b></td><td>${esc(TASK_VI[t.task_type] || t.task_type)}</td>
    <td>${esc(t.pickup)} → ${esc(t.dropoff)}</td>
    <td>${t.priority >= 2 ? badge(t.priority === 3 ? 'critical' : 'serious', '▲', t.priority) : t.priority}</td>
    <td>${esc(TASK_STATUS_VI[t.status] || t.status)}</td>
    <td>${t.robot_id ? rname(t.robot_id) : t.next_for_robot ? `kế tiếp: ${rname(t.next_for_robot)}` : '—'}</td>
    <td class="num">${fmtS(t.age_s)}</td></tr>`);
  $('#tasks tbody').innerHTML = rows.join('') || '<tr><td colspan="7" class="hint">Không có task.</td></tr>';
  $('#taskCount').textContent = `${state.tasks.length} task đang mở`;
}

function renderEvents() {
  const f = $('#evFilter').value;
  const evs = (state.events || []).filter((e) => !f || e.category === f).slice().reverse();
  const role = (l) => (l >= 2 ? 'critical' : l === 1 ? 'warning' : 'info');
  $('#events').innerHTML = evs.map((e) => `<li><span class="t">${fmtS(e.t)}</span>
    <span class="cat">${badge(role(e.level), e.level >= 1 ? '!' : 'i', CAT_VI[e.category] || e.category)}</span>
    <span>${esc(e.text)}</span></li>`).join('');
}

function fillForm() {
  const st = (layout.stations || []);
  const opts = (type) => st.filter((s) => !type || s.type === type).map((s) => `<option>${esc(s.id)}</option>`).join('');
  $('#fPickup').innerHTML = opts('pickup') + `<optgroup label="Khác">${opts('dropoff')}</optgroup>`;
  $('#fDropoff').innerHTML = opts('dropoff') + `<optgroup label="Khác">${opts('pickup')}</optgroup>`;
}

/* -------------------------------------------------------------------- map */
const canvas = $('#map');
const ctx = canvas.getContext('2d');
let view = { s: 1, ox: 0, oy: 0, W: 100, H: 60 };

function resize() {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  staticKey = '';
}
window.addEventListener('resize', resize);

function w2c(x, y) { return [view.ox + x * view.s, view.oy + (view.H - y) * view.s]; }
function c2w(px, py) { return [(px - view.ox) / view.s, view.H - (py - view.oy) / view.s]; }

function drawStatic() {
  const key = `${canvas.width}x${canvas.height}:${document.documentElement.dataset.theme || ''}:${!!layout}`;
  if (key === staticKey && staticLayer) return;
  staticKey = key;
  staticLayer = document.createElement('canvas');
  staticLayer.width = canvas.width; staticLayer.height = canvas.height;
  const g = staticLayer.getContext('2d');
  if (!layout) return;
  const [W, H] = layout.factory.size;
  const pad = 10 * (window.devicePixelRatio || 1);
  const s = Math.min((canvas.width - 2 * pad) / W, (canvas.height - 2 * pad) / H);
  view = { s, ox: (canvas.width - W * s) / 2, oy: (canvas.height - H * s) / 2, W, H };
  const P = (x, y) => w2c(x, y);
  const dpr = window.devicePixelRatio || 1;
  g.fillStyle = css('--surface');
  g.fillRect(0, 0, staticLayer.width, staticLayer.height);
  // hall
  let [x0, y0] = P(0, H); g.fillStyle = css('--floor'); g.fillRect(x0, y0, W * s, H * s);
  g.strokeStyle = css('--axis'); g.lineWidth = 3 * dpr; g.strokeRect(x0, y0, W * s, H * s);
  // zones
  for (const z of layout.zones || []) {
    const [a, b, c, d] = z.rect;
    const [px, py] = P(a, d);
    const col = z.color || [0.5, 0.5, 0.5];
    g.fillStyle = `rgba(${col.map((v) => Math.round(v * 255)).join(',')},0.13)`;
    g.fillRect(px, py, (c - a) * s, (d - b) * s);
    g.strokeStyle = `rgba(${col.map((v) => Math.round(v * 255)).join(',')},0.55)`;
    g.lineWidth = 1 * dpr; g.setLineDash([4 * dpr, 3 * dpr]); g.strokeRect(px, py, (c - a) * s, (d - b) * s); g.setLineDash([]);
  }
  // objects
  for (const o of layout.objects || []) {
    const [a, b, c, d] = o.rect;
    const [px, py] = P(a, d);
    g.fillStyle = o.kind === 'charger' ? css('--s4') : css('--obj');
    g.fillRect(px, py, (c - a) * s, (d - b) * s);
    g.strokeStyle = css('--obj-ink'); g.lineWidth = 1; g.strokeRect(px, py, (c - a) * s, (d - b) * s);
  }
  // lanes
  if (graph) {
    const N = graph.nodes;
    for (const e of graph.edges) {
      const a = N[e.u], b = N[e.v];
      const [ax, ay] = P(a.x, a.y), [bx, by] = P(b.x, b.y);
      g.beginPath(); g.moveTo(ax, ay); g.lineTo(bx, by);
      if (!e.lane) { g.strokeStyle = css('--obj-ink'); g.lineWidth = 1 * dpr; g.setLineDash([2 * dpr, 2 * dpr]); }
      else if (e.two_way) { g.strokeStyle = css('--lane-two'); g.lineWidth = 2 * dpr; g.setLineDash([5 * dpr, 3 * dpr]); }
      else { g.strokeStyle = css('--lane'); g.lineWidth = 1.5 * dpr; g.setLineDash([]); }
      g.stroke(); g.setLineDash([]);
      if (e.lane && !e.two_way) {   // small arrow head in the middle of the edge
        const L = Math.hypot(bx - ax, by - ay);
        if (L > 9 * dpr) {
          const ux = (bx - ax) / L, uy = (by - ay) / L, mx = (ax + bx) / 2, my = (ay + by) / 2, k = 3.2 * dpr;
          g.fillStyle = css('--lane');
          g.beginPath(); g.moveTo(mx + ux * k, my + uy * k);
          g.lineTo(mx - ux * k - uy * k, my - uy * k + ux * k); g.lineTo(mx - ux * k + uy * k, my - uy * k - ux * k);
          g.closePath(); g.fill();
        }
      }
    }
    // special nodes
    g.font = `${Math.max(9, 0.95 * s)}px system-ui`;
    g.textAlign = 'center'; g.textBaseline = 'middle';
    for (const n of N) {
      const [px, py] = P(n.x, n.y);
      if (n.kind === 'station' || n.kind === 'queue') {
        g.fillStyle = n.kind === 'station' ? css('--good') : 'transparent';
        g.strokeStyle = css('--good'); g.lineWidth = 1.5 * dpr;
        const r = 0.45 * s; g.beginPath(); g.rect(px - r, py - r, 2 * r, 2 * r);
        if (n.kind === 'station') { g.globalAlpha = 0.35; g.fill(); g.globalAlpha = 1; }
        g.stroke();
        if (n.kind === 'station') { g.fillStyle = css('--ink'); g.fillText(n.id, px, py + (n.y > 30 ? -1 : 1) * 1.25 * s); }
      } else if (n.kind === 'charger' || n.kind === 'parking') {
        g.strokeStyle = n.kind === 'charger' ? css('--s4') : css('--muted');
        g.lineWidth = 1.5 * dpr; const r = 0.42 * s;
        g.strokeRect(px - r, py - r, 2 * r, 2 * r);
      } else if (n.kind === 'yield') {
        g.fillStyle = css('--warning'); g.globalAlpha = 0.7;
        g.beginPath(); g.arc(px, py, 0.28 * s, 0, 2 * Math.PI); g.fill(); g.globalAlpha = 1;
      }
    }
  }
  // zone labels on top: two lines, placed in free floor space, with a surface halo
  g.textAlign = 'center'; g.textBaseline = 'middle'; g.lineJoin = 'round';
  for (const z of layout.zones || []) {
    if (z.id === 'PK' || z.id === 'PK2') continue;
    const [a, b, c, d] = z.rect;
    const at = LABEL_AT[z.id] || [(a + c) / 2, (b + d) / 2];
    const [px, py] = P(at[0], at[1]);
    const [l1, l2] = (ZONE_VI[z.id] || z.name).split(' · ');
    const lines = l2 ? [[l1, 1.5, 600], [l2, 1.15, 500]] : [[l1, 1.3, 600]];
    let y = py - (lines.length - 1) * 0.75 * s;
    for (const [txt, size, weight] of lines) {
      g.font = `${weight} ${Math.max(10, size * s)}px system-ui`;
      g.lineWidth = 4 * dpr; g.strokeStyle = css('--floor'); g.strokeText(txt, px, y);
      g.fillStyle = css('--ink-2'); g.fillText(txt, px, y);
      y += 1.5 * s;
    }
  }
}

const PRIO_VI = ['thấp', 'bình thường', 'cao (dây chuyền sắp dừng)', 'khẩn'];
// why a robot has its rank: the priority follows its task (priority_mode: task)
function prioWhy(r) {
  const t = (state.tasks || []).find((x) => x.task_id === r.task_id);
  if (!t) {
    if (r.state === 'TO_CHARGER' || r.state === 'WAIT_CHARGER') {
      return r.battery_percent < 15 ? 'đi sạc, pin rất thấp: ưu tiên cao nhất' : 'đi sạc';
    }
    return `không có task · hạng cơ bản ${r.base_rank} (chỉ để phân xử khi hoà)`;
  }
  const parts = [`task ${t.task_id} mức ${PRIO_VI[t.priority] || t.priority}`];
  parts.push(r.cargo ? 'đang chở hàng (+400)' : 'chưa lấy hàng');
  parts.push(`task chờ ${fmtS(t.age_s)}`);
  return parts.join(' · ');
}

function curPose(id, now) {
  const p = poses[id];
  if (!p) return null;
  const f = p.t1 > p.t0 ? Math.min(1, Math.max(0, (now - p.t0) / (p.t1 - p.t0))) : 1;
  let dth = p.to.th - p.from.th;
  while (dth > Math.PI) dth -= 2 * Math.PI;
  while (dth < -Math.PI) dth += 2 * Math.PI;
  return { x: p.from.x + (p.to.x - p.from.x) * f, y: p.from.y + (p.to.y - p.from.y) * f, th: p.from.th + dth * f };
}

function drawFrame() {
  requestAnimationFrame(drawFrame);
  if (!canvas.width) resize();
  drawStatic();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (staticLayer) ctx.drawImage(staticLayer, 0, 0);
  if (!state || !layout) return;
  const dpr = window.devicePixelRatio || 1;
  const s = view.s;
  const now = performance.now();
  const showAll = $('#optRoutes').checked;
  // routes
  for (const r of state.robots) {
    const mine = r.robot_id === selected;
    if (!showAll && !mine) continue;
    const pts = r.route_points;
    if (!pts || pts.length < 2) continue;
    const col = robotColor(r.robot_id);
    const cp = curPose(r.robot_id, now) || r.pose;
    const draw = (from, to, width, alpha) => {
      ctx.beginPath();
      const [sx, sy] = w2c(cp.x, cp.y); ctx.moveTo(sx, sy);
      for (let i = Math.max(1, from); i < to; i++) { const [px, py] = w2c(pts[i][0], pts[i][1]); ctx.lineTo(px, py); }
      ctx.strokeStyle = col; ctx.globalAlpha = alpha; ctx.lineWidth = width; ctx.lineJoin = 'round'; ctx.stroke(); ctx.globalAlpha = 1;
    };
    const dim = selected && !mine ? 0.25 : 0.85;
    draw(1, pts.length, (mine ? 3 : 2) * dpr, dim);
    draw(1, Math.min(r.granted_count, pts.length), (mine ? 7 : 5) * dpr, dim);
    const [gx, gy] = w2c(pts[pts.length - 1][0], pts[pts.length - 1][1]);
    ctx.fillStyle = col; ctx.globalAlpha = dim;
    ctx.beginPath(); ctx.arc(gx, gy, 3.2 * dpr, 0, 2 * Math.PI); ctx.fill(); ctx.globalAlpha = 1;
  }
  // robots
  ctx.font = `600 ${Math.max(10, 1.05 * s)}px system-ui`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  for (const r of state.robots) {
    const p = curPose(r.robot_id, now) || r.pose;
    const [px, py] = w2c(p.x, p.y);
    const col = robotColor(r.robot_id);
    const R = Math.max(5 * dpr, 0.75 * s);
    ctx.save(); ctx.translate(px, py); ctx.rotate(-p.th);
    // body (0.8 x 0.56 m, drawn slightly enlarged for legibility)
    const L = Math.max(9 * dpr, 1.15 * s), Wd = Math.max(6 * dpr, 0.8 * s);
    ctx.fillStyle = robotHollow(r.robot_id) ? css('--surface') : col;
    ctx.strokeStyle = robotHollow(r.robot_id) ? col : css('--surface');
    ctx.lineWidth = (robotHollow(r.robot_id) ? 3 : 2) * dpr;
    ctx.beginPath(); ctx.roundRect ? ctx.roundRect(-L / 2, -Wd / 2, L, Wd, 2 * dpr) : ctx.rect(-L / 2, -Wd / 2, L, Wd);
    ctx.fill(); ctx.stroke();
    // heading tick
    ctx.strokeStyle = robotHollow(r.robot_id) ? col : css('--surface'); ctx.lineWidth = 2 * dpr;
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(L / 2, 0); ctx.stroke();
    if (r.cargo) { ctx.fillStyle = css('--s2'); ctx.fillRect(-L / 4, -Wd / 4, L / 3, Wd / 2); }
    ctx.restore();
    if (r.robot_id === selected) {
      ctx.strokeStyle = css('--ink'); ctx.lineWidth = 2 * dpr;
      ctx.beginPath(); ctx.arc(px, py, R * 1.6, 0, 2 * Math.PI); ctx.stroke();
    }
    if (r.state === 'YIELDING' || r.blocked_by) {
      ctx.strokeStyle = css(r.state === 'YIELDING' ? '--warning' : '--serious'); ctx.lineWidth = 2 * dpr;
      ctx.setLineDash([3 * dpr, 2 * dpr]); ctx.beginPath(); ctx.arc(px, py, R * 1.3, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
    }
    // label: id number, in ink colour with a surface halo
    const label = r.robot_id.replace(/^\D+/, '');
    ctx.lineWidth = 3 * dpr; ctx.strokeStyle = css('--surface'); ctx.strokeText(label, px, py - R * 1.7);
    ctx.fillStyle = css('--ink'); ctx.fillText(label, px, py - R * 1.7);
  }
}

function robotAt(evt) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const [x, y] = c2w((evt.clientX - rect.left) * dpr, (evt.clientY - rect.top) * dpr);
  let best = null, bd = 2.0;
  for (const r of (state ? state.robots : [])) {
    const p = curPose(r.robot_id, performance.now()) || r.pose;
    const d = Math.hypot(p.x - x, p.y - y);
    if (d < bd) { bd = d; best = r; }
  }
  return { r: best, x, y };
}
canvas.addEventListener('mousemove', (evt) => {
  const tip = $('#tip');
  const { r, x, y } = robotAt(evt);
  if (!r) {
    tip.hidden = true;
    canvas.title = `x ${x.toFixed(1)} m, y ${y.toFixed(1)} m`;
    return;
  }
  const rect = canvas.getBoundingClientRect();
  tip.hidden = false;
  tip.style.left = `${Math.min(evt.clientX - rect.left + 14, rect.width - 300)}px`;
  tip.style.top = `${evt.clientY - rect.top + 14}px`;
  tip.innerHTML = `<b>${rname(r.robot_id)}</b> · P${r.priority_rank} · ${esc(STATE_VI[r.state] || r.state)}<br>
    ${esc(r.state_detail)}<br>Route: ${esc(r.route_summary || '—')}<br>
    ETA đích ${fmtS(r.eta_goal_s)}${r.task_id ? ` · xong task ${fmtS(r.eta_task_s)}` : ''} · pin ${r.battery_percent.toFixed(0)}%<br>
    Kế tiếp: ${esc(r.next_task_description)}`;
});
canvas.addEventListener('mouseleave', () => { $('#tip').hidden = true; });
canvas.addEventListener('click', (evt) => { const { r } = robotAt(evt); selected = r ? r.robot_id : null; render(); });

/* --------------------------------------------------------------- controls */
$('#btnPause').addEventListener('click', () => post('api/pause', { paused: !(state && state.paused) }));
$('#evFilter').addEventListener('change', renderEvents);
$('#optRoutes').addEventListener('change', () => {});
$('#btnDemo').addEventListener('click', async () => {
  const res = await post('api/demo', {});
  $('#demoMsg').textContent = res.ok ? `Demo với ${res.message.replace(/amr_/g, 'AMR-')}: xem bản đồ và nhật ký "Né đường"` : `Không chạy được: ${res.message}`;
});
$('#taskForm').addEventListener('submit', async (e) => {
  e.preventDefault();
  const res = await post('api/task', { pickup: $('#fPickup').value, dropoff: $('#fDropoff').value, priority: parseInt($('#fPrio').value, 10) });
  $('#formMsg').textContent = res.ok ? `Đã tạo ${res.task_id}` : `Lỗi: ${res.message}`;
});

resize();
loadLayout();
connect();
requestAnimationFrame(drawFrame);
