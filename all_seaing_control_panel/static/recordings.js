// Recordings tab: laptop-side MCAP recorder (recorder.py) and viewer/analysis of its recordings
// (recordings.py). Sources are the rolling buffer and saved ranges; a source shows a timeline
// (fusion outcomes per second) on which a range can be dragged and saved, a frame viewer (camera
// keyframe + YOLO boxes colored by fusion outcome + projected LiDAR + top view) and the run-log
// analyzers run on the recording's /run_log.

const LIVE_SRC = "__live__";  // the live view (live_view.py): latest frames through the bridge, not recorded
const isBufferSource = id => id === "__buffer__" || id.startsWith("__session__");  // vs saved recordings
const rec = {source: null, cameras: {}, timeline: null, range: null, frames: [], order: [], pos: 0, frame: null,
             playing: null, img: new Image()};

// ---------------------------------------------------------------- recorder
async function refreshRecorder() {
  try {
    const s = await api(`/api/recorder?${tq()}`);
    const b = $("#rec-badge");
    b.textContent = !s.enabled ? "stopped" : s.connected ? "recording" : "connecting";
    b.className = "badge " + (!s.enabled ? "stopped" : s.connected ? "ok" : "warn");
    const top = Object.entries(s.messages).sort((a, c) => c[1] - a[1]).slice(0, 6)
      .map(([t, n]) => `${esc(t)}: ${n}`).join("<br>");
    $("#rec-status").innerHTML = `${esc(s.url)}<br>buffer: ${s.segments} files, ${s.buffer_mb} MB` +
      (s.error ? `<br><span class="bad">${esc(s.error)}</span>` : "") + (top ? `<br>${top}` : "");
  } catch (e) { $("#rec-status").textContent = e.message; }
}
$("#rec-start").onclick = async () => { await api(`/api/recorder/start?${tq()}`, {method: "POST"}); refreshRecorder(); };
$("#rec-stop").onclick = async () => { await api(`/api/recorder/stop?${tq()}`, {method: "POST"}); refreshRecorder(); };
setInterval(() => { if ($("#tab-recordings").classList.contains("active")) refreshRecorder(); }, 3000);

// ---------------------------------------------------------------- sources
async function loadRecordings(select) {
  refreshRecorder();
  const r = await api("/api/recordings");
  rec.cameras = r.cameras;
  rec.sourceNames = Object.fromEntries(r.sources.map(s => [s.id, s.name]));
  $("#rec-sources").innerHTML = `<div class="rec-source ${rec.source === LIVE_SRC ? "active" : ""}" data-id="${LIVE_SRC}">
       <b>● Live view</b><div class="muted">latest frames through the bridge, not recorded</div></div>` +
    (r.sources.length ? r.sources.map(s =>
    `<div class="rec-source ${s.id === rec.source ? "active" : ""}" data-id="${esc(s.id)}">
       <button class="small secondary rec-del" data-del="${esc(s.id)}" title="${s.buffer ? "Delete this session from the buffer (an active one keeps its segment being recorded)" : "Delete this recording"}" style="float:right">×</button><b>${esc(s.name)}</b>${s.active ? ' <span class="badge ok">recording</span>' : ""}
       <div class="muted">${s.size_mb} MB${s.files > 1 ? `, ${s.files} segments` : ""}</div></div>`).join("")
    : '<div class="muted">No recordings yet: start the recorder.</div>');
  $("#rec-sources").querySelectorAll(".rec-source").forEach(d => d.onclick = e => {
    if (e.target.dataset.del) deleteSource(e.target.dataset.del); else selectSource(d.dataset.id);
  });
  const sel = $("#rec-camera");
  if (!sel.options.length) {
    sel.innerHTML = Object.keys(rec.cameras).map(c => `<option value="${esc(c)}">${esc(rec.cameras[c].title || c)}</option>`).join("");
    const an = await api("/api/analyzers");
    $("#rec-analyzer").innerHTML = an.map(a => `<option value="${a.name}" ${a.name === "fusion" ? "selected" : ""}>${esc(a.name)}</option>`).join("");
    filterAnalyzers($("#rec-analyzer"));  // domains.js
  }
  if (select) selectSource(select);
}
$("#rec-refresh").onclick = () => loadRecordings();

async function selectSource(id) {
  rec.source = id; rec.range = null;
  document.querySelectorAll(".rec-source").forEach(d => d.classList.toggle("active", d.dataset.id === id));
  $("#rec-detail").classList.remove("hidden");
  const live = id === LIVE_SRC;
  ["#rec-timeline-card", "#rec-analysis-card", "#rec-playback", "#rec-filter-label"].forEach(sel => $(sel).classList.toggle("hidden", live));
  if (rec.playing) $("#rec-play").click();
  if (!state.descriptions) { try { state.descriptions = await api("/api/descriptions"); } catch { state.descriptions = {}; } }
  if (live) {  // live: the camera frames (LiDAR/Camera) or the SLAM / Tasks map (domains.js)
    rec.frame = null; clearFrame();
    [...$("#rec-camera").options].forEach(o => { o.disabled = false; o.textContent = rec.cameras[o.value]?.title || o.value; });
    if (DOMAINS[currentDomain()].map) refreshRecMap(); else liveLoop();
    return;
  }
  const isBuf = isBufferSource(id);
  $("#rec-name").textContent = rec.sourceNames?.[id] || id.replace(/\.mcap$/, "");
  $("#rec-delete").classList.toggle("hidden", isBuf);
  $("#rec-save-bar").querySelectorAll("input, #rec-save").forEach(e => e.classList.toggle("hidden", !isBuf));
  $("#rec-timeline").innerHTML = '<div class="muted">loading…</div>';
  rec.timeline = await api(`/api/recordings/${encodeURIComponent(id)}/timeline`);
  drawTimeline();
  await markCameras();
  await loadFrames();
  runRecAnalysis();
}

// ---------------------------------------------------------------- timeline + range selection
function drawTimeline() {
  const tl = rec.timeline, box = $("#rec-timeline");
  if (!tl.seconds.length) { box.innerHTML = '<div class="muted">empty</div>'; return; }
  const W = 900, H = 120, M = 4, t0 = tl.start, t1 = tl.end;
  // gaps without data longer than GAP_S are collapsed to a GAP_PX break, so separate sessions
  // (e.g. several recorder runs in the buffer) each get width in proportion to their length
  const GAP_S = 20, GAP_PX = 16, segs = [], B = tl.bucket_s || 1;  // B: bucket length [s]
  for (const sec of tl.seconds) {
    const last = segs[segs.length - 1];
    if (last && sec.t - last.end <= GAP_S) last.end = sec.t + B; else segs.push({start: sec.t, end: sec.t + B});
  }
  const shown = segs.reduce((n, g) => n + g.end - g.start, 0);
  const k = (W - 2 * M - GAP_PX * (segs.length - 1)) / shown;  // px per second of data
  let x = M;
  for (const g of segs) { g.x = x; x += (g.end - g.start) * k + GAP_PX; }
  const sx = t => {  // time -> px (times inside a gap map to its break)
    let g = segs[0];
    for (const s of segs) if (t >= s.start) g = s;
    return g.x + Math.min(t - g.start, g.end - g.start) * k;
  };
  const tx = px => {  // px -> time
    let g = segs[0];
    for (const s of segs) if (px >= s.x) g = s;
    return g.start + Math.min(Math.max(0, px - g.x) / k, g.end - g.start);
  };
  const maxN = Math.max(1, ...tl.seconds.map(s => s.detections + s.rejected + s.skipped));
  const bw = Math.max(1, k * B);
  let bars = "";
  for (const s of tl.seconds) {
    let y = H - 22;
    for (const [k, c] of [["detections", "var(--ok)"], ["rejected", "var(--fail)"], ["skipped", "var(--stale)"]]) {
      const h = s[k] / maxN * (H - 30);
      if (h) { y -= h; bars += `<rect x="${sx(s.t)}" y="${y}" width="${bw}" height="${h}" fill="${c}"/>`; }
    }
    if (s.error || s.warn) bars += `<rect x="${sx(s.t)}" y="${H - 20}" width="${bw}" height="4" fill="${s.error ? "var(--fail)" : "var(--warn)"}"/>`;
    const kf = Object.keys(rec.cameras).reduce((n, c) => n + (s[`kf_${c}`] || 0), 0);
    if (kf) bars += `<rect x="${sx(s.t)}" y="${H - 14}" width="${bw}" height="4" fill="#2557d6" opacity="${Math.min(1, 0.2 + kf / (15 * B))}"/>`;
    if (s.clouds) bars += `<rect x="${sx(s.t)}" y="${H - 8}" width="${bw}" height="4" fill="#0f8b8d" opacity="${Math.min(1, 0.2 + s.clouds / (5 * B))}"/>`;
  }
  const r = rec.range;
  const sel = `<rect class="rec-sel" x="${r ? sx(r[0]) : 0}" y="0" width="${r ? Math.max(1, sx(r[1]) - sx(r[0])) : 0}" height="${H}" fill="#2557d6" opacity="0.18" stroke="#2557d6" pointer-events="none"/>`;
  const cur = rec.frame ? `<line x1="${sx(rec.frame.time)}" x2="${sx(rec.frame.time)}" y1="0" y2="${H}" stroke="var(--text)" stroke-width="1.5"/>` : "";
  const fmtGap = sec => sec >= 3600 ? `${(sec / 3600).toFixed(1)} h` : sec >= 60 ? `${Math.round(sec / 60)} min` : `${Math.round(sec)} s`;
  const breaks = segs.slice(1).map((g, i) => {
    const bx = g.x - GAP_PX / 2, gap = g.start - segs[i].end;
    return `<line x1="${bx - 3}" x2="${bx - 3}" y1="0" y2="${H}" stroke="var(--muted)" stroke-dasharray="3 3"/>
      <line x1="${bx + 3}" x2="${bx + 3}" y1="0" y2="${H}" stroke="var(--muted)" stroke-dasharray="3 3"/>
      <title>${fmtGap(gap)} without data</title>`;
  }).join("");
  box.innerHTML = `<svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="none">${bars}${breaks}${sel}${cur}</svg>
    <div class="legend"><span style="--c:var(--ok)">detections</span><span style="--c:var(--fail)">rejected boxes</span>
    <span style="--c:var(--stale)">skipped boxes</span><span style="--c:var(--warn)">warnings/errors</span>
    <span style="--c:#2557d6">image keyframes</span><span style="--c:#0f8b8d">fused LiDAR scans</span>
    <span class="muted">${fmtTime(t0)} – ${fmtTime(t1)} (${Math.round(shown)} s of data, ${B} s bars${segs.length > 1 ? ` in ${segs.length} sessions; gaps collapsed (dashed)` : ""})</span></div>`;
  const svg = box.querySelector("svg");
  const toT = ev => { const b = svg.getBoundingClientRect(); return tx((ev.clientX - b.left) / b.width * W); };
  // drag = select a range (only the selection rectangle is updated while dragging); click = jump there
  svg.onmousedown = ev => {
    ev.preventDefault();
    const a = toT(ev), rect = svg.querySelector(".rec-sel");
    let moved = false;
    const move = e2 => {
      const b2 = toT(e2);
      if (Math.abs(sx(b2) - sx(a)) < 2 && !moved) return;
      moved = true;
      rec.range = [Math.min(a, b2), Math.max(a, b2)];
      rect.setAttribute("x", sx(rec.range[0]));
      rect.setAttribute("width", Math.max(1, sx(rec.range[1]) - sx(rec.range[0])));
      showRange();
    };
    const up = () => {
      window.removeEventListener("mousemove", move); window.removeEventListener("mouseup", up);
      if (!moved) { rec.range = null; jumpToTime(a); } else applyFilter();
      drawTimeline(); showRange(); runRecAnalysis();
    };
    window.addEventListener("mousemove", move); window.addEventListener("mouseup", up);
  };
}
function showRange() {
  const r = rec.range;
  $("#rec-range").textContent = r ? `range ${new Date(r[0] * 1000).toLocaleTimeString()} – ${new Date(r[1] * 1000).toLocaleTimeString()} (${(r[1] - r[0]).toFixed(0)} s)` : "no range selected (drag on the timeline; analysis covers the whole source)";
}
$("#rec-save").onclick = async () => {
  if (!rec.range) return alert("Drag on the timeline to select a range first.");
  const name = $("#rec-save-name").value.trim() || `recording_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-")}`;
  try {
    const r = await api("/api/recordings/save", {method: "POST", body: {t0: rec.range[0], t1: rec.range[1], name, source: rec.source}});
    $("#rec-range").textContent = `saved ${r.saved}: ${r.messages} messages, ${r.size_mb} MB`;
    loadRecordings();
  } catch (e) { alert(e.message); }
};
async function deleteSource(id) {
  const buffer = isBufferSource(id);
  if (!confirm(buffer ? `Delete ${rec.sourceNames?.[id] || id} from the buffer? (a segment being recorded is kept)`
                      : `Delete the recording ${id.replace(/\.mcap$/, "")}? This can't be undone.`)) return;
  try { await api(`/api/recordings/${encodeURIComponent(id)}`, {method: "DELETE"}); }
  catch (e) { return alert(e.message); }
  if (rec.source === id) { rec.source = null; $("#rec-detail").classList.add("hidden"); }
  loadRecordings();
}
$("#rec-delete").onclick = () => deleteSource(rec.source);

// ---------------------------------------------------------------- frames
const nRejected = f => Object.entries(f.counts).filter(([k]) => !["detected", "skipped", "fused"].includes(k)).reduce((s, [, n]) => s + n, 0);
// grey out the cameras without frames in this source (e.g. the boat's three vs the sim's one) and
// switch to one that has frames
async function markCameras() {
  const sel = $("#rec-camera"), src = rec.source;
  const counts = await Promise.all([...sel.options].map(o =>
    api(`/api/recordings/${encodeURIComponent(src)}/frames?camera=${encodeURIComponent(o.value)}`).then(f => f.length).catch(() => 0)));
  if (rec.source !== src) return;
  [...sel.options].forEach((o, i) => { o.disabled = !counts[i]; o.textContent = `${rec.cameras[o.value]?.title || o.value}${counts[i] ? "" : " (no frames)"}`; });
  if (!counts[sel.selectedIndex]) { const i = counts.findIndex(n => n); if (i >= 0) sel.selectedIndex = i; }
}

async function loadFrames() {
  if (rec.source === LIVE_SRC) return;  // the live loop follows the camera selection
  $("#rec-filter").value = "fused";  // default; applyFilter falls back to "all" without fusion logs
  rec.frames = await api(`/api/recordings/${encodeURIComponent(rec.source)}/frames?camera=${encodeURIComponent($("#rec-camera").value)}`);
  applyFilter();
}
function applyFilter() {
  const f = $("#rec-filter").value;
  let fr = rec.frames.filter(x => !rec.range || (x.time >= rec.range[0] && x.time <= rec.range[1]));
  // no fusion log in this recording (e.g. an imported pre-run-log rosbag): only "all keyframes" makes sense
  const noFusionLog = !rec.frames.some(x => Object.keys(x.counts).length);
  const outcomesKnown = rec.frames.some(x => nRejected(x));  // rejection reasons logged (not just fusion outputs)
  [...$("#rec-filter").options].forEach(o => o.disabled = noFusionLog ? o.value !== "all" : !outcomesKnown && !["all", "fused"].includes(o.value));
  if (noFusionLog && f !== "all") { $("#rec-filter").value = "all"; return applyFilter(); }
  $("#rec-no-fusion").classList.toggle("hidden", !noFusionLog);
  if (f === "fused") fr = fr.filter(x => Object.keys(x.counts).length);  // fusion logged outcomes for it
  if (f === "rejected") fr = fr.filter(x => nRejected(x));
  if (f === "no_points") fr = fr.filter(x => x.counts.no_points);
  if (f === "worst") fr = fr.filter(x => nRejected(x)).sort((a, b) =>
    (b.counts.no_points || 0) - (a.counts.no_points || 0) || nRejected(b) - nRejected(a) || (b.counts.detected || 0) - (a.counts.detected || 0));
  rec.order = fr.map(x => x.index);
  rec.pos = 0;
  $("#rec-slider").max = Math.max(0, rec.order.length - 1);
  if (rec.order.length) showFrame(0);
  else { $("#rec-frame-info").textContent = "no matching keyframes"; clearFrame(); }
}
function clearFrame() {
  const c = $("#rec-image"); c.getContext("2d").clearRect(0, 0, c.width, c.height);
  $("#rec-boxes").innerHTML = ""; $("#rec-zooms").innerHTML = ""; $("#rec-events").textContent = "";
}
$("#rec-camera").onchange = loadFrames;
$("#rec-filter").onchange = applyFilter;
$("#rec-slider").oninput = () => showFrame(+$("#rec-slider").value);
$("#rec-prev").onclick = () => showFrame(Math.max(0, rec.pos - 1));
$("#rec-next").onclick = () => showFrame(Math.min(rec.order.length - 1, rec.pos + 1));
$("#rec-play").onclick = async () => {
  if (rec.playing) { rec.playing = null; $("#rec-play").textContent = "▶ play"; return; }
  const token = rec.playing = {};
  $("#rec-play").textContent = "❚❚ pause";
  while (rec.playing === token && rec.pos < rec.order.length - 1) {  // next frame once the previous one is shown
    const t = Date.now();
    await showFrame(rec.pos + 1);
    await new Promise(r => setTimeout(r, Math.max(0, 200 - (Date.now() - t))));
  }
  if (rec.playing === token) $("#rec-play").click();
};
$("#rec-show-points").onchange = drawFrame;
$("#rec-show-boxes").onchange = drawFrame;
$("#rec-show-geometry").onchange = drawFrame;
function jumpToTime(t) {
  $("#rec-filter").value = $("#rec-filter").options[0].disabled ? "all" : "fused"; applyFilter();
  let best = 0;
  rec.order.forEach((idx, i) => { if (Math.abs(rec.frames[idx].time - t) < Math.abs(rec.frames[rec.order[best]].time - t)) best = i; });
  if (rec.order.length) showFrame(best);
}

let frameReq = 0;
const frameCache = new Map();  // "source|camera|index" -> frame (recorded frames don't change)
function fetchFrame(idx) {
  const key = `${rec.source}|${$("#rec-camera").value}|${idx}`;
  if (!frameCache.has(key)) {
    if (frameCache.size > 40) frameCache.delete(frameCache.keys().next().value);
    frameCache.set(key, api(`/api/recordings/${encodeURIComponent(rec.source)}/frame?camera=${encodeURIComponent($("#rec-camera").value)}&index=${idx}`)
      .catch(e => { frameCache.delete(key); throw e; }));
  }
  return frameCache.get(key);
}
async function showFrame(pos) {
  rec.pos = pos; $("#rec-slider").value = pos;
  const idx = rec.order[pos], req = ++frameReq;
  $("#rec-frame-info").textContent = `${pos + 1}/${rec.order.length} loading…`;
  const f = await fetchFrame(idx);
  for (const p of [pos + 1, pos + 2]) if (p < rec.order.length) fetchFrame(rec.order[p]).catch(() => {});  // prefetch
  if (req !== frameReq) return;  // a newer frame was requested meanwhile
  await renderFrame(f, `${pos + 1}/${rec.order.length} · ${new Date(f.time * 1000).toLocaleTimeString()}`);
  drawTimeline();
}

// Draw a frame (recorded or live); resolves once the image is drawn
async function renderFrame(f, prefix) {
  rec.frame = f;
  $("#rec-frame-info").textContent = `${prefix} · stamp ${f.stamp.toFixed(3)}` +
    (f.scan_image_dt_ms !== undefined ? ` · scan−image ${f.scan_image_dt_ms} ms` : "") +
    (f.cloud_points ? ` · scan ${f.cloud_points} pts, ${f.points.length} in image` : "") + (f.lidar_note ? ` · ${f.lidar_note}` : "");
  if (f.image) {
    const img = new Image();
    await new Promise(r => { img.onload = r; img.onerror = r; img.src = "data:image/jpeg;base64," + f.image; });
    rec.img = img;
  }
  drawFrame(); drawBev(); renderBoxTable();
  $("#rec-events").textContent = f.events.map(e => JSON.stringify(e)).join("\n") + (f.frame_event ? "\n" + JSON.stringify(f.frame_event) : "");
}

// Live view: poll the newest frame while the live source is selected and the tab is visible
async function liveLoop() {
  if (rec.liveRunning) return;
  rec.liveRunning = true;
  try {
    while (rec.source === LIVE_SRC && $("#tab-recordings").classList.contains("active") && !DOMAINS[currentDomain()].map) {
      const t = Date.now();
      try {
        const f = await api(`/api/liveview/frame?${tq()}&camera=${encodeURIComponent($("#rec-camera").value)}`);
        if (rec.source !== LIVE_SRC) break;
        if (f.waiting) { $("#rec-frame-info").textContent = `Live: ${f.waiting}`; }
        else if (f.stamp !== rec.frame?.stamp) {
          await renderFrame(f, `Live · ${f.age_s} s old`);
        }
      } catch (e) { $("#rec-frame-info").textContent = `Live: ${e.message}`; }
      await new Promise(r => setTimeout(r, Math.max(0, 50 - (Date.now() - t))));
    }
  } finally { rec.liveRunning = false; }
}
document.querySelector('#tabs button[data-tab="recordings"]').addEventListener("click", () => { if (rec.source === LIVE_SRC) liveLoop(); });

function outcomeStyle(b) {
  const o = state.descriptions?.components?.bbox_project_pcloud?.outcomes || {};
  if (b.outcome === "detected") return {color: "#1f8a4c", label: "Detected"};
  if (b.outcome === "unknown") return {color: "#94a3b8", label: "No fusion event"};
  const d = o[`${b.outcome}/${b.reason}`] || {};
  return {color: d.color || (b.outcome === "skipped" ? "#64748b" : "#d64545"), label: d.label || `${b.outcome}: ${b.reason}`};
}

function drawFrame() {
  const f = rec.frame, c = $("#rec-image");
  if (!f) return;
  const [w, h] = f.image ? [rec.img.naturalWidth, rec.img.naturalHeight] : (f.image_size || [640, 480]);
  c.width = w; c.height = h;
  const g = c.getContext("2d");
  if (f.image) g.drawImage(rec.img, 0, 0); else { g.fillStyle = "#000"; g.fillRect(0, 0, w, h); }
  if ($("#rec-show-points").checked && f.points.length) {
    const maxR = 40;
    for (const [u, v, r] of f.points) {
      g.fillStyle = `hsl(${Math.min(1, r / maxR) * 240}, 100%, 50%)`;  // red = near, blue = far
      g.fillRect(u - 1.5, v - 1.5, 3, 3);
    }
  }
  if ($("#rec-show-geometry").checked) f.boxes.forEach(b => drawBoxGeometry(g, b));
  if ($("#rec-show-boxes").checked) {
    g.lineWidth = Math.max(2, w / 400); g.font = `${Math.max(12, w / 70)}px system-ui`;
    f.boxes.forEach((b, i) => {
      const s = outcomeStyle(b), [x0, y0, x1, y1] = b.bbox;
      g.strokeStyle = s.color; g.setLineDash(b.outcome === "detected" ? [] : [8, 4]);
      g.strokeRect(x0, y0, x1 - x0, y1 - y0); g.setLineDash([]);
      const t = `${i + 1} ${b.label} · ${s.label}`;
      g.fillStyle = s.color; g.fillRect(x0, y0 - parseInt(g.font) - 4, g.measureText(t).width + 6, parseInt(g.font) + 4);
      g.fillStyle = "#fff"; g.fillText(t, x0 + 3, y0 - 4);
    });
  }
  drawZooms();
}

// Fusion geometry of one buoy box (logged by bbox_project_pcloud when log_geometry is on):
// color contours (filled; kept = solid outline, too small = dashed; selected = thick white) and the
// box's LiDAR points colored by cluster (gray = no cluster; selected cluster ringed in white).
const CLUSTER_COLORS = ["#f59e0b", "#06b6d4", "#d946ef", "#84cc16", "#f43f5e", "#3b82f6", "#a855f7", "#14b8a6"];
// contours: their own colors (C1, C2, ...), distinct from the cluster colors of the points
const CONTOUR_COLORS = ["#facc15", "#fb7185", "#38bdf8", "#a3e635", "#f472b6", "#2dd4bf", "#fdba74", "#c4b5fd"];
const contourColor = k => CONTOUR_COLORS[k % CONTOUR_COLORS.length];
// Convex hull of 2D points (Andrew's monotone chain), counter-clockwise, no repeated end point
function convexHull(points) {
  const p = [...points].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (p.length < 3) return p;
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const half = pts => {
    const h = [];
    for (const q of pts) { while (h.length >= 2 && cross(h[h.length - 2], h[h.length - 1], q) <= 0) h.pop(); h.push(q); }
    h.pop();
    return h;
  };
  return half(p).concat(half([...p].reverse()));
}

function drawBoxGeometry(g, b, px = null, labels = false) {  // px: size of one screen pixel in image px (zooms)
  const d = b.details || {};
  const lw = px ? 2 * px : Math.max(1.5, g.canvas.width / 500);
  (d.contours || []).forEach((poly, k) => {
    if (poly.length < 6) return;
    const kept = (d.contour_area_ratios?.[k] ?? 0) > (d.contour_area_thres ?? 0.5);
    const selected = k === d.selected_contour;
    g.beginPath(); g.moveTo(poly[0], poly[1]);
    for (let i = 2; i < poly.length; i += 2) g.lineTo(poly[i], poly[i + 1]);
    g.closePath();
    const col = contourColor(k);
    g.fillStyle = col + "40"; g.fill();  // 25% alpha
    if (selected) { g.strokeStyle = "#fff"; g.lineWidth = 3.5 * lw; g.stroke(); }
    g.strokeStyle = col; g.lineWidth = selected ? 2 * lw : lw;
    g.setLineDash(kept || selected ? [] : [4 * lw, 3 * lw]); g.stroke(); g.setLineDash([]);
    if (labels) {  // "C1" at the contour's vertex centroid
      let cx = 0, cy = 0;
      for (let i = 0; i < poly.length; i += 2) { cx += poly[i]; cy += poly[i + 1]; }
      cx /= poly.length / 2; cy /= poly.length / 2;
      g.font = `bold ${13 * (px || 1)}px system-ui`; g.textAlign = "center"; g.textBaseline = "middle";
      g.lineWidth = 3 * (px || 1); g.strokeStyle = "#000"; g.strokeText(`C${k + 1}`, cx, cy);
      g.fillStyle = col; g.fillText(`C${k + 1}`, cx, cy);
      g.textAlign = "start"; g.textBaseline = "alphabetic";
    }
  });
  // LiDAR clusters: convex hull of each cluster's projected points (cluster color; selected = white rim)
  const byCluster = {};
  for (const [u, v, c] of d.points || []) if (c >= 0) (byCluster[c] ||= []).push([u, v]);
  for (const [c, pts] of Object.entries(byCluster)) {
    const hull = convexHull(pts), col = CLUSTER_COLORS[c % CLUSTER_COLORS.length];
    if (hull.length < 2) continue;
    g.beginPath(); g.moveTo(...hull[0]); hull.slice(1).forEach(p => g.lineTo(...p)); g.closePath();
    g.fillStyle = col + "33"; g.fill();
    if (+c === d.selected_cluster) { g.strokeStyle = "#fff"; g.lineWidth = 3.5 * lw; g.stroke(); }
    g.strokeStyle = col; g.lineWidth = 1.5 * lw; g.stroke();
  }
  const r = px ? 4 * px : Math.max(2, g.canvas.width / 320);
  for (const [u, v, k] of d.points || []) {
    g.fillStyle = k < 0 ? "#9ca3af" : CLUSTER_COLORS[k % CLUSTER_COLORS.length];
    g.beginPath(); g.arc(u, v, r, 0, 7); g.fill();
    if (k >= 0 && k === d.selected_cluster) { g.strokeStyle = "#fff"; g.lineWidth = px ? 1.5 * px : 1; g.stroke(); }
  }
}

// Enlarged crop of every buoy box (box + 40% margin, image with its contours and box points),
// captioned with what fusion measured: contour / cluster area ratios vs their thresholds.
function drawZooms() {
  const f = rec.frame, S = 220;
  const boxes = f.boxes.map((b, i) => [b, i]).filter(([b]) => b.outcome !== "skipped");
  $("#rec-zooms").innerHTML = boxes.map(([b, i]) => {
    const d = b.details || {}, s = outcomeStyle(b);
    const pct = v => `${Math.round(v * 100)}%`;
    const thres = d.contour_area_thres ?? 0.5;
    const ratios = (d.contour_area_ratios || []).map((r, k) => `<span style="color:${contourColor(k)};font-weight:600"
      title="${r > thres ? "big enough" : "too small"}${k === d.selected_contour ? ", selected" : ""}">C${k + 1} ${pct(r)}${k === d.selected_contour ? "✓" : ""}</span>`).join(" ");
    const sel = "";
    return `<figure><canvas width="${S * 2}" height="${S * 2}" data-i="${i}"></canvas><figcaption>
      <b>${i + 1}</b> · ${esc(b.label)} · <span style="color:${s.color}">${esc(s.label)}</span><br>
      ${d.bbox && d.bbox.some((v, i) => Math.abs(v - b.bbox[i]) > 1.5) ? `<span class="muted">fusion's box (dotted): ${Math.round(d.bbox[2] - d.bbox[0])}×${Math.round(d.bbox[3] - d.bbox[1])} px vs YOLO ${Math.round(b.bbox[2] - b.bbox[0])}×${Math.round(b.bbox[3] - b.bbox[1])} px</span><br>` : ""}
      ${d.contours ? `color region / fusion's box: ${ratios || "none"}${sel} (min ${pct(d.contour_area_thres ?? 0.5)})<br>
      box points: ${d.n_points ?? (d.points || []).length}${d.cluster_area_ratios ? ` · clusters: ${d.cluster_area_ratios.map((r, c) =>
        `<span style="color:${CLUSTER_COLORS[c % CLUSTER_COLORS.length]};font-weight:600">K${c + 1} ${pct(r)}${c === d.selected_cluster ? "✓" : ""}</span>`).join(" ")}` : ""}`
        : '<span class="muted">no geometry logged for this outcome</span>'}</figcaption></figure>`;
  }).join("");
  $("#rec-zooms").querySelectorAll("canvas").forEach(cv => {
    const b = f.boxes[+cv.dataset.i], [x0, y0, x1, y1] = b.bbox;
    // fusion's own box (bbox_object_margin padding), as logged with the outcome: shown dotted
    const fb = b.details?.bbox && b.details.bbox.some((v, i) => Math.abs(v - b.bbox[i]) > 1.5) ? b.details.bbox : null;
    const ex = fb ? [Math.min(x0, fb[0]), Math.min(y0, fb[1]), Math.max(x1, fb[2]), Math.max(y1, fb[3])] : [x0, y0, x1, y1];
    const m = 0.25 * Math.max(ex[2] - ex[0], ex[3] - ex[1], 10), cx = (ex[0] + ex[2]) / 2, cy = (ex[1] + ex[3]) / 2;
    const half = Math.max(ex[2] - ex[0], ex[3] - ex[1]) / 2 + m;
    const g = cv.getContext("2d"), k = cv.width / (2 * half);
    g.imageSmoothingEnabled = false;
    g.setTransform(k, 0, 0, k, -(cx - half) * k, -(cy - half) * k);  // image px -> zoom px
    if (f.image) g.drawImage(rec.img, 0, 0);
    if ($("#rec-show-geometry").checked) drawBoxGeometry(g, b, 1 / k, true);
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.strokeStyle = outcomeStyle(b).color; g.lineWidth = 3; g.setLineDash(b.outcome === "detected" ? [] : [10, 6]);
    g.strokeRect((x0 - cx + half) * k, (y0 - cy + half) * k, (x1 - x0) * k, (y1 - y0) * k);
    if (fb) {
      g.strokeStyle = "#e5e7eb"; g.lineWidth = 2; g.setLineDash([2, 4]);
      g.strokeRect((fb[0] - cx + half) * k, (fb[1] - cy + half) * k, (fb[2] - fb[0]) * k, (fb[3] - fb[1]) * k);
      g.setLineDash([]);
    }
  });
}

function drawBev() {
  const f = rec.frame, c = $("#rec-bev"), g = c.getContext("2d"), W = c.width;
  // ±20 m with a 5 m grid, widened (in 10 m steps, coarser grid) so every detection of the frame fits
  const far = Math.max(0, ...f.boxes.map(b => b.details?.position).filter(Boolean).map(p => Math.max(Math.abs(p[0]), Math.abs(p[1]))));
  const R = far + 2 > 20 ? Math.ceil((far + 2) / 10) * 10 : 20, GRID = R <= 20 ? 5 : R <= 50 ? 10 : 20, k = W / (2 * R);
  $("#rec-bev-caption").textContent = `Top view (base_link, ±${R} m, ${GRID} m grid; forward = up)`;
  const css = getComputedStyle(document.documentElement);
  g.clearRect(0, 0, W, W);
  g.strokeStyle = css.getPropertyValue("--border");
  for (let m = -R; m <= R; m += GRID) {
    g.beginPath(); g.moveTo(0, W / 2 - m * k); g.lineTo(W, W / 2 - m * k); g.stroke();
    g.beginPath(); g.moveTo(W / 2 + m * k, 0); g.lineTo(W / 2 + m * k, W); g.stroke();
  }
  g.fillStyle = css.getPropertyValue("--muted");
  for (const [x, y] of f.bev) g.fillRect(W / 2 - y * k, W / 2 - x * k, 1.5, 1.5);  // x forward = up, y left = left
  for (const b of f.boxes) {
    const p = b.details?.position;
    if (b.outcome !== "detected" || !p) continue;
    g.fillStyle = "#1f8a4c"; g.beginPath(); g.arc(W / 2 - p[1] * k, W / 2 - p[0] * k, 5, 0, 7); g.fill();
  }
  g.fillStyle = "#2557d6"; g.beginPath(); g.moveTo(W / 2, W / 2 - 8); g.lineTo(W / 2 - 5, W / 2 + 6); g.lineTo(W / 2 + 5, W / 2 + 6); g.fill();
}

function renderBoxTable() {
  const f = rec.frame;
  if (!f.boxes.length) { $("#rec-boxes").innerHTML = '<div class="muted">No YOLO boxes in this frame.</div>'; return; }
  $("#rec-boxes").innerHTML = `<table class="outcomes"><tr><th>#</th><th>Label</th><th>Outcome</th><th>Details</th></tr>${f.boxes.map((b, i) => {
    const s = outcomeStyle(b);
    const d = Object.entries(b.details).filter(([k]) => !["bbox", "label", "reason", "contours", "points"].includes(k))
      .map(([k, v]) => `${esc(k)}=${esc(Array.isArray(v) ? v.map(x => typeof x === "number" ? +x.toFixed(2) : x).join(",") : v)}`).join(" ");
    return `<tr><td>${i + 1}</td><td>${esc(b.label)} <span class="muted">${!b.probability ? "" : (b.probability * 100).toFixed(0) + "%"}</span></td>
      <td><span class="swatch" style="background:${s.color}"></span>${esc(s.label)}</td><td class="muted">${d}</td></tr>`;
  }).join("")}</table>`;
}

// ---------------------------------------------------------------- analysis
async function runRecAnalysis() {
  if (!rec.source) return;
  if (!state.descriptions) { try { state.descriptions = await api("/api/descriptions"); } catch { state.descriptions = {}; } }
  const analyzer = $("#rec-analyzer").value, r = rec.range;
  $("#rec-report-info").textContent = "running…";
  try {
    const rep = await api(`/api/recordings/${encodeURIComponent(rec.source)}/report?analyzer=${analyzer}` +
                          (r ? `&t0=${r[0]}&t1=${r[1]}` : ""));
    $("#rec-report-info").textContent = `${rep.entries} log entries${r ? " in the selected range" : ""}`;
    renderReport($("#rec-report"), rep, state.descriptions);
    const plots = $("#rec-plots"); plots.innerHTML = "";
    rep.plots.forEach(p => renderPlot(plots, stylePlot(analyzer, p, state.descriptions)));
  } catch (e) { $("#rec-report-info").textContent = e.message; }
  refreshRecMap();  // domains.js: SLAM / Tasks map of the source / range
}
$("#rec-analyze").onclick = runRecAnalysis;
$("#rec-analyzer").onchange = runRecAnalysis;

// ---------------------------------------------------------------- rosbags on the target (bag_import.py)
const IMPORT_PHASES = {convert: "making a light copy on the target", download: "downloading", mcap: "writing the recording"};
async function loadBags() {
  $("#bags").innerHTML = '<div class="muted">loading…</div>';
  let bags;
  try { bags = await api(`/api/bags?${tq()}`); } catch (e) { $("#bags").innerHTML = `<div class="bad">${esc(e.message)}</div>`; return; }
  $("#bags").innerHTML = bags.length ? bags.map(b => `<div class="bag-row"><div><b>${esc(b.name)}</b>
      <div class="muted">${fmtBytes(b.bytes)} · ${new Date(b.mtime * 1000).toLocaleDateString()}</div></div>
      <button class="small" data-bag="${esc(b.path)}">view</button></div>`).join("")
    : '<div class="muted">No bags in the target\'s bags folder.</div>';
  $("#bags").querySelectorAll("[data-bag]").forEach(btn => btn.onclick = async () => {
    try {
      await api("/api/recordings/import", {method: "POST", body: {bag: btn.dataset.bag, target: state.target,
        image_rate: +$("#bag-image-rate").value, cloud_rate: +$("#bag-cloud-rate").value}});
    } catch (e) { return alert(e.message); }
    pollImports();
  });
}
$("#bags-refresh").onclick = loadBags;

let importsTimer = null, seenDone = new Set();
async function pollImports() {
  let jobs;
  try { jobs = await api("/api/recordings/imports"); } catch { return; }
  const shown = jobs.filter(j => j.status !== "done" || Date.now() / 1000 - j.started < 600).slice(0, 5);
  $("#bag-imports").innerHTML = shown.map(j => `<div class="card" style="padding:8px">
      <b>${esc(j.name)}</b> <span class="badge ${j.status === "error" ? "fail" : j.status === "done" ? "ok" : "running"}">${j.status}</span>
      <div class="progress"><div style="width:${Math.round(j.progress * 100)}%"></div></div>
      <div class="muted">${j.status === "running" ? `${Math.round(j.progress * 100)}% · ${IMPORT_PHASES[j.phase]}`
        : j.status === "done" ? `saved as ${esc(j.result)}` : `<pre class="bad" style="white-space:pre-wrap">${esc(j.error)}</pre>`}</div></div>`).join("");
  for (const j of jobs) if (j.status === "done" && !seenDone.has(j.id)) {
    const fresh = seenDone.size > 0 || Date.now() / 1000 - j.started < 600;
    seenDone.add(j.id);
    if (fresh && importsTimer) loadRecordings(j.result);  // open it when it finishes while we watch
  }
  const running = jobs.some(j => j.status === "running");
  clearTimeout(importsTimer);
  importsTimer = running ? setTimeout(pollImports, 1000) : null;
}
document.querySelector('#tabs button[data-tab="recordings"]').addEventListener("click", () => { loadBags(); pollImports(); });
$("#target-select").addEventListener("change", () => { if ($("#tab-recordings").classList.contains("active")) loadBags(); });
