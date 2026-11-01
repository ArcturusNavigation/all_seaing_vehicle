// Domain panels (LiDAR/Camera, SLAM, Tasks) in "Runs & analysis" and "Recordings": one shared choice
// (remembered), which filters each tab's analyzers and shows the map player (mapview.js) for SLAM and
// Tasks. Map data: /api/runs/<run>/view, /api/live/view, /api/recordings/<source>/view (analyzer view()).

const DOMAINS = {
  lidar: {analyzers: ["fusion", "events"]},
  slam: {analyzers: ["slam", "events"], map: "slam"},
  tasks: {analyzers: ["tasks", "events"], map: "tasks"},
};
let labelNames = null;
const runMap = new MapPlayer($("#run-map"));
const recMap = new MapPlayer($("#rec-map"));
let runMapKey = null, recMapKey = null, mapBusy = {run: false, rec: false};

function currentDomain() {
  let d = "lidar";
  try { d = localStorage.getItem("domain") || "lidar"; } catch {}
  return DOMAINS[d] ? d : "lidar";
}

// show only the analyzers of the domain in a <select>; keep the current one if it belongs to it
function filterAnalyzers(sel) {
  const allowed = DOMAINS[currentDomain()].analyzers;
  [...sel.options].forEach(o => { o.hidden = !allowed.includes(o.value); });
  if (sel.options.length && !allowed.includes(sel.value)) sel.value = allowed.find(a => [...sel.options].some(o => o.value === a)) || sel.value;
}

function applyDomain(refresh = true) {
  const d = currentDomain();
  document.body.classList.remove("domain-lidar", "domain-slam", "domain-tasks");
  document.body.classList.add(`domain-${d}`);
  document.querySelectorAll(".domain-bar button").forEach(b => b.classList.toggle("active", b.dataset.domain === d));
  filterAnalyzers($("#analyzer-select"));
  filterAnalyzers($("#rec-analyzer"));
  if (!refresh) return;
  if ($("#tab-runs").classList.contains("active") && state.run) refreshReport();
  if ($("#tab-recordings").classList.contains("active") && rec.source) {
    if (rec.source === LIVE_SRC && !DOMAINS[d].map) liveLoop();
    else runRecAnalysis();
  }
}
document.querySelectorAll(".domain-bar button").forEach(b => b.onclick = () => {
  try { localStorage.setItem("domain", b.dataset.domain); } catch {}
  runMapKey = recMapKey = null;
  applyDomain();
});

async function ensureLabels() {
  if (!labelNames) { try { labelNames = await api("/api/labels"); } catch { labelNames = {}; } }
  return labelNames;
}

// Runs & analysis: map of the selected run (reloaded when run/domain change) or live (every refresh)
async function refreshRunMap() {
  const m = DOMAINS[currentDomain()].map;
  if (!m || !state.run || mapBusy.run) return;
  const live = state.run === LIVE, key = `${state.target}|${state.run}|${m}`;
  if (!live && key === runMapKey) return;
  mapBusy.run = true;
  if (!live) $("#run-map-info").textContent = "loading…";
  try {
    const view = live ? await api(`/api/live/view?${tq()}&analyzer=${m}&window=${$("#live-window").value}`)
                      : await api(`/api/runs/${encodeURIComponent(state.run)}/view?${tq()}&analyzer=${m}`);
    runMap.follow = live;
    runMap.setData(view, await ensureLabels());
    runMapKey = key;
    $("#run-map-info").textContent = view.frames.length ? `${view.node || ""} · ${view.frames.length} frames` : "";
  } catch (e) { $("#run-map-info").textContent = e.message; }
  mapBusy.run = false;
}

// Recordings: map of the source (or the selected range), or live
async function refreshRecMap() {
  const m = DOMAINS[currentDomain()].map;
  if (!m || !rec.source || mapBusy.rec) return;
  const live = rec.source === LIVE_SRC, r = rec.range;
  const key = `${rec.source}|${m}|${r ? r.join(",") : ""}`;
  if (!live && key === recMapKey) return;
  mapBusy.rec = true;
  if (!live) $("#rec-map-info").textContent = "loading…";
  try {
    const view = live ? await api(`/api/live/view?${tq()}&analyzer=${m}&window=120`)
                      : await api(`/api/recordings/${encodeURIComponent(rec.source)}/view?analyzer=${m}` + (r ? `&t0=${r[0]}&t1=${r[1]}` : ""));
    recMap.follow = live;
    recMap.setData(view, await ensureLabels());
    recMapKey = key;
    $("#rec-map-info").textContent = view.frames.length ? `${view.node || ""} · ${view.frames.length} frames${r ? " in the selected range" : ""}` : "";
  } catch (e) { $("#rec-map-info").textContent = e.message; }
  mapBusy.rec = false;
}
// live maps: refresh as soon as the previous refresh finished (no fixed period), in both tabs
async function liveMapLoop() {
  for (;;) {
    const m = DOMAINS[currentDomain()].map;
    const t = Date.now();
    if (m && rec.source === LIVE_SRC && $("#tab-recordings").classList.contains("active")) await refreshRecMap();
    else if (m && state.run === LIVE && $("#tab-runs").classList.contains("active")) await refreshRunMap();
    await new Promise(r => setTimeout(r, Math.max(50, 100 - (Date.now() - t))));
  }
}
liveMapLoop();

applyDomain(false);
