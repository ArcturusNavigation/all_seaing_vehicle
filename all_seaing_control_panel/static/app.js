// Arcturus Control Panel UI. Talks to the FastAPI backend (server.py) under /api.

const $ = sel => document.querySelector(sel);
const esc = s => String(s ?? "").replace(/[&<>"]/g, c => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;"}[c]));
const state = {mode: localStorage.getItem("mode") || "auto", detectedMode: null, target: null, registry: null, selectedAction: null, logProc: null, logOffset: 0, run: null, catalog: null};

async function api(path, opts = {}) {
  const res = await fetch(path, {headers: {"Content-Type": "application/json"}, ...opts,
                                 body: opts.body !== undefined ? JSON.stringify(opts.body) : undefined});
  if (!res.ok) throw new Error((await res.text()) || res.statusText);
  return res.json();
}
const tq = () => `target_name=${encodeURIComponent(state.target)}`;
function fmtTime(t) { return new Date(t * 1000).toLocaleString(); }

// ---------------------------------------------------------------- tabs & target
document.querySelectorAll("#tabs button").forEach(b => b.onclick = () => {
  document.querySelectorAll("#tabs button").forEach(x => x.classList.toggle("active", x === b));
  document.querySelectorAll(".tab").forEach(t => t.classList.toggle("active", t.id === "tab-" + b.dataset.tab));
  history.replaceState(null, "", "#" + b.dataset.tab);  // bookmarkable tabs, e.g. /#health
  ({runs: loadRuns, params: loadParams, checks: renderCheckList, recordings: () => loadRecordings()})[b.dataset.tab]?.();
});

async function init() {
  const info = await api("/api/info");
  const sel = $("#target-select");
  sel.innerHTML = info.targets.map(t => `<option value="${t.name}">${esc(t.label)}</option>`).join("");
  state.target = localStorage.getItem("target") || info.default_target;
  if (!info.targets.some(t => t.name === state.target)) state.target = info.default_target;
  sel.value = state.target;
  sel.onchange = () => { state.target = sel.value; localStorage.setItem("target", sel.value); checkTarget(); loadRuns(); };
  $("#links").innerHTML = info.links.map(l => `<a href="${esc(l.url)}" target="_blank">${esc(l.title)}</a>`).join("");
  $("#mode-select").value = state.mode;
  $("#mode-select").onchange = () => { state.mode = $("#mode-select").value; localStorage.setItem("mode", state.mode); detectMode(); };
  state.registry = await api("/api/registry");
  renderActions();
  checkTarget();
  setInterval(checkTarget, 15000);
  detectMode();
  setInterval(detectMode, 15000);
  refreshProcs();
  setInterval(refreshProcs, 4000);
  setInterval(pollLog, 1000);
  setInterval(refreshDiagnostics, 1000);
  renderCheckList();
  const [tab, sub] = location.hash.slice(1).split("/");  // e.g. #health, #runs/live
  if (tab) document.querySelector(`#tabs button[data-tab="${tab}"]`)?.click();
  if (tab === "runs" && sub === "live") selectRun(LIVE);
}

async function checkTarget() {
  const dot = $("#target-dot");
  try {
    const s = await api(`/api/targets/${state.target}/status`);
    dot.className = "dot " + (s.reachable ? "ok" : "fail");
    dot.title = s.reachable ? `connected: ${s.hostname}` : `unreachable: ${s.error}`;
  } catch (e) { dot.className = "dot fail"; dot.title = e.message; }
}

// Run mode (live boat / rosbag replay / simulation): detected from the ROS graph unless forced
const MODE_LABELS = {live: "live boat", replay: "rosbag replay", sim: "simulation"};
const effectiveMode = () => state.mode === "auto" ? (state.detectedMode || "auto") : state.mode;
async function detectMode() {
  try { state.detectedMode = (await api(`/api/mode?${tq()}`)).mode; } catch { state.detectedMode = null; }
  const m = effectiveMode(), badge = $("#mode-detected");
  badge.textContent = (state.mode === "auto" ? "detected: " : "") + (MODE_LABELS[m] || "unknown");
  badge.className = "badge " + (m === "live" ? "ok" : m === "replay" ? "warn" : "unknown");
  const banner = $("#mode-banner");
  banner.classList.toggle("hidden", m === "live" || m === "auto");
  banner.innerHTML = m === "replay"
    ? "<b>Rosbag replay:</b> sensor topics come from the bag player, nodes use the bag's /clock. Hardware checks are skipped; " +
      "rates reflect the replay (real-time only at rate 1.0)."
    : m === "sim" ? "<b>Simulation:</b> sensors come from Gazebo; hardware checks are skipped." : "";
}

// ---------------------------------------------------------------- run page
function renderActions() {
  const filter = $("#action-filter").value.toLowerCase();
  const {categories, actions} = state.registry;
  $("#actions").innerHTML = categories.map(cat => {
    const items = actions.filter(a => a.category === cat.id &&
      (a.title + " " + (a.description || "")).toLowerCase().includes(filter));
    if (!items.length) return "";
    return `<div class="cat-title">${esc(cat.title)}</div>` + items.map(a =>
      `<div class="action ${state.selectedAction === a.id ? "selected" : ""}" data-id="${a.id}">${esc(a.title)}
         <span class="target-tag">${esc(a.target || "")}${a.mode === "blocking" ? " · quick" : ""}</span></div>`).join("");
  }).join("");
  document.querySelectorAll(".action").forEach(el => el.onclick = () => selectAction(el.dataset.id));
}
$("#action-filter").oninput = renderActions;

async function choicesFor(kind) {
  try { return await api(`/api/choices/${kind}?${tq()}`); } catch { return []; }
}

async function selectAction(id) {
  state.selectedAction = id;
  renderActions();
  const a = state.registry.actions.find(x => x.id === id);
  const form = $("#action-form");
  form.classList.remove("hidden");
  const fields = [];
  for (const arg of a.args || []) {
    const name = `arg-${arg.name}`, d = arg.default ?? "";
    let input;
    if (arg.type === "bool") input = `<input type="checkbox" id="${name}" ${d === true || d === "true" ? "checked" : ""}>`;
    else if (arg.type === "choice") input = `<select id="${name}">${arg.choices.map(c =>
      `<option ${String(c) === String(d) ? "selected" : ""}>${esc(c)}</option>`).join("")}</select>`;
    else if (arg.type === "bag" || arg.type === "run") {
      const opts = await choicesFor(arg.type);
      input = `<select id="${name}">${opts.map(o => `<option>${esc(o)}</option>`).join("")}</select>`;
    } else input = `<input id="${name}" value="${esc(d)}" ${arg.type === "number" ? 'type="number" step="any"' : ""}>`;
    fields.push(`<label for="${name}" title="${esc(arg.help || "")}">${esc(arg.name)}</label>${input}`);
  }
  const target = a.target || state.target;
  form.innerHTML = `
    <div class="card-title">${esc(a.title)}</div>
    ${a.description ? `<div class="muted">${esc(a.description)}</div>` : ""}
    <div class="form-grid">${fields.join("")}
      <label>target</label><select id="arg-__target">${[...$("#target-select").options].map(o =>
        `<option value="${o.value}" ${o.value === target ? "selected" : ""}>${esc(o.text)}</option>`).join("")}</select>
      ${a.sudo ? `<label>sudo password</label><input id="arg-__sudo" type="password" autocomplete="off">` : ""}
      ${a.mode !== "blocking" && !a.sudo ? `<label title="Also record a rosbag with this preset; it stops when this process stops">record rosbag</label>
        <select id="arg-__record"><option value="">no</option>${(state.rosbagPresets || []).map(p =>
          `<option value="${esc(p.id)}">${esc(p.title)}</option>`).join("")}</select>` : ""}
    </div>
    <pre id="cmd-preview"></pre>
    <div class="toolbar" style="margin-top:8px"><button id="start-action">${a.mode === "blocking" ? "Run" : "Start"}</button></div>
    <pre id="blocking-output" class="hidden"></pre>`;
  const values = () => Object.fromEntries((a.args || []).map(arg => {
    const el = $(`#arg-${arg.name}`);
    return [arg.name, arg.type === "bool" ? el.checked : el.value];
  }));
  const preview = async () => {
    const r = await api(`/api/actions/${id}/preview`, {method: "POST", body: {values: values(), target: $("#arg-__target").value}});
    $("#cmd-preview").textContent = r.command;
  };
  form.querySelectorAll("input,select").forEach(el => { if (el.id !== "arg-__record") el.oninput = el.onchange = preview; });
  preview();
  $("#start-action").onclick = async () => {
    const btn = $("#start-action");
    btn.disabled = true;
    try {
      const body = {values: values(), target: $("#arg-__target").value};
      if (a.sudo) body.sudo_password = $("#arg-__sudo").value;
      if ($("#arg-__record")?.value) body.record_preset = $("#arg-__record").value;
      const r = await api(`/api/actions/${id}/start`, {method: "POST", body});
      if (r.blocking) {
        const out = $("#blocking-output");
        out.classList.remove("hidden");
        out.textContent = `$ ${r.command || ""}\n${r.output}\n[exit code ${r.returncode}]`;
      } else { refreshProcs(); refreshRosbags(); showLog(r.id, r.title); }
    } catch (e) { alert(e.message); }
    btn.disabled = false;
  };
}

async function refreshProcs() {
  let procs;
  try { procs = await api("/api/processes"); } catch { return; }
  procs = procs.filter(p => p.status === "running" || p.status === "stopping");  // ended processes aren't listed
  $("#procs tbody").innerHTML = procs.slice(0, 30).map(p => `
    <tr class="clickable" data-id="${p.id}" data-title="${esc(p.title)}">
      <td>${new Date(p.started * 1000).toLocaleTimeString()}</td>
      <td>${esc(p.title)}<div class="muted">${esc(p.cmd)}</div></td>
      <td>${esc(p.target)}</td>
      <td><span class="badge ${p.status}">${p.status}</span></td>
      <td>${p.status === "running" ? `<button class="small danger" data-stop="${p.id}">stop</button>`
           : p.status === "stopping" ? "" : `<button class="small secondary" data-remove="${p.id}">×</button>`}</td>
    </tr>`).join("");
  document.querySelectorAll("#procs tr.clickable").forEach(tr => tr.onclick = e => {
    if (e.target.dataset.stop) { api(`/api/processes/${e.target.dataset.stop}/stop`, {method: "POST"}).then(refreshProcs); return; }
    if (e.target.dataset.remove) { api(`/api/processes/${e.target.dataset.remove}`, {method: "DELETE"}).then(refreshProcs); return; }
    showLog(tr.dataset.id, tr.dataset.title);
  });
}
$("#refresh-procs").onclick = refreshProcs;

function showLog(id, title) {
  state.logProc = id; state.logOffset = 0;
  $("#log").textContent = "";
  $("#log-title").textContent = title;
  $("#log-card").classList.remove("hidden");
  pollLog();
}

async function pollLog() {
  if (!state.logProc) return;
  try {
    const r = await api(`/api/processes/${state.logProc}/log?offset=${state.logOffset}`);
    if (r.text) {
      const log = $("#log");
      log.textContent += r.text;
      if (log.textContent.length > 400000) log.textContent = log.textContent.slice(-300000);
      if ($("#log-follow").checked) log.scrollTop = log.scrollHeight;
    }
    state.logOffset = r.offset;
  } catch {}
}

// ---------------------------------------------------------------- health (rendering in health.js)
async function refreshDiagnostics() {
  if (!$("#tab-health").classList.contains("active")) return;
  if (!state.descriptions) {
    try { state.descriptions = await api("/api/descriptions"); } catch { state.descriptions = {}; }
  }
  let diags;
  try { diags = await api("/api/diagnostics"); } catch { return; }
  const counts = {OK: 0, WARN: 0, ERROR: 0, STALE: 0};
  diags.forEach(d => counts[d.level] = (counts[d.level] || 0) + 1);
  $("#diag-summary").innerHTML = `<div class="toolbar">${Object.entries(counts).map(([l, n]) =>
    `<span class="badge ${l}">${LEVEL_TEXT[l]}: ${n}</span>`).join(" ")}
    ${diags.length ? "" : '<span class="muted">No /diagnostics received (is anything running? same ROS_DOMAIN_ID?)</span>'}</div>`;
  renderHealth($("#diagnostics"), diags, state.descriptions,
               {filter: $("#diag-filter").value.toLowerCase(), problems: $("#diag-problems").checked});
}

// ---------------------------------------------------------------- checks
async function renderCheckList() {
  const list = await api("/api/checks");
  const box = $("#checks");
  if (box.dataset.loaded) return;
  box.dataset.loaded = 1;
  box.innerHTML = list.map(c => `
    <div class="card check" id="check-${c.id}">
      <span class="badge unknown" data-badge>not run</span>
      <div><b>${esc(c.title)}</b><div class="check-desc">${esc(c.description || "")}</div>
        <div data-msg></div><div data-details class="details"></div></div>
      <div><button class="small secondary" data-run="${c.id}">run</button> <span data-fix></span></div>
    </div>`).join("");
  box.querySelectorAll("[data-run]").forEach(b => b.onclick = () => runCheck(b.dataset.run));
}

async function runCheck(id) {
  const el = $(`#check-${id}`);
  el.querySelector("[data-badge]").className = "badge unknown";
  el.querySelector("[data-badge]").textContent = "running...";
  try {
    const r = await api(`/api/checks/${id}?${tq()}&mode=${effectiveMode()}`, {method: "POST"});
    el.querySelector("[data-badge]").className = "badge " + r.status;
    el.querySelector("[data-badge]").textContent = r.status;
    el.querySelector("[data-msg]").textContent = r.message;
    el.querySelector("[data-details]").innerHTML = Object.entries(r.details || {}).map(([k, v]) =>
      `${esc(k)}: ${esc(typeof v === "object" ? JSON.stringify(v) : v)}`).join("<br>");
    const fix = el.querySelector("[data-fix]");
    fix.innerHTML = r.fix ? `<button class="small" data-fix-action="${r.fix}">fix</button>` : "";
    if (r.fix) fix.querySelector("button").onclick = () => {
      document.querySelector('#tabs button[data-tab="run"]').click();
      selectAction(r.fix);
    };
  } catch (e) {
    el.querySelector("[data-badge]").className = "badge fail";
    el.querySelector("[data-badge]").textContent = "error";
    el.querySelector("[data-msg]").textContent = e.message;
  }
}
let checksRunning = false;
async function runAllChecks() {
  if (checksRunning) return;  // never overlap rounds
  checksRunning = true;
  try {
    await Promise.all((await api("/api/checks")).map(c => runCheck(c.id)));
    $("#checks-last").textContent = "last run " + new Date().toLocaleTimeString();
  } finally { checksRunning = false; }
}
$("#run-all-checks").onclick = runAllChecks;
// auto-run: a new round starts `interval` s after the previous one finished
(function autoChecks() {
  const tick = async () => {
    if ($("#checks-auto").checked) await runAllChecks();
    setTimeout(tick, Math.max(5, +$("#checks-interval").value || 15) * 1000);
  };
  $("#checks-auto").checked = localStorage.getItem("checksAuto") === "1";
  $("#checks-interval").value = localStorage.getItem("checksInterval") || 15;
  $("#checks-auto").onchange = () => { localStorage.setItem("checksAuto", $("#checks-auto").checked ? "1" : "0"); if ($("#checks-auto").checked) runAllChecks(); };
  $("#checks-interval").onchange = () => localStorage.setItem("checksInterval", $("#checks-interval").value);
  setTimeout(tick, 1000);
})();

// ---------------------------------------------------------------- runs & analysis (past runs + live)
// "Live" is the run happening now: entries streamed from /run_log through foxglove_bridge.
const LIVE = "__live__";

async function loadRuns() {
  if (!$("#tab-runs").classList.contains("active")) return;
  const liveRow = `<tr class="clickable run-item live-item ${state.run === LIVE ? "selected" : ""}" data-run="${LIVE}">
    <td>● Live <span class="muted">(streaming from ${esc(state.target)})</span></td></tr>`;
  try {
    const runs = await api(`/api/runs?${tq()}`);
    $("#runs").innerHTML = `<table>${liveRow}${runs.map(r => `
      <tr class="clickable run-item ${state.run === r.name ? "selected" : ""}" data-run="${esc(r.name)}"><td>${esc(r.name)}
        <div class="muted">${r.files} node log(s), ${(r.size_kb / 1024).toFixed(1)} MB</div></td></tr>`).join("")}</table>`;
  } catch (e) { $("#runs").innerHTML = `<table>${liveRow}</table><div class="muted">${esc(e.message)}</div>`; }
  document.querySelectorAll("#runs tr").forEach(tr => tr.onclick = () => selectRun(tr.dataset.run));
}
$("#refresh-runs").onclick = loadRuns;

async function selectRun(run) {
  state.run = run;
  const live = run === LIVE;
  document.querySelectorAll("#runs tr").forEach(tr => tr.classList.toggle("selected", tr.dataset.run === run));
  $("#tab-runs").classList.toggle("live", live);
  $("#run-detail").classList.remove("hidden");
  $("#run-name").textContent = live ? "Live" : run;
  $("#live-status").innerHTML = "";
  ["#report", "#report-plots", "#plots"].forEach(id => $(id).innerHTML = "");
  $("#report-text").textContent = $("#analysis-output").textContent = "";
  if (!state.descriptions) { try { state.descriptions = await api("/api/descriptions"); } catch { state.descriptions = {}; } }
  const sel = $("#analyzer-select");
  if (!sel.options.length) {
    sel.innerHTML = (await api("/api/analyzers")).map(a => `<option value="${a.name}" title="${esc(a.description)}">${esc(a.name)}</option>`).join("");
    if ([...sel.options].some(o => o.value === "fusion")) sel.value = "fusion";
  }
  if (typeof filterAnalyzers === "function") filterAnalyzers(sel);  // domains.js: the panel's analyzers
  if (!live) {
    state.catalog = await api(`/api/runs/${encodeURIComponent(run)}/catalog?${tq()}`);
    fillPlotMenus();
  }
  refreshReport();
}

let reportBusy = false;
async function refreshReport() {
  if (!state.run || reportBusy) return;
  reportBusy = true;
  const live = state.run === LIVE, analyzer = $("#analyzer-select").value;
  try {
    if (live) {
      const st = await api(`/api/live/status?${tq()}`);
      $("#live-status").innerHTML = st.subscribed ? `<span class="badge ok">receiving</span> <span class="muted">${st.entries.toLocaleString()} entries buffered</span>`
        : st.connected ? `<span class="badge warn">bridge connected, no /run_log yet</span>`
        : `<span class="badge fail">bridge not reachable</span> <span class="muted">${esc(st.url)} ${esc(st.error)}</span>`;
    }
    const r = live ? await api(`/api/live/analyze?${tq()}&analyzer=${analyzer}&window=${$("#live-window").value}`)
                   : await api(`/api/runs/${encodeURIComponent(state.run)}/report?${tq()}&analyzer=${analyzer}`);
    $("#report-info").textContent = `${r.entries.toLocaleString()} log entries` + (live ? ` in the last ${$("#live-window").value} s` : "");
    renderReport($("#report"), r, state.descriptions);
    $("#report-text").textContent = r.output;
    const plots = $("#report-plots");
    plots.innerHTML = "";
    r.plots.forEach(p => renderPlot(plots, stylePlot(analyzer, p, state.descriptions)));
    if (live && !$("#feed-pause").checked) await refreshFeed();
  } catch (e) { $("#report").innerHTML = `<div class="bad">${esc(e.message)}</div>`; }
  reportBusy = false;
  if (typeof refreshRunMap === "function") refreshRunMap();  // domains.js: SLAM / Tasks map
}
$("#run-analyzer").onclick = refreshReport;
$("#analyzer-select").onchange = refreshReport;
$("#live-window").onchange = refreshReport;
// live report + log feed: refresh as soon as the previous refresh finished (no fixed period)
(async function liveReportLoop() {
  for (;;) {
    const t = Date.now();
    if ($("#tab-runs").classList.contains("active") && state.run === LIVE && $("#live-auto").checked) await refreshReport();
    await new Promise(r => setTimeout(r, Math.max(50, 200 - (Date.now() - t))));
  }
})();

async function refreshFeed() {
  const q = new URLSearchParams({level: $("#feed-level").value, node: $("#feed-node").value, event: $("#feed-event").value,
                                 grep: $("#feed-grep").value, window: $("#live-window").value, limit: 200});
  const feed = await api(`/api/live/entries?${tq()}&${q}`);
  $("#live-feed").innerHTML = feed.reverse().map(e => {
    const fields = Object.fromEntries(Object.entries(e).filter(([k]) => !["t", "wall", "node", "level", "event", "_rel"].includes(k)));
    return `<div class="${e.level}" title="${esc(JSON.stringify(fields))}"><span class="lvl">${e.level}</span>
      ${e.t ? e.t.toFixed(2) : ""} <b>${esc(e.node)}</b> ${esc(e.event)} <span class="muted">${esc(JSON.stringify(fields))}</span></div>`;
  }).join("") || '<div class="muted">no matching entries</div>';
}

// past runs: command-line analysis and custom plots
function fillPlotMenus() {
  const cat = state.catalog || {};
  const nodeSel = $("#plot-node"), evSel = $("#plot-event");
  nodeSel.innerHTML = Object.keys(cat).map(n => `<option>${esc(n)}</option>`).join("");
  const fillFields = () => {
    const f = (cat[nodeSel.value] || {})[evSel.value] || {numeric: [], string: []};
    $("#plot-field").innerHTML = f.numeric.map(x => `<option>${esc(x)}</option>`).join("");
    $("#plot-x").innerHTML = $("#plot-y").innerHTML = f.numeric.map(x => `<option>${esc(x)}</option>`).join("");
    $("#plot-state").innerHTML = f.string.map(x => `<option>${esc(x)}</option>`).join("");
  };
  const fillEvents = () => {
    evSel.innerHTML = Object.keys(cat[nodeSel.value] || {}).map(e => `<option>${esc(e)}</option>`).join("");
    fillFields();
  };
  nodeSel.onchange = fillEvents; evSel.onchange = fillFields;
  fillEvents();
}

async function cliAnalyze(extra) {
  $("#analysis-output").textContent = "running...";
  const r = await api(`/api/runs/${encodeURIComponent(state.run)}/analyze?${tq()}&analyzers=&extra=${encodeURIComponent(extra)}`);
  $("#analysis-output").textContent = r.output;
}
$("#run-stats").onclick = () => cliAnalyze("--stats");
$("#run-extra").onclick = () => cliAnalyze($("#analysis-extra").value);

async function plot(spec) {
  $("#plots").innerHTML = '<div class="muted">loading...</div>';
  try {
    const plots = await api(`/api/runs/${encodeURIComponent(state.run)}/plots?${tq()}`, {method: "POST", body: spec});
    $("#plots").innerHTML = plots.length ? "" : '<div class="muted">nothing to plot</div>';
    plots.forEach(p => renderPlot($("#plots"), p));
  } catch (e) { $("#plots").textContent = e.message; }
}
$("#plot-rates").onclick = () => plot({kind: "event_rates"});
$("#plot-field-btn").onclick = () => plot({kind: "field", node: $("#plot-node").value, event: $("#plot-event").value,
  fields: [...$("#plot-field").selectedOptions].map(o => o.value)});
$("#plot-timeline-btn").onclick = () => plot({kind: "timeline", node: $("#plot-node").value, event: $("#plot-event").value,
  field: $("#plot-state").value});
$("#plot-xy-btn").onclick = () => plot({kind: "xy", series: [{node: $("#plot-node").value, event: $("#plot-event").value,
  x: $("#plot-x").value, y: $("#plot-y").value}]});

// ---------------------------------------------------------------- parameters
async function loadParams() {
  $("#params").innerHTML = '<div class="muted">loading (queries running nodes)...</div>';
  let params;
  try { params = await api(`/api/params?${tq()}`); } catch (e) { $("#params").textContent = e.message; return; }
  const groups = {};
  params.forEach(p => (groups[p.group] = groups[p.group] || []).push(p));
  $("#params").innerHTML = Object.entries(groups).map(([g, ps]) => `
    <div class="card param-group"><div class="card-title">${esc(g)}</div>
    <table><thead><tr><th>Parameter</th><th>Launch file</th><th>Live</th><th>Saved</th><th>New value</th><th></th></tr></thead>
    <tbody>${ps.map(p => `
      <tr class="param-row"><td><b>${esc(p.param)}</b> <span class="muted">${esc(p.node)}</span>
          <div class="muted">${esc(p.description || "")}${p.applies === "restart" ? " · applies after restart" : ""}</div></td>
        <td><code>${esc(p.launch_value ?? "not found")}</code></td>
        <td><code>${esc(p.live_value ?? "not running")}</code></td>
        <td>${p.saved_value !== null && p.saved_value !== undefined ? `<code>${esc(JSON.stringify(p.saved_value))}</code>
             <button class="small secondary" data-clear="${p.id}">×</button>` : ""}</td>
        <td><input data-input="${p.id}" placeholder="${esc(p.type)}"></td>
        <td><button class="small" data-set="${p.id}">set</button></td></tr>`).join("")}</tbody></table></div>`).join("");
  document.querySelectorAll("[data-set]").forEach(b => b.onclick = async () => {
    const value = document.querySelector(`[data-input="${b.dataset.set}"]`).value;
    if (!value) return;
    try {
      const r = await api(`/api/params/${b.dataset.set}?${tq()}`, {method: "POST", body: {value}});
      if (r.live_result) alert(`Saved ${JSON.stringify(r.saved)}.\nLive: ${r.live_result}`);
      loadParams();
    } catch (e) { alert(e.message); }
  });
  document.querySelectorAll("[data-clear]").forEach(b => b.onclick = async () => {
    await api(`/api/params/${b.dataset.clear}?${tq()}`, {method: "DELETE"});
    loadParams();
  });
}
$("#refresh-params").onclick = loadParams;
$("#preview-apply").onclick = async () => {
  const box = $("#apply-preview");
  box.classList.remove("hidden");
  box.innerHTML = '<div class="muted">computing changes...</div>';
  const r = await api(`/api/params-apply/preview?${tq()}`);
  const diffs = Object.entries(r.diffs);
  box.innerHTML = `<div class="card-title">Changes to the launch files on ${esc(state.target)}</div>
    ${r.problems.map(p => `<div class="badge fail">${esc(p)}</div>`).join(" ")}
    ${diffs.length ? diffs.map(([f, d]) => `<pre>${d.split("\n").map(l =>
      `<span class="${l.startsWith("+") ? "diff-add" : l.startsWith("-") ? "diff-del" : ""}">${esc(l)}</span>`).join("\n")}</pre>`).join("")
      : '<div class="muted">No changes: the saved values already match the launch files.</div>'}
    <div class="toolbar" style="margin-top:8px">${diffs.length ? '<button id="do-apply">Write launch files</button>' : ""}
      <button class="secondary" id="cancel-apply">Close</button></div>`;
  $("#cancel-apply").onclick = () => box.classList.add("hidden");
  if (diffs.length) $("#do-apply").onclick = async () => {
    const res = await api(`/api/params-apply?${tq()}`, {method: "POST"});
    box.innerHTML = `<div>Written: ${res.written.map(esc).join(", ")}. Review and commit with git when ready.</div>`;
    loadParams();
  };
};

init();
