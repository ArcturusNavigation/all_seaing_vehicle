// Run tab, "Rosbag recording": record presets (config/rosbag_presets.yaml, imported from
// all_seaing_bringup/rosbag_record/*.bash) on the selected target, stop recordings, see each bag's
// size and the free space; create / edit / copy / delete presets. Backend: rosbags.py, /api/rosbags/*.
// A launch's form also offers "record rosbag" (app.js): that recording stops with the launch.

const fmtBytes = b => b == null ? "–" : b >= 1e9 ? `${(b / 1e9).toFixed(2)} GB` : b >= 1e6 ? `${(b / 1e6).toFixed(1)} MB` : `${Math.round(b / 1e3)} kB`;
const rb = {lastSizes: {}};  // bag_dir -> [time, bytes]: write rate

async function loadRosbagPresets() {
  try { state.rosbagPresets = await api("/api/rosbags/presets"); } catch { state.rosbagPresets = []; }
  $("#rosbag-presets").innerHTML = `<table class="outcomes">${state.rosbagPresets.map(p => `
    <tr><td><b>${esc(p.title)}</b><div class="muted">${p.all_topics ? "all topics" : `${p.topics.length} topics`} · ${esc(p.prefix)}_…${p.source ? ` · from ${esc(p.source.split("/").pop())}` : ""}</div></td>
      <td style="white-space:nowrap"><button class="small" data-rec="${esc(p.id)}">● record</button>
        <button class="small secondary" data-edit="${esc(p.id)}">edit</button>
        <button class="small secondary" data-copy="${esc(p.id)}">copy</button>
        <button class="small secondary" data-del="${esc(p.id)}">×</button></td></tr>`).join("")}</table>`;
  $("#rosbag-presets").querySelectorAll("button").forEach(b => b.onclick = () => {
    const d = b.dataset, p = state.rosbagPresets.find(x => x.id === (d.rec || d.edit || d.copy || d.del));
    if (d.rec) startRosbag(p.id);
    else if (d.edit) editPreset(p, false);
    else if (d.copy) editPreset({...p, title: p.title + " (copy)", prefix: p.prefix + "_copy"}, true);
    else if (d.del) deletePreset(p);
  });
}

async function startRosbag(id) {
  try { await api("/api/rosbags/record", {method: "POST", body: {preset: id, target: state.target}}); }
  catch (e) { return alert(e.message); }
  refreshRosbags(); refreshProcs();
}

// Editor for a new / copied preset (isNew) or an existing one
function editPreset(p, isNew) {
  const ed = $("#rosbag-editor");
  ed.classList.remove("hidden");
  ed.innerHTML = `<div class="card" style="margin-top:8px">
    <div class="card-title">${isNew ? "New preset" : `Edit preset: ${esc(p.title)}`}</div>
    <div class="form-grid">
      <label>title</label><input id="rp-title" value="${esc(p.title || "")}">
      <label title="bag folder: <prefix>_MM_DD__HH_MM_SS">folder prefix</label><input id="rp-prefix" value="${esc(p.prefix || "")}">
      <label>all topics</label><label><input type="checkbox" id="rp-all" ${p.all_topics ? "checked" : ""}> record everything (--all)</label>
      <label>topics</label><textarea id="rp-topics" rows="10" placeholder="one topic per line">${esc((p.topics || []).join("\n"))}</textarea>
    </div>
    <div class="toolbar" style="margin-top:8px"><button id="rp-save">Save</button><button id="rp-cancel" class="secondary">Cancel</button>
      ${p.source && !isNew ? `<span class="muted">imported from ${esc(p.source)} (the script itself isn't changed)</span>` : ""}</div></div>`;
  const allBox = $("#rp-all"), topics = $("#rp-topics");
  const sync = () => topics.disabled = allBox.checked;
  allBox.onchange = sync; sync();
  $("#rp-cancel").onclick = () => ed.classList.add("hidden");
  $("#rp-save").onclick = async () => {
    const body = {title: $("#rp-title").value, prefix: $("#rp-prefix").value, all_topics: allBox.checked,
                  topics: topics.value.split(/\s+/).filter(Boolean)};
    try {
      await api(isNew ? "/api/rosbags/presets" : `/api/rosbags/presets/${encodeURIComponent(p.id)}`,
                {method: isNew ? "POST" : "PUT", body});
    } catch (e) { return alert(e.message); }
    ed.classList.add("hidden");
    loadRosbagPresets();
  };
  ed.scrollIntoView({block: "nearest"});
}
$("#rosbag-new").onclick = () => editPreset({title: "", prefix: "", all_topics: false, topics: []}, true);

async function deletePreset(p) {
  if (!confirm(`Delete the preset "${p.title}"? (recorded bags are kept${p.source ? "; the script isn't changed" : ""})`)) return;
  try { await api(`/api/rosbags/presets/${encodeURIComponent(p.id)}`, {method: "DELETE"}); } catch (e) { return alert(e.message); }
  loadRosbagPresets();
}

// Recordings of the target: running ones (size + write rate, stop) and recent ones (size)
async function refreshRosbags() {
  if (!$("#tab-run").classList.contains("active")) return;
  let st;
  try { st = await api(`/api/rosbags/status?${tq()}`); } catch { return; }
  $("#rosbag-free").textContent = st.free_bytes != null ? `${fmtBytes(st.free_bytes)} free in ${st.bags_path}` : "";
  const now = Date.now() / 1000, title = id => state.rosbagPresets?.find(p => p.id === id)?.title || id;
  $("#rosbag-recordings").innerHTML = st.recordings.length ? `<table class="outcomes">
    <tr><th>Bag</th><th>Status</th><th class="num">Size</th><th></th></tr>${st.recordings.map(r => {
      const prev = rb.lastSizes[r.bag_dir], running = r.status === "running";
      const rate = running && prev && r.bytes != null && now > prev[0] ? (r.bytes - prev[1]) / (now - prev[0]) : null;
      if (r.bytes != null) rb.lastSizes[r.bag_dir] = [now, r.bytes];
      const dur = ((r.ended || now) - r.started);
      return `<tr><td><b>${esc(r.bag_dir.split("/").pop())}</b><div class="muted">${esc(title(r.preset))}${r.linked_to ? " · with a launch" : ""} · ${Math.round(dur / 60)} min</div></td>
        <td><span class="badge ${r.status}">${r.status}</span></td>
        <td class="num">${fmtBytes(r.bytes)}${rate != null ? `<div class="muted">${fmtBytes(Math.max(0, rate))}/s</div>` : ""}</td>
        <td>${running ? `<button class="small danger" data-stop="${r.id}">stop</button>` : ""}</td></tr>`;
    }).join("")}</table>` : '<div class="muted">No recordings yet: record a preset below, or pick "record rosbag" when starting a launch.</div>';
  $("#rosbag-recordings").querySelectorAll("[data-stop]").forEach(b => b.onclick = async () => {
    await api(`/api/processes/${b.dataset.stop}/stop`, {method: "POST"}); refreshRosbags(); refreshProcs();
  });
}

loadRosbagPresets();
setInterval(refreshRosbags, 3000);
document.querySelector('#tabs button[data-tab="run"]').addEventListener("click", refreshRosbags);
$("#target-select").addEventListener("change", refreshRosbags);
refreshRosbags();
