// Health page rendering: presents /diagnostics statuses by what they mean, using
// config/descriptions.yaml (served at /api/descriptions). Unknown statuses/fields get a
// generic card with generated labels. Every card keeps the raw key/values in "Raw values".

const LEVEL_TEXT = {OK: "OK", WARN: "Warning", ERROR: "Error", STALE: "No data"};
const openDetails = new Set();  // keep "Raw values" sections open across 1 Hz re-renders

function num(v) { const n = parseFloat(v); return isNaN(n) ? null : n; }
function fmt(v, digits = 1) {
  const n = num(v);
  if (n === null) return esc(v ?? "–");
  if (Math.abs(n) >= 1e6) return n.toExponential(1);
  return Number.isInteger(n) ? n.toLocaleString() : n.toFixed(digits);
}
function humanize(key) {
  return key.replace(/[/_]+/g, " ").replace(/\brate\b/, "per second").replace(/^\w/, c => c.toUpperCase());
}
// average rate since the node started (counter total / uptime): sparse flows don't read as 0
function avgRate(v, key) {
  const up = num(v.uptime_s), total = num(v[key]);
  return up && total !== null ? total / up : null;
}
function levelBadge(level) { return `<span class="badge ${level}">${LEVEL_TEXT[level] || level}</span>`; }

function rawValues(d) {
  const id = "raw-" + d.name;
  const open = openDetails.has(id) ? "open" : "";
  return `<details class="raw" data-id="${esc(id)}" ${open}><summary>Raw values</summary>
    <div class="kv">${Object.entries(d.values).map(([k, v]) => `<span>${esc(k)}</span><span>${esc(v)}</span>`).join("")}</div>
  </details>`;
}

// Bar showing a value against an expected minimum (e.g. topic rate vs expected rate)
function rateBar(rate, expected) {
  if (rate === null) return "";
  const scale = Math.max(expected * 1.5, rate, 1);
  const cls = expected && rate < expected ? "warn" : "ok";
  return `<div class="ratebar"><div class="fill ${cls}" style="width:${Math.min(100, rate / scale * 100)}%"></div>
    ${expected ? `<div class="mark" style="left:${expected / scale * 100}%" title="expected ≥ ${expected} Hz"></div>` : ""}</div>`;
}

// Mode of the node producing a topic (e.g. the local map's merge mode), from its status
function modeNote(info, allDiags) {
  if (!info.mode_from) return "";
  const d = allDiags.find(x => x.name === info.mode_from.status);
  const mode = d?.values?.[info.mode_from.key];
  return mode ? ` <span class="badge unknown" title="mode of ${esc(info.mode_from.status)}">${esc(mode)}</span>` : "";
}

// ------------------------------------------------------------------ merge components (local map)
function renderMerge(d, comp) {
  const v = d.values;
  const rate = k => num(v[k + "/rate"]) ?? 0, total = k => num(v[k]) ?? 0;
  const mode = v.mode || "?", individual = mode.startsWith("individual");
  const cams = (v.cameras || "").split(",").map(c => c.trim()).filter(Boolean);
  const inTotal = cams.reduce((s, c) => s + total(`in/${c}`), 0);
  const outTotal = total("out");
  // merged mode: each published set consumed one message per camera; the rest found no match
  const unmatched = individual ? 0 : Math.max(0, inTotal - outTotal * cams.length);
  return `<div class="card diag ${d.level}">
    <div class="card-title">Local map ${levelBadge(d.level)} <span class="badge unknown">${esc(mode)}</span></div>
    <div class="muted" style="margin-bottom:6px">${individual
      ? "Each camera's detections are published as soon as they arrive: one camera per local-map message."
      : "One local-map message per synchronized set of all cameras; camera messages without a matching set are dropped."}</div>
    <table class="outcomes"><tr><th>Camera</th><th class="num">Messages/s</th><th class="num" title="last second">Obstacles/s</th>
        <th class="num" title="average since the node started (obstacles are sparse: often 0 in a given second)">Obstacles avg/s</th>
        <th class="num">Obstacles (total)</th></tr>
      ${cams.map(c => `<tr><td>${esc(comp.cameras?.[c] || c)}</td><td class="num">${fmt(rate(`in/${c}`))}</td>
        <td class="num">${fmt(rate(`obstacles_in/${c}`))}</td><td class="num">${fmt(avgRate(v, `obstacles_in/${c}`), 2)}</td>
        <td class="num">${total(`obstacles_in/${c}`).toLocaleString()}</td></tr>`).join("")}
      <tr><td><b>Local map out</b></td><td class="num"><b>${fmt(rate("out"))}</b></td>
        <td class="num"><b>${fmt(rate("obstacles_out"))}</b></td><td class="num"><b>${fmt(avgRate(v, "obstacles_out"), 2)}</b></td>
        <td class="num">${total("obstacles_out").toLocaleString()}</td></tr></table>
    <div class="facts">
      <span title="obstacles seen by two cameras, removed by angle/distance thresholds">Duplicates removed: <b>${total("duplicates_removed").toLocaleString()}</b> (${fmt(rate("duplicates_removed"))}/s)</span>
      ${individual ? "" : `<span class="${unmatched ? "bad" : ""}" title="camera messages that never formed a synchronized set">Unmatched camera messages: <b>${unmatched.toLocaleString()}</b></span>`}
    </div>
    ${rawValues(d)}</div>`;
}

// ------------------------------------------------------------------ topic monitor: one table
function renderTopics(statuses, desc, allDiags) {
  const groups = {};
  for (const d of statuses) {
    const topic = "/" + d.name.slice("topic_monitor/".length);
    const info = desc.topics?.[topic] || {};
    (groups[info.group || "Other topics"] = groups[info.group || "Other topics"] || []).push({d, topic, info});
  }
  const order = ["Sensors", "Perception", "Localization & control"];
  const names = Object.keys(groups).sort((a, b) => (order.indexOf(a) + 1 || 99) - (order.indexOf(b) + 1 || 99));
  return `<div class="card"><div class="card-title">Topics
      <span class="muted">measured on the boat computer by the topic monitor</span></div>
    <table class="topics"><thead><tr><th>Topic</th><th>Status</th><th>Rate</th><th></th><th title="now minus the message timestamp: pipeline delay, or clock/time-sync problems if large">Stamp age</th></tr></thead>
    <tbody>${names.map(g => `<tr class="group-row"><td colspan="5">${esc(g)}</td></tr>` + groups[g].map(({d, topic, info}) => {
      const rate = num(d.values["messages/rate"]), expected = num(d.values.expected_min_rate) || 0;
      const age = num(d.values.stamp_age_ms);
      const ageCls = age !== null && Math.abs(age) > 1000 ? "bad" : "";
      return `<tr title="${esc(info.description || "")}">
        <td><b>${esc(info.title || topic)}</b>${modeNote(info, allDiags)}<div class="muted mono">${esc(topic)}</div></td>
        <td>${levelBadge(d.level)}${d.level !== "OK" ? `<div class="muted">${esc(d.message)}</div>` : ""}</td>
        <td class="num">${rate === null ? "–" : rate.toFixed(1) + " Hz"}${expected ? `<div class="muted">expected ≥ ${expected} Hz</div>` : ""}</td>
        <td class="barcell">${rateBar(rate, expected)}</td>
        <td class="num ${ageCls}">${age === null ? "–" : fmt(age, 0) + " ms"}</td></tr>`;
    }).join("")).join("")}</tbody></table></div>`;
}

// ------------------------------------------------------------------ funnel components (e.g. fusion)
function renderFunnel(d, comp, instance) {
  const v = d.values;
  const rate = k => num(v[k + "/rate"]) ?? 0, total = k => num(v[k]) ?? 0;
  const inRate = rate(comp.input.key), outRate = rate(comp.output.key);
  const inTotal = total(comp.input.key), outTotal = total(comp.output.key);
  const outcomes = Object.entries(comp.outcomes || {}).map(([k, o]) => ({k, ...o, rate: rate(k), total: total(k)}));
  // also show reasons not listed in the descriptions
  for (const k of Object.keys(v)) {
    if (/^(rejected|skipped)\//.test(k) && !k.endsWith("/rate") && !(comp.outcomes || {})[k]) {
      outcomes.push({k, label: humanize(k), color: "#64748b", rate: rate(k), total: total(k)});
    }
  }
  const segs = [{label: comp.output.label, color: "var(--ok)", rate: outRate, total: outTotal,
                 description: "Boxes located in 3D and published as detections"}, ...outcomes];
  const segTotal = segs.reduce((s, x) => s + x.total, 0) || 1;
  const segRate = segs.reduce((s, x) => s + x.rate, 0);
  const yieldPct = inTotal ? (outTotal / inTotal * 100).toFixed(0) + "%" : "–";
  const drops = Object.entries(comp.frame_drops || {}).filter(([k]) => total(k) > 0);
  const fields = comp.fields || {};
  const timing = fields.proc_ms && v["proc_ms/mean"] !== undefined
    ? `${fmt(v["proc_ms/mean"])} ms avg · ${fmt(v["proc_ms/max"])} ms max` : "–";
  const latency = v.input_age_ms !== undefined ? `${fmt(v.input_age_ms, 0)} ms` : "–";

  return `<div class="card diag ${d.level}">
    <div class="card-title">${esc(instance)} ${levelBadge(d.level)}
      ${d.level !== "OK" ? `<span class="muted">${esc(d.message)}</span>` : ""}</div>
    <div class="stats">
      <div title="${esc(comp.heartbeat?.description || "")}"><b>${fmt(rate(comp.heartbeat?.key || "updates"))}/s</b><span>${esc(comp.heartbeat?.label || "Updates")}</span></div>
      <div><b>${fmt(inRate)}/s</b><span>${esc(comp.input.label)}</span></div>
      <div title="last second; average since start in brackets"><b>${fmt(outRate)}/s</b><span>${esc(comp.output.label)} (avg ${fmt(avgRate(v, comp.output.key), 2)}/s)</span></div>
      <div title="detections / boxes since start"><b>${yieldPct}</b><span>Yield (since start)</span></div>
    </div>
    <div class="muted small-title">Where the boxes went (since start, ${inTotal.toLocaleString()} boxes; last second: ${fmt(segRate)}/s)</div>
    <div class="funnel">${segs.filter(s => s.total > 0).map(s =>
      `<div style="width:${s.total / segTotal * 100}%;background:${s.color}" title="${esc(s.label)}: ${s.total.toLocaleString()} (${(s.total / segTotal * 100).toFixed(0)}%)${s.description ? "\n" + esc(s.description) : ""}"></div>`).join("")}</div>
    <table class="outcomes">${segs.filter(s => s.total > 0 || s.rate > 0).sort((a, b) => b.total - a.total).map(s => `
      <tr title="${esc(s.description || "")}"><td><span class="swatch" style="background:${s.color}"></span>${esc(s.label)}</td>
        <td class="num">${(s.total / segTotal * 100).toFixed(0)}%</td><td class="num">${s.total.toLocaleString()}</td>
        <td class="num muted">${fmt(s.rate)}/s</td></tr>`).join("")}</table>
    <div class="facts">
      <span title="${esc(fields.proc_ms?.description || "")}">⏱ ${esc(fields.proc_ms?.label || "Processing")}: <b>${timing}</b></span>
      <span title="${esc(fields.input_age_ms?.description || "")}">↹ ${esc(fields.input_age_ms?.label || "Input latency")}: <b>${latency}</b></span>
      ${drops.map(([k, o]) => `<span class="bad" title="${esc(o.description || "")}">${esc(o.label)}: <b>${total(k)}</b></span>`).join("")}
    </div>
    ${rawValues(d)}
  </div>`;
}

// ------------------------------------------------------------------ generic component card
function renderGeneric(d, desc) {
  const v = d.values, fields = desc.fields || {};
  const rows = [];
  const seen = new Set();
  for (const [k, val] of Object.entries(v)) {
    if (seen.has(k)) continue;
    const f = fields[k] || {};
    if (f.hidden) continue;
    if (v[k + "/rate"] !== undefined) {  // counter: total + rate
      seen.add(k + "/rate");
      const avg = avgRate(v, k);
      rows.push([f.label || humanize(k), `${fmt(val)} <span class="muted">· ${fmt(v[k + "/rate"])}/s${avg !== null ? ` · avg ${fmt(avg, 2)}/s` : ""}</span>`, f.description]);
    } else if (k.endsWith("/mean") && v[k.replace(/mean$/, "max")] !== undefined) {  // timing
      const base = k.slice(0, -5);
      seen.add(base + "/max"); seen.add(base + "/count");
      rows.push([(fields[base] || {}).label || humanize(base),
                 `${fmt(val)} avg · ${fmt(v[base + "/max"])} max${(fields[base] || {}).unit ? " " + fields[base].unit : ""}`]);
    } else if (!k.endsWith("/rate")) {
      rows.push([f.label || humanize(k), `${fmt(val)}${f.unit ? " " + f.unit : ""}`, f.description]);
    }
  }
  return `<div class="card diag ${d.level}">
    <div class="card-title">${esc(d.name)} ${levelBadge(d.level)} <span class="muted">${esc(d.message)}</span></div>
    <table class="outcomes">${rows.map(([l, val, title]) => `<tr title="${esc(title || "")}"><td>${esc(l)}</td><td class="num">${val}</td></tr>`).join("")}</table>
    ${rawValues(d)}</div>`;
}

// ------------------------------------------------------------------ page
function renderHealth(container, diags, desc, {filter = "", problems = false} = {}) {
  const shown = diags.filter(d => d.name.toLowerCase().includes(filter) && (!problems || d.level !== "OK"));
  const comps = desc.components || {};
  const sections = {};  // section title -> html parts
  const add = (title, html, note) => { (sections[title] = sections[title] || {note, parts: []}).parts.push(html); };
  const topics = [];
  for (const d of shown) {
    const [prefix, ...rest] = d.name.split("/");
    const comp = comps[prefix];
    if (comp?.kind === "topics") { topics.push(d); continue; }
    if (comp?.kind === "merge") { add(comp.title, renderMerge(d, comp), comp.description); continue; }
    if (comp?.kind === "funnel") {
      const inst = rest.join("/");
      add(comp.title, renderFunnel(d, comp, comp.instances?.[inst] || inst || prefix), comp.description);
      continue;
    }
    add("Other components", renderGeneric(d, desc));
  }
  container.innerHTML =
    (topics.length ? renderTopics(topics, desc, diags) : "") +
    Object.entries(sections).map(([title, s]) => `
      <div class="section-title">${esc(title)}</div>
      ${s.note ? `<div class="muted section-note">${esc(s.note)}</div>` : ""}
      <div class="diag-grid">${s.parts.join("")}</div>`).join("");
  container.querySelectorAll("details.raw").forEach(el => el.ontoggle = () => {
    el.open ? openDetails.add(el.dataset.id) : openDetails.delete(el.dataset.id);
  });
}
