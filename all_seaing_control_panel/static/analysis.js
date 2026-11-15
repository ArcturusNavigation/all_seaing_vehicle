// Rich display of analyzer reports (Runs & analysis tab), for past runs and live data alike.
// A report is {analyzer, entries, output (text), summary (structured, if the analyzer defines
// summary()), plots}. Analyzers with a renderer below get a readable view; others show their text.
// Labels/explanations come from config/descriptions.yaml (same as the Health page).

const REPORT_RENDERERS = {
  fusion: renderFusionReport,
  events: renderEventsReport,
};

function renderReport(container, report, desc) {
  const render = REPORT_RENDERERS[report.analyzer];
  if (render && report.summary) {
    container.innerHTML = render(report.summary, desc);
    container.querySelectorAll("table.sortable").forEach(makeSortable);
  } else {
    container.innerHTML = `<pre>${esc(report.output)}</pre>`;
  }
}

// "unknown" = the box label isn't a buoy color (banner shapes, numbers...): see bbox_project_pcloud
const colorLabel = col => col === "unknown" ? "other labels" : col;

// ------------------------------------------------------------------ fusion
function renderFusionReport(summary, desc) {
  const comp = desc.components?.bbox_project_pcloud || {};
  const outcomes = comp.outcomes || {};
  // reasons are "<reason>" (rejected) or "skipped: <reason>"
  const key = r => r.startsWith("skipped: ") ? `skipped/${r.slice(9)}` : `rejected/${r}`;
  const label = r => outcomes[key(r)]?.label || humanize(r);
  const color = r => outcomes[key(r)]?.color || "#64748b";
  const explain = r => outcomes[key(r)]?.description || "";
  const cams = Object.keys(summary);
  if (!cams.length) return '<div class="muted">No fusion events in this window.</div>';

  return `<div class="diag-grid">${cams.map(cam => {
    const c = summary[cam];
    const instance = comp.instances?.[cam] || cam;
    const rejected = Object.entries(c.rejected).sort((a, b) => b[1] - a[1])
      .concat(Object.entries(c.skipped || {}).map(([r, n]) => [`skipped: ${r}`, n]));
    const nRejected = rejected.reduce((s, [, n]) => s + n, 0);
    const total = c.detections + nRejected || 1;
    const segs = [{label: "Detections", color: "var(--ok)", n: c.detections, desc: "Boxes located in 3D and published"},
                  ...rejected.map(([r, n]) => ({label: label(r), color: color(r), n, desc: explain(r)}))];
    const worst = rejected.filter(([r]) => !r.startsWith("skipped: "))[0];  // main loss: real rejections only
    // color x reason table
    const colors = Object.keys(c.by_color).sort((a, b) =>
      Object.values(c.by_color[b]).reduce((s, n) => s + n, 0) - Object.values(c.by_color[a]).reduce((s, n) => s + n, 0));
    const reasons = rejected.map(([r]) => r);
    const maxCell = Math.max(1, ...colors.flatMap(col => reasons.map(r => c.by_color[col][r] || 0)));
    return `<div class="card">
      <div class="card-title">${esc(instance)}</div>
      <div class="stats">
        <div><b>${c.frames.toLocaleString()}</b><span>Fused frames</span></div>
        <div><b>${c.boxes.toLocaleString()}</b><span>YOLO boxes</span></div>
        <div><b>${c.detections.toLocaleString()}</b><span>Detections</span></div>
        <div><b>${c.boxes ? (c.detections / c.boxes * 100).toFixed(0) + "%" : "–"}</b><span>Yield</span></div>
      </div>
      ${worst ? `<div class="muted small-title">Main loss: <b style="color:${color(worst[0])}">${esc(label(worst[0]))}</b>
        (${(worst[1] / total * 100).toFixed(0)}% of boxes)${explain(worst[0]) ? ` — ${esc(explain(worst[0]))}` : ""}</div>` : ""}
      <div class="funnel">${segs.filter(s => s.n).map(s =>
        `<div style="width:${s.n / total * 100}%;background:${s.color}" title="${esc(s.label)}: ${s.n} (${(s.n / total * 100).toFixed(0)}%)\n${esc(s.desc)}"></div>`).join("")}</div>
      <table class="outcomes">${segs.filter(s => s.n).map(s => `<tr title="${esc(s.desc)}">
        <td><span class="swatch" style="background:${s.color}"></span>${esc(s.label)}</td>
        <td class="num">${(s.n / total * 100).toFixed(0)}%</td><td class="num">${s.n.toLocaleString()}</td></tr>`).join("")}</table>
      ${colors.length && reasons.length ? `<div class="muted small-title" style="margin-top:10px">Rejections by buoy color</div>
      <table class="outcomes heat"><tr><th></th>${colors.map(col => `<th class="num">${esc(colorLabel(col))}</th>`).join("")}</tr>
        ${reasons.map(r => `<tr title="${esc(explain(r))}"><td><span class="swatch" style="background:${color(r)}"></span>${esc(label(r))}</td>
          ${colors.map(col => {
            const n = c.by_color[col][r] || 0;
            return `<td class="num" style="background:${n ? `color-mix(in srgb, ${color(r)} ${Math.round(15 + 60 * n / maxCell)}%, transparent)` : "transparent"}">${n || ""}</td>`;
          }).join("")}</tr>`).join("")}</table>` : ""}
      ${Object.keys(c.dropped).length ? `<div class="facts"><span class="bad">Frames dropped: ${Object.entries(c.dropped).map(([r, n]) => `${esc(r)} ${n}`).join(", ")}</span></div>` : ""}
    </div>`;
  }).join("")}</div>`;
}

// Plot series of an analyzer shown with the same labels/colors as its summary
const PLOT_STYLERS = {
  fusion: (plot, desc) => {
    const comp = desc.components?.bbox_project_pcloud || {};
    plot.title = plot.title.replace(/Fusion funnel: (.*)/, (m, cam) => `Boxes over time: ${comp.instances?.[cam] || cam}`);
    for (const s of plot.series || []) {
      if (s.name === "detections") { s.label = "Detections"; s.color = "#1f8a4c"; continue; }
      if (s.name === "boxes in") { s.label = "YOLO boxes in"; s.color = "#94a3b8"; continue; }
      const o = comp.outcomes?.[s.name.replace(/^rejected: /, "rejected/")];
      const o2 = o || comp.outcomes?.[s.name.replace(/^skipped: /, "skipped/")];
      if (o2) { s.label = o2.label; s.color = o2.color; }
    }
    return plot;
  },
};
function stylePlot(analyzer, plot, desc) {
  return PLOT_STYLERS[analyzer] ? PLOT_STYLERS[analyzer](plot, desc) : plot;
}

// ------------------------------------------------------------------ events
function renderEventsReport(rows) {
  if (!rows.length) return '<div class="muted">No events.</div>';
  return `<table class="outcomes sortable"><thead><tr><th>Node</th><th>Event</th><th class="num">Count</th>
      <th class="num">Rate [/s]</th><th class="num">First [s]</th><th class="num">Last [s]</th></tr></thead>
    <tbody>${rows.map(r => `<tr><td>${esc(r.node)}</td><td>${esc(r.event)}</td><td class="num">${r.count.toLocaleString()}</td>
      <td class="num">${r.rate}</td><td class="num">${r.first}</td><td class="num">${r.last}</td></tr>`).join("")}</tbody></table>`;
}

function makeSortable(table) {
  table.querySelectorAll("th").forEach((th, i) => th.onclick = () => {
    const body = table.tBodies[0], rows = [...body.rows];
    const dir = th.dataset.dir === "asc" ? -1 : 1;
    th.dataset.dir = dir === 1 ? "asc" : "desc";
    rows.sort((a, b) => {
      const x = a.cells[i].textContent.replace(/,/g, ""), y = b.cells[i].textContent.replace(/,/g, "");
      return (isNaN(x) || isNaN(y) ? x.localeCompare(y) : x - y) * dir;
    });
    rows.forEach(r => body.appendChild(r));
  });
}
