// Minimal SVG plotting for plot specs from the backend (see runs.py):
//   {title, kind: "line" | "stacked" | "xy" | "timeline", x_label, y_label, series: [{name, x, y}], segments: [...]}
// "stacked": series are stacked areas (a partition of a total), in order from the bottom; a series
// with `total: true` is drawn as an outline on top instead of a band.
// No external libraries, so the panel works offline (e.g. at competition).

const PLOT_COLORS = ["#2557d6", "#d64545", "#1f8a4c", "#b7791f", "#7b3fd6", "#0f8b8d", "#c2410c", "#64748b",
                     "#db2777", "#4d7c0f"];

function niceTicks(min, max, n = 5) {
  if (!isFinite(min) || !isFinite(max)) return [];
  if (min === max) { min -= 1; max += 1; }
  const step0 = (max - min) / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  const step = [1, 2, 5, 10].map(m => m * mag).find(s => s >= step0) || mag * 10;
  const ticks = [];
  for (let v = Math.ceil(min / step) * step; v <= max + 1e-9; v += step) ticks.push(+v.toPrecision(10));
  return ticks;
}

function svgEl(tag, attrs, parent) {
  const el = document.createElementNS("http://www.w3.org/2000/svg", tag);
  for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
  if (parent) parent.appendChild(el);
  return el;
}

function renderPlot(container, spec) {
  const wrap = document.createElement("div");
  wrap.className = "plot card";
  wrap.innerHTML = `<div class="card-title">${spec.title || ""}</div>`;
  container.appendChild(wrap);
  const W = 900, H = 280, M = {l: 56, r: 12, t: 10, b: 34};
  const svg = svgEl("svg", {viewBox: `0 0 ${W} ${H}`, preserveAspectRatio: "none"}, wrap);
  const css = getComputedStyle(document.documentElement);
  const axisColor = css.getPropertyValue("--muted"), gridColor = css.getPropertyValue("--border");

  let xs = [], ys = [];
  let stackTop = null;
  if (spec.kind === "timeline") {
    for (const s of spec.segments || []) xs.push(s.start, s.end);
  } else if (spec.kind === "stacked") {
    const bands = (spec.series || []).filter(s => !s.total);
    stackTop = (bands[0]?.x || []).map((_, j) => bands.reduce((sum, s) => sum + (s.y[j] || 0), 0));
    for (const s of spec.series || []) { xs.push(...s.x); ys.push(...s.y); }
    ys.push(...stackTop);
  } else {
    for (const s of spec.series || []) { xs.push(...s.x); ys.push(...s.y); }
  }
  if (!xs.length) { wrap.insertAdjacentHTML("beforeend", `<div class="muted">no data</div>`); return; }
  let [x0, x1] = [Math.min(...xs), Math.max(...xs)];
  let [y0, y1] = ys.length ? [Math.min(...ys), Math.max(...ys)] : [0, 1];
  if (spec.kind === "line" || spec.kind === "stacked") y0 = Math.min(0, y0);
  if (x0 === x1) { x0 -= 1; x1 += 1; } if (y0 === y1) { y0 -= 1; y1 += 1; }
  if (spec.kind === "xy") {  // equal aspect-ish: pad the smaller range
    const pad = (a, b, r) => { const c = (a + b) / 2; return [c - r / 2, c + r / 2]; };
    const rx = x1 - x0, ry = y1 - y0, k = (W - M.l - M.r) / (H - M.t - M.b);
    if (rx / ry > k) [y0, y1] = pad(y0, y1, rx / k); else [x0, x1] = pad(x0, x1, ry * k);
  }
  const sx = x => M.l + (x - x0) / (x1 - x0) * (W - M.l - M.r);
  const sy = y => H - M.b - (y - y0) / (y1 - y0) * (H - M.t - M.b);

  for (const t of niceTicks(x0, x1, 8)) {
    svgEl("line", {x1: sx(t), x2: sx(t), y1: M.t, y2: H - M.b, stroke: gridColor}, svg);
    svgEl("text", {x: sx(t), y: H - M.b + 14, "font-size": 11, fill: axisColor, "text-anchor": "middle"}, svg).textContent = t;
  }
  if (spec.kind !== "timeline") {
    for (const t of niceTicks(y0, y1, 5)) {
      svgEl("line", {x1: M.l, x2: W - M.r, y1: sy(t), y2: sy(t), stroke: gridColor}, svg);
      svgEl("text", {x: M.l - 6, y: sy(t) + 4, "font-size": 11, fill: axisColor, "text-anchor": "end"}, svg).textContent = t;
    }
  }
  svgEl("text", {x: (W + M.l) / 2, y: H - 4, "font-size": 11, fill: axisColor, "text-anchor": "middle"}, svg).textContent = spec.x_label || "";
  if (spec.y_label) {
    svgEl("text", {x: 12, y: H / 2, "font-size": 11, fill: axisColor, "text-anchor": "middle",
                   transform: `rotate(-90 12 ${H / 2})`}, svg).textContent = spec.y_label;
  }

  const legend = document.createElement("div");
  legend.className = "legend";
  if (spec.kind === "timeline") {
    const labels = [...new Set((spec.segments || []).map(s => s.label))];
    const color = l => PLOT_COLORS[labels.indexOf(l) % PLOT_COLORS.length];
    for (const s of spec.segments) {
      const r = svgEl("rect", {x: sx(s.start), y: M.t + 20, width: Math.max(1, sx(s.end) - sx(s.start)), height: 60,
                               fill: color(s.label), opacity: 0.85}, svg);
      svgEl("title", {}, r).textContent = `${s.label}: ${s.start.toFixed(1)}–${s.end.toFixed(1)} s`;
    }
    labels.forEach(l => legend.insertAdjacentHTML("beforeend", `<span style="--c:${color(l)}">${l}</span>`));
  } else if (spec.kind === "stacked") {
    const bands = (spec.series || []).filter(s => !s.total);
    const totalSum = stackTop.reduce((a, b) => a + b, 0) || 1;
    let base = (bands[0]?.x || []).map(() => 0);
    bands.forEach((s, i) => {
      const c = s.color || PLOT_COLORS[i % PLOT_COLORS.length];
      const top = base.map((b, j) => b + (s.y[j] || 0));
      const pts = s.x.map((x, j) => `${sx(x).toFixed(1)},${sy(top[j]).toFixed(1)}`)
        .concat(s.x.map((x, j) => `${sx(x).toFixed(1)},${sy(base[j]).toFixed(1)}`).reverse()).join(" ");
      const share = (s.y.reduce((a, b) => a + b, 0) / totalSum * 100).toFixed(0);
      const poly = svgEl("polygon", {points: pts, fill: c, "fill-opacity": 0.75, stroke: c, "stroke-width": 0.5}, svg);
      svgEl("title", {}, poly).textContent = `${s.label || s.name}: ${share}% of the stacked total`;
      legend.insertAdjacentHTML("beforeend", `<span style="--c:${c}">${s.label || s.name} (${share}%)</span>`);
      base = top;
    });
    for (const s of (spec.series || []).filter(s => s.total)) {
      const c = s.color || axisColor;
      svgEl("polyline", {points: s.x.map((x, j) => `${sx(x).toFixed(1)},${sy(s.y[j]).toFixed(1)}`).join(" "), fill: "none",
                         stroke: c, "stroke-width": 1.8, "stroke-dasharray": "5 3", "vector-effect": "non-scaling-stroke"}, svg);
      legend.insertAdjacentHTML("beforeend", `<span style="--c:${c}">${s.label || s.name} (outline)</span>`);
    }
  } else {
    (spec.series || []).forEach((s, i) => {
      const c = s.color || PLOT_COLORS[i % PLOT_COLORS.length];
      if (!s.x.length) return;
      const pts = s.x.map((x, j) => `${sx(x).toFixed(1)},${sy(s.y[j]).toFixed(1)}`).join(" ");
      svgEl("polyline", {points: pts, fill: "none", stroke: c, "stroke-width": 1.6, "vector-effect": "non-scaling-stroke"}, svg);
      if (spec.kind === "xy" && s.x.length) {
        svgEl("circle", {cx: sx(s.x[0]), cy: sy(s.y[0]), r: 4, fill: c}, svg);
      }
      legend.insertAdjacentHTML("beforeend", `<span style="--c:${c}">${s.label || s.name}</span>`);
    });
  }
  wrap.appendChild(legend);
}
