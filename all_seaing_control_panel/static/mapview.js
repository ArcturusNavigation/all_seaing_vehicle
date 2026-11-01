// Map player for the SLAM and Tasks panels (Runs & analysis, Recordings): draws an analyzer's view()
// (all_seaing_utility run_log_analyzers/slam.py, tasks.py) on a top-down map of the global frame
// (x right, y up), with a time slider, play, and per-frame details.
//   const mp = new MapPlayer(container); mp.setData(view, labels); mp.follow = true (live: stay on the latest frame)
// The map fits the whole map (all landmarks / obstacles, goals, the robot and its trail): ±20 m with a 5 m grid
// at least, widened (coarser grid) to fit it all, centered on its extent.

const LABEL_COLORS = {red: "#e11d48", green: "#16a34a", blue: "#2563eb", yellow: "#eab308", black: "#334155"};
const SLAM_RESULT_COLORS = {0: "#16a34a", 1: "#2563eb", 2: "#94a3b8", 3: "#f97316"};
const SLAM_RESULT_NAMES = {0: "matched", 1: "new landmark", 2: "unlabeled, not added", 3: "distance-gated → new"};
const STATE_COLORS = ["#2557d6", "#0f8b8d", "#b7791f", "#7b3fd6", "#c2410c", "#4d7c0f", "#db2777", "#64748b"];

class MapPlayer {
  constructor(root) {
    this.root = root;
    this.view = null; this.labels = {}; this.i = 0; this.follow = false; this.playing = null;
    root.innerHTML = `
      <div class="toolbar map-toolbar">
        <button class="small mp-prev">◀</button><button class="small mp-play">▶ play</button><button class="small mp-next">▶</button>
        <input type="range" class="mp-slider" min="0" max="0" value="0" style="flex:1;min-width:160px">
        <span class="muted mp-time"></span>
        <label class="mp-cov-label"><input type="checkbox" class="mp-cov" checked> covariance</label>
      </div>
      <div class="mp-states"></div>
      <div class="mp-body">
        <canvas class="mp-canvas" width="900" height="620"></canvas>
        <div class="mp-side"><div class="mp-legend muted small-title"></div><div class="mp-info"></div></div>
      </div>
      <div class="mp-events"></div>`;
    this.canvas = root.querySelector(".mp-canvas");
    this.slider = root.querySelector(".mp-slider");
    this.slider.oninput = () => { this.follow = false; this.show(+this.slider.value); };
    root.querySelector(".mp-prev").onclick = () => this.show(Math.max(0, this.i - 1));
    root.querySelector(".mp-next").onclick = () => this.show(Math.min(this.frames().length - 1, this.i + 1));
    root.querySelector(".mp-play").onclick = () => this.togglePlay();
    root.querySelector(".mp-cov").onchange = () => this.draw();
  }
  frames() { return this.view?.frames || []; }

  setData(view, labels) {
    const atEnd = !this.view || this.i >= this.frames().length - 1;
    const t = this.frames()[this.i]?.t;
    this.view = view; this.labels = labels || this.labels;
    const n = this.frames().length;
    this.slider.max = Math.max(0, n - 1);
    if (!n) { this.root.querySelector(".mp-time").textContent = "no data for this panel in the selected source"; this.draw(); return; }
    // keep the same moment when data refreshes, unless following the latest (live) or at the end
    let i = n - 1;
    if (!(this.follow || atEnd) && t !== undefined) i = Math.max(0, this.frames().findIndex(f => f.t >= t));
    this.drawStates();
    this.show(i);
  }

  togglePlay() {
    const btn = this.root.querySelector(".mp-play");
    if (this.playing) { clearInterval(this.playing); this.playing = null; btn.textContent = "▶ play"; return; }
    btn.textContent = "❚❚ pause"; this.follow = false;
    this.playing = setInterval(() => this.i < this.frames().length - 1 ? this.show(this.i + 1) : this.togglePlay(), 100);
  }

  show(i) {
    this.i = i; this.slider.value = i;
    const f = this.frames()[i];
    if (f) this.root.querySelector(".mp-time").textContent =
      `${i + 1}/${this.frames().length} · t = ${f.t.toFixed(1)} s${this.view.frame_id ? ` · frame ${this.view.frame_id}` : ""}`;
    this.draw();
  }

  labelName(l) { return this.labels[l] ?? (l === -1 ? "unlabeled" : `label ${l}`); }
  labelColor(l) {
    const name = this.labels[l] || "";
    const c = Object.keys(LABEL_COLORS).find(k => name.startsWith(k));
    return c ? LABEL_COLORS[c] : l === -1 ? "#94a3b8" : "#7c3aed";
  }

  // ---------------------------------------------------------------- geometry
  // view centered on the extent of pts (the whole map + robot), ±20 m minimum, widened to fit all of it
  setup(pts) {
    const c = this.canvas, W = c.width, H = c.height;
    const xs = pts.map(p => p[0]), ys = pts.map(p => p[1]);
    const center = [(Math.min(...xs) + Math.max(...xs)) / 2, (Math.min(...ys) + Math.max(...ys)) / 2];
    const far = Math.max(0, ...pts.map(p => Math.max(Math.abs(p[0] - center[0]), Math.abs(p[1] - center[1]) * W / H)));
    const R = far + 2 > 20 ? Math.ceil((far + 2) / 10) * 10 : 20;  // half-width [m]
    const grid = R <= 20 ? 5 : R <= 50 ? 10 : R <= 120 ? 20 : 50;
    const k = W / (2 * R);
    this.tf = {cx: center[0], cy: center[1], k, W, H, R, grid};
    return [x => W / 2 + (x - center[0]) * k, y => H / 2 - (y - center[1]) * k];
  }

  grid(g) {
    const {cx, cy, k, W, H, grid} = this.tf, css = getComputedStyle(document.documentElement);
    g.strokeStyle = css.getPropertyValue("--border"); g.lineWidth = 1;
    g.fillStyle = css.getPropertyValue("--muted"); g.font = "11px system-ui";
    for (let x = Math.ceil((cx - W / 2 / k) / grid) * grid; x <= cx + W / 2 / k; x += grid) {
      const px = W / 2 + (x - cx) * k; g.beginPath(); g.moveTo(px, 0); g.lineTo(px, H); g.stroke(); g.fillText(`${x}`, px + 2, H - 4);
    }
    for (let y = Math.ceil((cy - H / 2 / k) / grid) * grid; y <= cy + H / 2 / k; y += grid) {
      const py = H / 2 - (y - cy) * k; g.beginPath(); g.moveTo(0, py); g.lineTo(W, py); g.stroke(); g.fillText(`${y}`, 2, py - 2);
    }
  }

  robot(g, X, Y, pose, color = "#2557d6") {
    if (!pose) return;
    const [x, y, h] = pose, px = X(x), py = Y(y), s = 11;
    g.save(); g.translate(px, py); g.rotate(-h);
    g.fillStyle = color; g.beginPath(); g.moveTo(s, 0); g.lineTo(-s * 0.7, s * 0.6); g.lineTo(-s * 0.7, -s * 0.6); g.closePath(); g.fill();
    g.restore();
  }

  ellipse(g, X, Y, x, y, cxx, cxy, cyy, color) {  // 2-sigma covariance ellipse
    const tr = cxx + cyy, det = cxx * cyy - cxy * cxy, d = Math.sqrt(Math.max(0, tr * tr / 4 - det));
    const l1 = tr / 2 + d, l2 = Math.max(0, tr / 2 - d), ang = Math.atan2(l1 - cxx, cxy || 1e-9);
    const a = 2 * Math.sqrt(Math.max(l1, 0)) * this.tf.k, b = 2 * Math.sqrt(l2) * this.tf.k;
    if (!isFinite(a) || a > 4 * this.tf.W) return;
    g.save(); g.strokeStyle = color; g.globalAlpha = 0.3; g.setLineDash([3, 3]);
    g.beginPath(); g.ellipse(X(x), Y(y), Math.max(a, 1), Math.max(b, 1), -ang, 0, 7); g.stroke(); g.restore();
  }

  trail(g, X, Y, pts, color, dash = []) {  // broken where the pose jumps > 5 m (relocalization, looping bag)
    if (pts.length < 2) return;
    g.strokeStyle = color; g.lineWidth = 1.5; g.setLineDash(dash);
    g.beginPath(); g.moveTo(X(pts[0][0]), Y(pts[0][1]));
    for (let j = 1; j < pts.length; j++) {
      const jump = Math.hypot(pts[j][0] - pts[j - 1][0], pts[j][1] - pts[j - 1][1]) > 5;
      jump ? g.moveTo(X(pts[j][0]), Y(pts[j][1])) : g.lineTo(X(pts[j][0]), Y(pts[j][1]));
    }
    g.stroke(); g.setLineDash([]);
  }

  arrow(g, X, Y, from, vec, scale, color) {  // vec in the robot frame (forward, left), rotated by heading
    if (!from || !vec) return;
    const [x, y, h] = from, vx = vec[0] * Math.cos(h) - vec[1] * Math.sin(h), vy = vec[0] * Math.sin(h) + vec[1] * Math.cos(h);
    const x1 = X(x), y1 = Y(y), x2 = X(x + vx * scale), y2 = Y(y + vy * scale);
    if (Math.hypot(x2 - x1, y2 - y1) < 2) return;
    g.strokeStyle = color; g.lineWidth = 2.5; g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2); g.stroke();
    const a = Math.atan2(y2 - y1, x2 - x1);
    g.beginPath(); g.moveTo(x2, y2); g.lineTo(x2 - 8 * Math.cos(a - 0.4), y2 - 8 * Math.sin(a - 0.4));
    g.lineTo(x2 - 8 * Math.cos(a + 0.4), y2 - 8 * Math.sin(a + 0.4)); g.closePath(); g.fillStyle = color; g.fill();
  }

  // ---------------------------------------------------------------- drawing
  draw() {
    const g = this.canvas.getContext("2d"), f = this.frames()[this.i];
    g.clearRect(0, 0, this.canvas.width, this.canvas.height);
    if (!f) { this.root.querySelector(".mp-info").innerHTML = ""; this.root.querySelector(".mp-events").innerHTML = ""; return; }
    if (this.view.domain === "slam") this.drawSlam(g, f); else this.drawTasks(g, f);
  }

  drawSlam(g, f) {
    const pose = f.pose || [0, 0, 0];
    const trail = (this.view.trail || []).filter((_, j) => (this.view.trail_t?.[j] ?? 0) <= f.t);
    // fit the whole map: every landmark, the detections, the robot and its trail
    const banners = f.banners || [];
    const pts = [pose, ...f.landmarks.map(l => [l[2], l[3]]), ...f.detections.map(d => [d[1], d[2]]), ...trail,
                 ...banners.map(b => [b[2], b[3]])];
    const [X, Y] = this.setup(pts);
    this.grid(g);
    this.trail(g, X, Y, trail, "#2557d6");
    const byId = {};
    for (const l of f.landmarks) {
      const [id, label, x, y, cxx, cxy, cyy, dead] = l, col = this.labelColor(label);
      byId[id] = l;
      if (this.root.querySelector(".mp-cov").checked) this.ellipse(g, X, Y, x, y, cxx, cxy, cyy, col);
      g.beginPath(); g.arc(X(x), Y(y), 6, 0, 7);
      if (dead) { g.strokeStyle = col; g.lineWidth = 2; g.stroke(); } else { g.fillStyle = col; g.fill(); }
      g.fillStyle = getComputedStyle(document.documentElement).getPropertyValue("--text"); g.font = "11px system-ui";
      g.fillText(`#${id} ${this.labelName(label)}`, X(x) + 8, Y(y) - 6);
    }
    // banners: a segment of the banner's width through its center, perpendicular to its normal (arrow)
    for (const [id, label, x, y, th, width] of banners) {
      const col = this.labelColor(label), w = Math.max(width || 0, 1) / 2;
      const ex = -Math.sin(th) * w, ey = Math.cos(th) * w;  // along the banner (normal = (cos th, sin th))
      g.strokeStyle = col; g.lineWidth = 5; g.lineCap = "round";
      g.beginPath(); g.moveTo(X(x - ex), Y(y - ey)); g.lineTo(X(x + ex), Y(y + ey)); g.stroke(); g.lineCap = "butt";
      this.arrow(g, X, Y, [x, y, th], [1.5, 0], 1, col);
      g.fillStyle = col; g.font = "11px system-ui"; g.fillText(`banner #${id} ${this.labelName(label)}`, X(x) + 8, Y(y) + 14);
    }
    for (const d of f.detections) {
      const [label, x, y, res, id] = d, col = SLAM_RESULT_COLORS[res] || "#000", lcol = this.labelColor(label);
      if (res === 0 && byId[id]) {  // association line
        g.strokeStyle = col; g.lineWidth = 1.5; g.beginPath(); g.moveTo(X(x), Y(y)); g.lineTo(X(byId[id][2]), Y(byId[id][3])); g.stroke();
      }
      const px = X(x), py = Y(y);  // ✕ in the label's color, ring in the association result's color
      g.strokeStyle = col; g.lineWidth = 2; g.beginPath(); g.arc(px, py, 8, 0, 7); g.stroke();
      g.strokeStyle = lcol; g.lineWidth = 2.5;
      g.beginPath(); g.moveTo(px - 5, py - 5); g.lineTo(px + 5, py + 5); g.moveTo(px + 5, py - 5); g.lineTo(px - 5, py + 5); g.stroke();
    }
    for (const r of f.removed) {
      if (!r.position) continue;
      g.strokeStyle = "#dc2626"; g.lineWidth = 2; g.beginPath(); g.arc(X(r.position[0]), Y(r.position[1]), 9, 0, 7); g.stroke();
    }
    if (f.odom_pose && Math.hypot(f.odom_pose[0] - pose[0], f.odom_pose[1] - pose[1]) > 0.05) this.robot(g, X, Y, f.odom_pose, "#94a3b8");
    this.robot(g, X, Y, pose);
    this.root.querySelector(".mp-legend").innerHTML =
      `±${this.tf.R} m, ${this.tf.grid} m grid · colors = labels · landmarks: filled = seen in this update, hollow = not, dashed = 2σ · ` +
      `detections ✕ with a ring: ` + Object.entries(SLAM_RESULT_NAMES).map(([k, n]) => `<span style="color:${SLAM_RESULT_COLORS[k]}">◯ ${n}</span>`).join(" ") +
      ` · banners: bar = width, arrow = normal` +
      ` · <span style="color:#dc2626">○ removed</span>${f.odom_pose ? ' · <span style="color:#94a3b8">▲ odometry pose</span>' : ""}`;
    const u = f.update;
    this.root.querySelector(".mp-info").innerHTML = `
      <div><b>Pose</b> (${pose.map(v => v.toFixed(2)).join(", ")})${f.pose_cov ? ` · σxy ${Math.sqrt(f.pose_cov[0] + f.pose_cov[2]).toFixed(2)} m` : ""}</div>
      <div><b>${f.landmarks.length}</b> landmarks${u ? ` · ${esc(u.algorithm)}, threshold ${u.threshold}${u.distance_gate_m != null ? `, gate ${u.distance_gate_m} m` : ""}${u.labeled === false ? " · unlabeled batch" : ""}` : ""}</div>
      ${f.detections.length ? `<table class="outcomes"><tr><th>Detection</th><th>Result</th><th class="num">d²</th><th class="num">nearest d²</th></tr>
        ${f.detections.map(d => `<tr><td><span class="swatch" style="background:${this.labelColor(d[0])}"></span>${esc(this.labelName(d[0]))}</td>
          <td style="color:${SLAM_RESULT_COLORS[d[3]]}">${SLAM_RESULT_NAMES[d[3]]}${d[4] >= 0 ? ` #${d[4]}` : ""}</td>
          <td class="num">${d[5] >= 0 ? d[5] : ""}</td><td class="num">${d[7] >= 0 ? d[7] : ""}</td></tr>`).join("")}</table>`
        : '<div class="muted">No detections in this update.</div>'}
      ${f.removed.length ? `<div style="margin-top:6px"><b>Removed:</b> ${f.removed.map(r => `#${r.id} ${esc(this.labelName(r.label))} (${esc(r.reason)})`).join(", ")}</div>` : ""}`;
    this.root.querySelector(".mp-events").innerHTML = "";
  }

  drawTasks(g, f) {
    const ts = f.task_state || {}, nav = f.nav_control, ctl = f.control;
    const pose = f.pose || ts.pose || [0, 0, 0];
    const recent = (this.view.events || []).filter(e => e.t <= f.t && e.t > f.t - 60);
    const trail = (this.view.trail || []).filter(p => p[0] <= f.t).map(p => [p[1], p[2]]);
    // fit the whole map: obstacles, waypoint, gate, carrot, every goal sent so far, the robot and its trail
    const pts = [pose, ...(ts.obstacles || []).map(o => o.slice(0, 2)), ...trail];
    if (ts.waypoint) pts.push(ts.waypoint);
    if (ts.gate) pts.push(ts.gate.left, ts.gate.right);
    if (nav?.carrot) pts.push(nav.carrot);
    for (const e of this.view.events || []) if (e.t <= f.t && e.target) pts.push(e.target);
    const [X, Y] = this.setup(pts);
    this.grid(g);
    this.trail(g, X, Y, trail, "#2557d6");
    for (const o of ts.obstacles || []) {
      g.fillStyle = this.labelColor(o[2]); g.beginPath(); g.arc(X(o[0]), Y(o[1]), 5, 0, 7); g.fill();
    }
    // navigation goals of the last minute (older = fainter)
    for (const e of recent.filter(e => e.target && e.event.startsWith("nav_goal") || e.event === "waypoint_goal" || e.event === "follow_path_goal")) {
      if (!e.target) continue;
      g.globalAlpha = 0.25 + 0.75 * (1 - (f.t - e.t) / 60);
      g.strokeStyle = "#7b3fd6"; g.lineWidth = 2; const px = X(e.target[0]), py = Y(e.target[1]);
      g.strokeRect(px - 5, py - 5, 10, 10); g.globalAlpha = 1;
    }
    if (ts.gate) {
      const l = ts.gate.left, r = ts.gate.right;
      g.strokeStyle = "#f59e0b"; g.lineWidth = 3; g.beginPath(); g.moveTo(X(l[0]), Y(l[1])); g.lineTo(X(r[0]), Y(r[1])); g.stroke();
      for (const b of [l, r]) { g.beginPath(); g.arc(X(b[0]), Y(b[1]), 9, 0, 7); g.stroke(); }
    }
    if (ts.gate_waypoint) { g.fillStyle = "#f59e0b"; g.beginPath(); g.arc(X(ts.gate_waypoint[0]), Y(ts.gate_waypoint[1]), 4, 0, 7); g.fill(); }
    if (ts.waypoint) {  // the task's current waypoint: star
      const px = X(ts.waypoint[0]), py = Y(ts.waypoint[1]);
      g.fillStyle = "#7b3fd6"; g.beginPath();
      for (let j = 0; j < 10; j++) { const rr = j % 2 ? 4 : 10, a = j * Math.PI / 5 - Math.PI / 2; g.lineTo(px + rr * Math.cos(a), py + rr * Math.sin(a)); }
      g.closePath(); g.fill();
    }
    if (nav?.carrot) {
      g.strokeStyle = "#0f8b8d"; g.setLineDash([4, 3]); g.beginPath(); g.moveTo(X(pose[0]), Y(pose[1])); g.lineTo(X(nav.carrot[0]), Y(nav.carrot[1])); g.stroke(); g.setLineDash([]);
      g.fillStyle = "#0f8b8d"; g.beginPath(); g.arc(X(nav.carrot[0]), Y(nav.carrot[1]), 4, 0, 7); g.fill();
    }
    const steer = ctl || nav;
    if (steer) {
      this.arrow(g, X, Y, pose, steer.cmd, 1, "#2557d6");  // 1 m per m/s
      this.arrow(g, X, Y, pose, steer.avoid_vel, 1, "#dc2626");
    }
    for (const e of recent.filter(e => e.event === "report" || e.event.startsWith("gate_"))) {  // markers at the robot pose / buoys
      if (e.event === "report" && e.pose) { g.fillStyle = "#db2777"; g.font = "bold 14px system-ui"; g.fillText("⚑", X(e.pose[0]) - 4, Y(e.pose[1]) - 8); }
    }
    this.robot(g, X, Y, pose);
    this.root.querySelector(".mp-legend").innerHTML = `±${this.tf.R} m, ${this.tf.grid} m grid · ` +
      `<span style="color:#7b3fd6">★ current waypoint, □ goals sent (last 60 s)</span> · <span style="color:#f59e0b">— gate</span> · ` +
      `<span style="color:#0f8b8d">● carrot (nav)</span> · <span style="color:#2557d6">→ command</span> · <span style="color:#dc2626">→ obstacle avoidance</span> (1 m per m/s) · ` +
      `<span style="color:#db2777">⚑ report</span> · dots: obstacles the task sees`;
    const stateNow = (this.view.states || []).filter(s => s.start <= f.t && s.end >= f.t);
    this.root.querySelector(".mp-info").innerHTML = `
      <div><b>Pose</b> (${pose.map(v => (+v).toFixed(2)).join(", ")})</div>
      ${stateNow.length ? stateNow.map(s => `<div><b>${esc(s.task)}</b>: ${esc(s.state)}</div>`).join("") : '<div class="muted">No task running.</div>'}
      ${ts.waypoint ? `<div>Waypoint (${ts.waypoint.map(v => v.toFixed(2)).join(", ")})</div>` : ""}
      ${ts.gate ? `<div>Gate L (${ts.gate.left.map(v => v.toFixed(1))}) R (${ts.gate.right.map(v => v.toFixed(1))})</div>` : ""}
      ${nav ? `<div>Nav: ${nav.distance} m to goal, heading → ${(nav.desired_heading * 180 / Math.PI).toFixed(0)}°</div>` : ""}
      ${steer ? `<div>Command (${steer.cmd.join(", ")}), avoidance (${steer.avoid_vel.join(", ")})</div>` : ""}
      ${ctl ? `<div>Controller error (robot frame) (${ctl.error_robot.join(", ")})</div>` : ""}`;
    const evs = recent.slice(-40).reverse();
    this.root.querySelector(".mp-events").innerHTML = evs.length ? `<div class="small-title muted">Decisions (last 60 s, newest first; click to jump)</div>
      <table class="outcomes mp-evtable">${evs.map(e => `<tr data-t="${e.t}" class="${e.level === "WARN" || e.level === "ERROR" ? "bad" : ""}">
        <td class="num">${e.t.toFixed(1)}</td><td>${esc(e.node || "")}</td><td><b>${esc(e.event)}</b></td>
        <td class="muted">${esc(describeEvent(e))}</td></tr>`).join("")}</table>` : "";
    this.root.querySelectorAll(".mp-evtable tr").forEach(tr => tr.onclick = () => {
      this.follow = false; const j = this.frames().findIndex(fr => fr.t >= +tr.dataset.t); if (j >= 0) this.show(j);
    });
  }

  drawStates() {  // task state bands under the slider
    const box = this.root.querySelector(".mp-states"), st = this.view?.states || [], fr = this.frames();
    if (!st.length || !fr.length) { box.innerHTML = ""; return; }
    const t0 = fr[0].t, t1 = fr[fr.length - 1].t || 1, names = [...new Set(st.map(s => `${s.task}: ${s.state}`))];
    box.innerHTML = `<div class="mp-band">${st.map(s => {
      const name = `${s.task}: ${s.state}`, l = (s.start - t0) / (t1 - t0) * 100, w = Math.max(0.3, (s.end - s.start) / (t1 - t0) * 100);
      return `<div title="${esc(name)} (${s.start.toFixed(1)}–${s.end.toFixed(1)} s)" style="left:${l}%;width:${w}%;background:${STATE_COLORS[names.indexOf(name) % STATE_COLORS.length]}"></div>`;
    }).join("")}</div><div class="legend">${names.map((n, j) => `<span style="--c:${STATE_COLORS[j % STATE_COLORS.length]}">${esc(n)}</span>`).join("")}</div>`;
  }
}

// One-line description of a task/navigation decision event (the fields that explain it)
function describeEvent(e) {
  const xy = v => Array.isArray(v) ? `(${v.slice(0, 2).map(n => (+n).toFixed(1)).join(", ")})` : "";
  switch (e.event) {
    case "task_attempt": return `${e.task} ${e.action_type}${e.search_target ? " → " + xy(e.search_target) : ""}`;
    case "task_result": return `${e.task}: ${e.outcome}`;
    case "task_goal_response": case "task_goal": return `${e.task}: ${e.accepted ? "accepted" : "rejected"}`;
    case "task_state_change": return `${e.task}: ${e.previous ?? "–"} → ${e.state}`;
    case "task_phase": return `${e.task}: ${e.phase}`;
    case "task_finished": return `${e.task}: ${e.success ? "success" : "failed"}`;
    case "nav_goal": case "nav_goal_updated": return `${e.kind} → ${xy(e.target)}`;
    case "nav_goal_result": return e.result;
    case "nav_goal_resent": return e.reason;
    case "waypoint_goal": case "follow_path_goal": case "continuous_waypoint_update": return `→ ${xy(e.target)}`;
    case "waypoint_crossed": return `${xy(e.target)} at ${e.distance} m`;
    case "gate_found": case "gate_fake": return `L ${xy(e.left)} R ${xy(e.right)}`;
    case "gate_not_found": return e.reason + (e.pairs_too_close?.length ? `; too close ${e.pairs_too_close}` : "") + (e.pairs_too_far?.length ? `; too far ${e.pairs_too_far}` : "");
    case "report": return `${e.type} ${JSON.stringify(e.data || {}).slice(0, 120)}`;
    case "process_start": case "process_end": return e.message || "";
    case "robot_state": return e.state;
    case "mux_source": return `${e.previous ?? "–"} → ${e.source}` + (e.switches > 1 ? ` (${e.switches} switches within 1 s of each other: sources alternating)` : "");
    case "slam_restart": return `position ${e.position}, buoys ${e.buoys}, banners ${e.banners}`;
    default: return "";
  }
}
