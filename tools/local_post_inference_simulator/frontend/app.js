"use strict";

// Phase 1 UI: canvas scenario editor. This file only builds shape/mask data
// and calls the backend bridge; it must never compute IPM/planning itself.
// See docs/local_post_inference_simulator/plan.md Phase 1.

const CANVAS_EL = document.getElementById("scene-canvas");
const CTX = CANVAS_EL.getContext("2d");

const SCOPE_CLASS_NAMES = [
  "main-lane", "other-lane", "turn-lane",
  "dashed-white", "dashed-yellow", "double-solid-white",
  "solid-white", "solid-yellow", "stop-line",
];
const POLYGON_DEFAULT_CLASSES = new Set(["main-lane", "other-lane", "turn-lane"]);
const ROUTE_INTENTS = ["follow_main", "turn_left", "turn_right", "lane_change_left", "lane_change_right"];
const VERTEX_HIT_PX = 7;
const EDGE_HIT_PX = 6;

let labelMapping = []; // [{label, class_name, color}] loaded from /api/label_mapping
let scenario = emptyScenario();
let currentFrameIdx = 0;
let selectedObjectId = null;
let drawMode = "idle"; // "idle" | "drawing"
let inProgressPoints = [];
let draggingPointIndex = null;
let objectCounters = {}; // class_name -> highest used numeric suffix
let mousePos = null;
let statusPollTimer = null;

function resetSelectionState() {
  selectedObjectId = null;
  drawMode = "idle";
  inProgressPoints = [];
}

function emptyScenario() {
  return {
    name: "untitled_scenario",
    canvas: { width: 640, height: 480 },
    calibration: { source: "config/calibration.json" },
    route_intent: { intent: "follow_main", seq: 1 },
    frames: [{ frame_id: 1, objects: [] }],
  };
}

// ---------- Backend bootstrap ----------

async function loadLabelMapping() {
  try {
    const res = await fetch("/api/label_mapping");
    if (!res.ok) throw new Error("HTTP error");
    const data = await res.json();
    labelMapping = (data.labels || []).filter((entry) => SCOPE_CLASS_NAMES.indexOf(entry.class_name) !== -1);
  } catch (err) {
    alert("Error: Cannot connect to backend to fetch label mapping. Please restart the backend.");
    labelMapping = [];
  }
  populateClassSelect();
}

function populateClassSelect() {
  const select = document.getElementById("class-select");
  select.innerHTML = "";
  for (const entry of labelMapping) {
    const opt = document.createElement("option");
    opt.value = entry.class_name;
    opt.textContent = entry.label + ": " + entry.class_name;
    select.appendChild(opt);
  }
  if (labelMapping.length > 0) {
    selectedObjectId = null;
    selectedClassName = labelMapping[0].class_name;
    select.value = selectedClassName;
  }
  syncShapeSelectWithClass();
}

let selectedClassName = null;

function classInfo(className) {
  for (const entry of labelMapping) {
    if (entry.class_name === className) return entry;
  }
  return null;
}

function syncShapeSelectWithClass() {
  const shapeSelect = document.getElementById("shape-select");
  shapeSelect.value = POLYGON_DEFAULT_CLASSES.has(selectedClassName) ? "polygon" : "polyline";
}

function populateRouteIntentSelect() {
  const select = document.getElementById("route-intent-select");
  select.innerHTML = "";
  for (const intent of ROUTE_INTENTS) {
    const opt = document.createElement("option");
    opt.value = intent;
    opt.textContent = intent;
    select.appendChild(opt);
  }
  select.value = scenario.route_intent.intent;
}

// ---------- Scenario state helpers ----------

function currentFrame() {
  return scenario.frames[currentFrameIdx];
}

function findObjectById(id) {
  const frame = currentFrame();
  for (const obj of frame.objects) {
    if (obj.id === id) return obj;
  }
  return null;
}

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function clampConfidence(v) {
  if (Number.isNaN(v)) return 1.0;
  return clamp(v, 0, 1);
}

function autoObjectId(className) {
  const next = (objectCounters[className] || 0) + 1;
  objectCounters[className] = next;
  return className + "_" + next;
}

function bumpObjectCounterFromId(className, id) {
  if (typeof id !== "string") return;
  const prefix = className + "_";
  if (id.indexOf(prefix) !== 0) return;
  const num = parseInt(id.slice(prefix.length), 10);
  if (!Number.isNaN(num)) {
    objectCounters[className] = Math.max(objectCounters[className] || 0, num);
  }
}

// ---------- Geometry / hit testing ----------

function pointInPolygon(px, py, points) {
  let inside = false;
  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    const xi = points[i][0], yi = points[i][1];
    const xj = points[j][0], yj = points[j][1];
    const intersect = (yi > py) !== (yj > py) &&
      px < ((xj - xi) * (py - yi)) / (yj - yi) + xi;
    if (intersect) inside = !inside;
  }
  return inside;
}

function distToSegment(px, py, x1, y1, x2, y2) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const lengthSq = dx * dx + dy * dy;
  let t = lengthSq === 0 ? 0 : ((px - x1) * dx + (py - y1) * dy) / lengthSq;
  t = clamp(t, 0, 1);
  const cx = x1 + t * dx;
  const cy = y1 + t * dy;
  return Math.hypot(px - cx, py - cy);
}

function nearestVertexIndex(points, x, y, threshold) {
  let bestIdx = -1;
  let bestDist = threshold;
  points.forEach((pt, idx) => {
    const d = Math.hypot(pt[0] - x, pt[1] - y);
    if (d <= bestDist) {
      bestDist = d;
      bestIdx = idx;
    }
  });
  return bestIdx;
}

function objectAtPoint(px, py) {
  const frame = currentFrame();
  for (let i = frame.objects.length - 1; i >= 0; i--) {
    const obj = frame.objects[i];
    const points = obj.points_px;
    if (obj.shape === "polygon" && points.length >= 3 && pointInPolygon(px, py, points)) {
      return obj;
    }
    const numEdges = obj.shape === "polygon" ? points.length : points.length - 1;
    for (let k = 0; k < numEdges; k++) {
      const nextIdx = (k + 1) % points.length;
      if (distToSegment(px, py, points[k][0], points[k][1], points[nextIdx][0], points[nextIdx][1]) < EDGE_HIT_PX) {
        return obj;
      }
    }
  }
  return null;
}

// ---------- Drawing lifecycle ----------

function startDrawing() {
  if (!selectedClassName) {
    alert("Chưa có class nào để vẽ (label mapping chưa tải xong?).");
    return;
  }
  resetSelectionState();
  drawMode = "drawing";
  renderObjectTable();
  render();
}

function cancelDrawing() {
  drawMode = "idle";
  inProgressPoints = [];
  render();
}

function finalizeShape() {
  if (drawMode !== "drawing") return;
  
  // Lọc điểm trùng liên tiếp (do lỗi double click trình duyệt)
  const cleanPoints = [];
  for (const pt of inProgressPoints) {
    if (cleanPoints.length === 0) {
      cleanPoints.push([pt[0], pt[1]]);
    } else {
      const last = cleanPoints[cleanPoints.length - 1];
      if (last[0] !== pt[0] || last[1] !== pt[1]) {
        cleanPoints.push([pt[0], pt[1]]);
      }
    }
  }

  const shapeType = document.getElementById("shape-select").value;
  const minPoints = shapeType === "polygon" ? 3 : 2;
  if (cleanPoints.length < minPoints) {
    alert("Cần tối thiểu " + minPoints + " điểm hợp lệ cho shape loại " + shapeType + ".");
    return;
  }
  const info = classInfo(selectedClassName);
  if (!info) {
    alert("Chưa chọn class hợp lệ.");
    return;
  }
  const frame = currentFrame();
  let id = document.getElementById("object-id-input").value.trim();
  if (!id) {
    id = autoObjectId(selectedClassName);
  } else {
    bumpObjectCounterFromId(selectedClassName, id);
  }
  if (frame.objects.some((o) => o.id === id)) {
    alert('Object id "' + id + '" đã tồn tại trong frame này. Chọn id khác.');
    return;
  }
  const confidence = clampConfidence(parseFloat(document.getElementById("object-confidence-input").value));
  frame.objects.push({
    id,
    class_name: selectedClassName,
    label: info.label,
    shape: shapeType,
    points_px: cleanPoints,
    confidence,
    enabled: true,
  });
  inProgressPoints = [];
  drawMode = "idle";
  document.getElementById("object-id-input").value = "";
  selectedObjectId = id;
  renderObjectTable();
  render();
}

// Dedicated ID-swap control (Phase 6): renames an object's id within the
// current frame only, so a scenario can simulate a tracker ID swap between
// frames (same geometry, different id) without hand-editing exported JSON.
function renameObjectIdInCurrentFrame(oldId, newId) {
  const trimmed = (newId || "").trim();
  if (!trimmed || trimmed === oldId) return false;
  const frame = currentFrame();
  if (frame.objects.some((o) => o.id === trimmed)) {
    alert('Object id "' + trimmed + '" đã tồn tại trong frame này. Chọn id khác.');
    return false;
  }
  const obj = frame.objects.find((o) => o.id === oldId);
  if (!obj) return false;
  obj.id = trimmed;
  if (selectedObjectId === oldId) selectedObjectId = trimmed;
  renderObjectTable();
  render();
  return true;
}

function deleteObject(id) {
  const frame = currentFrame();
  const idx = frame.objects.findIndex((o) => o.id === id);
  if (idx === -1) return;
  frame.objects.splice(idx, 1);
  if (selectedObjectId === id) resetSelectionState();
  renderObjectTable();
  render();
}

// ---------- Frame management ----------



function updateFrameRouteIntentUI() {
  const select = document.getElementById("frame-route-intent-select");
  if (scenario.frames.length === 0) return;
  const frame = scenario.frames[currentFrameIdx];
  if (frame.route_intent && frame.route_intent.intent) {
    select.value = frame.route_intent.intent;
  } else {
    select.value = "";
  }
}

let runningFrameIdx = null; // backend's running_frame_idx: the frame a step() call is actually processing right now (Phase 6 timeline feedback)

function renderFrameList() {
  const container = document.getElementById("frame-list");
  container.innerHTML = "";
  scenario.frames.forEach((frame, idx) => {
    const chip = document.createElement("div");
    let cls = "frame-chip";
    if (idx === currentFrameIdx) cls += " active";
    if (runningFrameIdx !== null && idx === runningFrameIdx) cls += " running";
    chip.className = cls;
    chip.textContent = "F" + frame.frame_id;
    chip.title = frame.objects.length + " object(s)" + (idx === runningFrameIdx ? " (đang chạy trên ROS)" : "");
    chip.addEventListener("click", () => {
      resetSelectionState();
      currentFrameIdx = idx;
      renderFrameList();
      updateFrameRouteIntentUI();
      renderObjectTable();
      render();
    });
    container.appendChild(chip);
  });
}

function duplicateCurrentFrame() {
  const frame = currentFrame();
  const maxId = Math.max(...scenario.frames.map((f) => f.frame_id));
  const cloned = {
    frame_id: maxId + 1,
    objects: frame.objects.map((o) => ({ ...o, points_px: o.points_px.map((p) => [p[0], p[1]]) })),
  };
  if (frame.route_intent) {
    cloned.route_intent = { ...frame.route_intent };
  }
  scenario.frames.push(cloned);
  currentFrameIdx = scenario.frames.length - 1;
  resetSelectionState();
  renderFrameList();
  updateFrameRouteIntentUI();
  renderObjectTable();
  render();
}

function deleteCurrentFrame() {
  if (scenario.frames.length <= 1) {
    alert("Scenario cần tối thiểu 1 frame.");
    return;
  }
  scenario.frames.splice(currentFrameIdx, 1);
  currentFrameIdx = Math.max(0, currentFrameIdx - 1);
  resetSelectionState();
  renderFrameList();
  updateFrameRouteIntentUI();
  renderObjectTable();
  render();
}

// ---------- Object table ----------

function renderObjectTable() {
  const tbody = document.getElementById("object-table-body");
  tbody.innerHTML = "";
  const frame = currentFrame();
  for (const obj of frame.objects) {
    const tr = document.createElement("tr");
    if (obj.id === selectedObjectId) tr.classList.add("selected");

    const tdEnabled = document.createElement("td");
    const cb = document.createElement("input");
    cb.type = "checkbox";
    cb.checked = obj.enabled !== false;
    cb.addEventListener("change", () => {
      obj.enabled = cb.checked;
      render();
    });
    tdEnabled.appendChild(cb);

    const tdId = document.createElement("td");
    tdId.textContent = obj.id;
    tdId.style.cursor = "pointer";
    tdId.setAttribute("data-editable", "id");
    tdId.title = "Click để chọn, double-click để đổi id (test ID-swap giữa các frame)";
    tdId.addEventListener("click", () => {
      selectedObjectId = obj.id;
      drawMode = "idle";
      renderObjectTable();
      render();
    });
    tdId.addEventListener("dblclick", (evt) => {
      evt.stopPropagation();
      const next = prompt('Đổi id cho object trong frame hiện tại (chỉ frame ' + currentFrame().frame_id + '):', obj.id);
      if (next !== null) renameObjectIdInCurrentFrame(obj.id, next);
    });

    const tdClass = document.createElement("td");
    tdClass.textContent = obj.label + ":" + obj.class_name;

    const tdShape = document.createElement("td");
    tdShape.textContent = obj.shape;

    const tdConf = document.createElement("td");
    tdConf.textContent = obj.confidence.toFixed(2);

    const tdActions = document.createElement("td");
    const delBtn = document.createElement("button");
    delBtn.textContent = "✕";
    delBtn.className = "link-btn";
    delBtn.title = "Xoá object";
    delBtn.addEventListener("click", (evt) => {
      evt.stopPropagation();
      deleteObject(obj.id);
    });
    tdActions.appendChild(delBtn);

    tr.append(tdEnabled, tdId, tdClass, tdShape, tdConf, tdActions);
    tbody.appendChild(tr);
  }
}

// ---------- Canvas rendering ----------

function hexToRgba(hex, alpha) {
  const clean = hex.replace("#", "");
  const r = parseInt(clean.substring(0, 2), 16);
  const g = parseInt(clean.substring(2, 4), 16);
  const b = parseInt(clean.substring(4, 6), 16);
  return "rgba(" + r + ", " + g + ", " + b + ", " + alpha + ")";
}

function updateDrawStatus() {
  const el = document.getElementById("draw-status");
  if (drawMode === "drawing") {
    const shapeType = document.getElementById("shape-select").value;
    el.textContent = "drawing " + (selectedClassName || "") + " (" + shapeType + ") - " + inProgressPoints.length + " điểm";
  } else if (selectedObjectId) {
    el.textContent = "selected: " + selectedObjectId;
  } else {
    el.textContent = "idle";
  }
}

function drawObject(obj, isSelected) {
  const info = classInfo(obj.class_name);
  const baseColor = (info && info.color) || "#4da6ff";
  const disabled = obj.enabled === false;
  const points = obj.points_px;
  if (points.length === 0) return;

  // Captured real-inference objects can carry extra mask contours beyond the
  // editable points_px (see ObjectSchema.polygons_px). Render them read-only
  // so the canvas shows the full mask that the pipeline will receive.
  if (Array.isArray(obj.polygons_px) && obj.polygons_px.length > 1) {
    CTX.lineWidth = 1.5;
    CTX.strokeStyle = disabled ? "rgba(255,255,255,0.15)" : hexToRgba(baseColor, 0.7);
    CTX.fillStyle = disabled ? "rgba(255,255,255,0.03)" : hexToRgba(baseColor, 0.18);
    for (let k = 1; k < obj.polygons_px.length; k++) {
      const extra = obj.polygons_px[k];
      if (!extra || extra.length < 2) continue;
      CTX.beginPath();
      CTX.moveTo(extra[0][0], extra[0][1]);
      for (let i = 1; i < extra.length; i++) CTX.lineTo(extra[i][0], extra[i][1]);
      CTX.closePath();
      CTX.fill();
      CTX.stroke();
    }
  }

  CTX.lineWidth = isSelected ? 3 : 2;
  CTX.strokeStyle = disabled ? "rgba(255,255,255,0.25)" : baseColor;

  CTX.beginPath();
  CTX.moveTo(points[0][0], points[0][1]);
  for (let i = 1; i < points.length; i++) CTX.lineTo(points[i][0], points[i][1]);

  if (obj.shape === "polygon" && points.length >= 3) {
    CTX.closePath();
    CTX.fillStyle = disabled ? "rgba(255,255,255,0.05)" : hexToRgba(baseColor, 0.32);
    CTX.fill();
    CTX.stroke();
  } else {
    CTX.lineWidth = disabled ? 2 : 4;
    CTX.stroke();
  }

  for (const pt of points) {
    CTX.beginPath();
    CTX.arc(pt[0], pt[1], isSelected ? 5 : 3, 0, Math.PI * 2);
    CTX.fillStyle = isSelected ? "#4da6ff" : "#ffffff";
    CTX.fill();
  }

  if (!disabled) {
    CTX.font = "11px sans-serif";
    CTX.lineWidth = 3;
    CTX.strokeStyle = "#000000";
    CTX.strokeText(obj.id, points[0][0] + 6, points[0][1] - 6);
    CTX.fillStyle = "#ffffff";
    CTX.fillText(obj.id, points[0][0] + 6, points[0][1] - 6);
  }
}

function drawInProgressShape() {
  if (inProgressPoints.length === 0) return;
  const shapeType = document.getElementById("shape-select").value;
  const info = classInfo(selectedClassName);
  const color = (info && info.color) || "#4da6ff";

  CTX.setLineDash([6, 4]);
  CTX.strokeStyle = color;
  CTX.lineWidth = 2;
  CTX.beginPath();
  CTX.moveTo(inProgressPoints[0][0], inProgressPoints[0][1]);
  for (let i = 1; i < inProgressPoints.length; i++) {
    CTX.lineTo(inProgressPoints[i][0], inProgressPoints[i][1]);
  }
  if (mousePos) {
    CTX.lineTo(mousePos[0], mousePos[1]);
  }
  if (shapeType === "polygon" && inProgressPoints.length >= 2) {
    CTX.lineTo(inProgressPoints[0][0], inProgressPoints[0][1]);
  }
  CTX.stroke();
  CTX.setLineDash([]);

  for (const pt of inProgressPoints) {
    CTX.beginPath();
    CTX.arc(pt[0], pt[1], 4, 0, Math.PI * 2);
    CTX.fillStyle = color;
    CTX.fill();
  }
}

function render() {
  updateDrawStatus();

  CTX.fillStyle = "#000000";
  CTX.fillRect(0, 0, CANVAS_EL.width, CANVAS_EL.height);

  CTX.strokeStyle = "rgba(255,255,255,0.05)";
  CTX.lineWidth = 1;
  for (let gx = 0; gx <= CANVAS_EL.width; gx += 40) {
    CTX.beginPath();
    CTX.moveTo(gx, 0);
    CTX.lineTo(gx, CANVAS_EL.height);
    CTX.stroke();
  }
  for (let gy = 0; gy <= CANVAS_EL.height; gy += 40) {
    CTX.beginPath();
    CTX.moveTo(0, gy);
    CTX.lineTo(CANVAS_EL.width, gy);
    CTX.stroke();
  }

  const frame = currentFrame();
  for (const obj of frame.objects) {
    drawObject(obj, obj.id === selectedObjectId);
  }

  if (drawMode === "drawing") {
    drawInProgressShape();
  }
}

// ---------- Canvas input ----------

function canvasPointFromEvent(evt) {
  const rect = CANVAS_EL.getBoundingClientRect();
  const scaleX = CANVAS_EL.width / rect.width;
  const scaleY = CANVAS_EL.height / rect.height;
  const x = clamp(Math.round((evt.clientX - rect.left) * scaleX), 0, CANVAS_EL.width);
  const y = clamp(Math.round((evt.clientY - rect.top) * scaleY), 0, CANVAS_EL.height);
  return [x, y];
}

function onCanvasClick(evt) {
  if (drawMode !== "drawing") return;
  const [x, y] = canvasPointFromEvent(evt);
  inProgressPoints.push([x, y]);
  render();
}

function onCanvasDblClick(evt) {
  if (drawMode !== "drawing") return;
  evt.preventDefault();
  
  if (inProgressPoints.length > 0) {
    inProgressPoints.pop();
  }
  
  finalizeShape();
}

function onCanvasMouseDown(evt) {
  if (drawMode === "drawing") return;
  const [x, y] = canvasPointFromEvent(evt);

  if (selectedObjectId) {
    const obj = findObjectById(selectedObjectId);
    if (obj) {
      const idx = nearestVertexIndex(obj.points_px, x, y, VERTEX_HIT_PX);
      if (idx !== -1) {
        draggingPointIndex = idx;
        return;
      }
    }
  }

  const hit = objectAtPoint(x, y);
  selectedObjectId = hit ? hit.id : null;
  renderObjectTable();
  render();
}

function onCanvasMouseMove(evt) {
  const [x, y] = canvasPointFromEvent(evt);
  mousePos = [x, y];

  if (draggingPointIndex !== null && selectedObjectId) {
    const obj = findObjectById(selectedObjectId);
    if (obj) {
      obj.points_px[draggingPointIndex] = [x, y];
      // Captured objects mirror points_px as their first contour; keep them in
      // sync so the payload reflects the edit instead of the stale capture.
      if (Array.isArray(obj.polygons_px) && obj.polygons_px.length > 0) {
        obj.polygons_px[0] = obj.points_px.map((p) => [p[0], p[1]]);
      }
    }
  }
  render();
}

function onCanvasMouseUp() {
  draggingPointIndex = null;
}

// ---------- Scenario panel sync ----------

function syncScenarioPanelFromState() {
  document.getElementById("scenario-name").value = scenario.name;
  document.getElementById("canvas-width").value = scenario.canvas.width;
  document.getElementById("canvas-height").value = scenario.canvas.height;
  document.getElementById("calibration-source").value = scenario.calibration.source;
  document.getElementById("route-intent-seq").value = scenario.route_intent.seq;
}

function resizeCanvasElement() {
  CANVAS_EL.width = scenario.canvas.width;
  CANVAS_EL.height = scenario.canvas.height;
}

function newScenario() {
  if (!confirm("Xoá scenario hiện tại và tạo scenario trống mới?")) return;
  scenario = emptyScenario();
  objectCounters = {};
  currentFrameIdx = 0;
  resetSelectionState();
  syncScenarioPanelFromState();
  populateRouteIntentSelect();
  resizeCanvasElement();
  renderFrameList();
  updateFrameRouteIntentUI();
  renderObjectTable();
  render();
}

// ---------- Export / Import ----------

function buildExportScenario() {
  return {
    name: scenario.name,
    canvas: { width: scenario.canvas.width, height: scenario.canvas.height },
    calibration: { source: scenario.calibration.source },
    route_intent: { intent: scenario.route_intent.intent, seq: scenario.route_intent.seq },
    frames: scenario.frames.map((frame) => {
      const fr = {
        frame_id: frame.frame_id,
        objects: frame.objects
          .filter((obj) => obj.enabled !== false)
          .map((obj) => ({
            id: obj.id,
            class_name: obj.class_name,
            label: obj.label,
            shape: obj.shape,
            points_px: obj.points_px,
            confidence: obj.confidence,
          })),
      };
      if (frame.route_intent) {
        fr.route_intent = {
          intent: frame.route_intent.intent,
          seq: frame.route_intent.seq
        };
      }
      return fr;
    }),
  };
}

function exportScenarioToFile() {
  const data = buildExportScenario();
  const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = (data.name || "scenario") + ".json";
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

function normalizeImportedObject(obj) {
  const className = obj.class_name;
  bumpObjectCounterFromId(className, obj.id);
  
  const info = classInfo(className);
  const correctLabel = info ? info.label : (obj.label != null ? obj.label : -1);

  const shape = obj.shape || "polygon";
  const points_px = (obj.points_px || []).map((p) => [p[0], p[1]]);
  
  if (shape === "polygon" && points_px.length < 3) {
      console.warn(`Ignoring imported polygon ${obj.id} because it has less than 3 points.`);
      return null;
  }
  if (shape === "polyline" && points_px.length < 2) {
      console.warn(`Ignoring imported polyline ${obj.id} because it has less than 2 points.`);
      return null;
  }

  return {
    id: obj.id,
    class_name: className,
    label: correctLabel,
    shape: shape,
    points_px: points_px,
    confidence: obj.confidence != null ? obj.confidence : 1.0,
    enabled: true,
  };
}

function applyImportedScenario(parsed) {
  if (!parsed || !Array.isArray(parsed.frames) || parsed.frames.length === 0) {
    alert('Scenario JSON thiếu "frames".');
    return;
  }
  objectCounters = {};
  scenario = {
    name: parsed.name || "imported_scenario",
    canvas: {
      width: (parsed.canvas && parsed.canvas.width) || 640,
      height: (parsed.canvas && parsed.canvas.height) || 480,
    },
    calibration: { source: (parsed.calibration && parsed.calibration.source) || "config/calibration.json" },
    route_intent: {
      intent: (parsed.route_intent && parsed.route_intent.intent) || "follow_main",
      seq: (parsed.route_intent && parsed.route_intent.seq) != null ? parsed.route_intent.seq : 1,
    },
    frames: parsed.frames.map((frame, idx) => {
      const existingIds = new Set();
      const validObjects = [];
      for (const rawObj of (frame.objects || [])) {
        if (!existingIds.has(rawObj.id)) {
          const normObj = normalizeImportedObject(rawObj);
          if (normObj) {
            existingIds.add(rawObj.id);
            validObjects.push(normObj);
          }
        } else {
          console.warn(`Ignoring imported object with duplicate id ${rawObj.id}`);
        }
      }
      const parsedFrame = {
        frame_id: frame.frame_id != null ? frame.frame_id : idx + 1,
        objects: validObjects,
      };
      if (frame.route_intent) {
        parsedFrame.route_intent = {
          intent: frame.route_intent.intent,
          seq: frame.route_intent.seq != null ? frame.route_intent.seq : 1
        };
      }
      return parsedFrame;
    }),
  };
  currentFrameIdx = 0;
  resetSelectionState();
  syncScenarioPanelFromState();
  populateRouteIntentSelect();
  resizeCanvasElement();
  renderFrameList();
  updateFrameRouteIntentUI();
  renderObjectTable();
  render();
}

function importScenarioFromFile(file) {
  const reader = new FileReader();
  reader.onload = () => {
    try {
      const parsed = JSON.parse(reader.result);
      applyImportedScenario(parsed);
    } catch (err) {
      alert("File JSON không hợp lệ: " + err.message);
    }
  };
  reader.readAsText(file);
}

// ---------- Backend run controls ----------

function setOutputJson(obj) {
  document.getElementById("output-json").textContent = JSON.stringify(obj, null, 2);
}

function renderRunnerStatus(status) {
  document.getElementById("runner-status").textContent = JSON.stringify(status, null, 2);
}

function renderRunnerError(err) {
  const msg = err && err.message ? err.message : String(err);
  document.getElementById("runner-status").textContent =
    "Error: " + msg + "\n(Kiểm tra FastAPI backend + ipm_transform_node/control_node đang chạy, xem README.md)";
}

function refreshPreviewImage() {
  document.getElementById("preview-image").src = "/api/scenarios/preview?ts=" + Date.now();
}

function refreshBevImage() {
  const params = new URLSearchParams({
    show_slices: document.getElementById("bev-show-slices").checked,
    show_raw_midpoints: document.getElementById("bev-show-raw").checked,
    show_filtered_midpoints: document.getElementById("bev-show-filtered").checked,
    show_candidate_trajectory: document.getElementById("bev-show-candidate").checked,
    show_normalized_trajectory: document.getElementById("bev-show-normalized").checked,
    show_committed_trajectory: document.getElementById("bev-show-committed").checked,
    ts: Date.now(),
  });
  document.getElementById("bev-image").src = "/api/ipm/bev?" + params.toString();
}

async function loadScenarioToBackend() {
  const mode = document.getElementById("run-mode-select").value;
  try {
    const res = await fetch("/api/scenarios/load?mode=" + encodeURIComponent(mode), {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(buildExportScenario()),
    });
    const data = await res.json();
    if (!res.ok) throw new Error(data.detail || "load failed");
    renderRunnerStatus(data.runner);
    setOutputJson(data);
    refreshPreviewImage();
    refreshBevImage();
  } catch (err) {
    renderRunnerError(err);
  }
}

async function postRunnerAction(path) {
  try {
    const res = await fetch(path, { method: "POST" });
    const data = await res.json();
    if (!res.ok) throw new Error(data.detail || (path + " failed"));
    renderRunnerStatus(data.runner);
    if (Object.prototype.hasOwnProperty.call(data, "step_result")) {
      setOutputJson(data.step_result);
    } else {
      setOutputJson(data);
    }
    refreshPreviewImage();
    refreshBevImage();
  } catch (err) {
    renderRunnerError(err);
  }
}

async function fetchReport() {
  try {
    const res = await fetch("/api/scenarios/report");
    const data = await res.json();
    if (res.ok && data.report) {
      document.getElementById("report-content").textContent = JSON.stringify(data.report.metrics, null, 2);
      document.getElementById("report-modal").style.display = "block";
    }
  } catch (err) {
    console.error("Failed to fetch report", err);
  }
}

let wasPlaying = false;
function startStatusPolling() {
  if (statusPollTimer) return;
  statusPollTimer = setInterval(async () => {
    try {
      const res = await fetch("/api/scenarios/status");
      const data = await res.json();
      const runner = data.runner || data;
      renderRunnerStatus(runner);
      if (runner.is_playing) {
        refreshPreviewImage();
        refreshBevImage();
      }
      if (wasPlaying && !runner.is_playing) {
        fetchReport();
      }
      wasPlaying = !!runner.is_playing;

      // Gate on runner.is_playing too (not just running_frame_idx != null) as a
      // second line of defense: the backend response is snapshotted consistently,
      // but this keeps the frontend correct even if that ever regresses.
      const nextRunningIdx = runner.is_playing && runner.running_frame_idx != null ? runner.running_frame_idx : null;
      if (nextRunningIdx !== runningFrameIdx) {
        runningFrameIdx = nextRunningIdx;
        renderFrameList();
      }
    } catch (err) {
      // Backend not reachable yet; keep last known status on screen.
    }
  }, 1000);
}

// ---------- Event wiring ----------

function bindStaticEvents() {
  document.getElementById("scenario-name").addEventListener("change", (e) => {
    scenario.name = e.target.value || "untitled_scenario";
  });
  document.getElementById("canvas-width").addEventListener("change", (e) => {
    if (scenario.frames.some(f => f.objects.length > 0)) {
       alert("Warning: Changing canvas size with existing objects may cause them to be positioned incorrectly or out of bounds.");
    }
    scenario.canvas.width = clamp(parseInt(e.target.value, 10) || 640, 64, 4096);
    resizeCanvasElement();
    render();
  });
  document.getElementById("canvas-height").addEventListener("change", (e) => {
    if (scenario.frames.some(f => f.objects.length > 0)) {
       alert("Warning: Changing canvas size with existing objects may cause them to be positioned incorrectly or out of bounds.");
    }
    scenario.canvas.height = clamp(parseInt(e.target.value, 10) || 480, 64, 4096);
    resizeCanvasElement();
    render();
  });
  document.getElementById("calibration-source").addEventListener("change", (e) => {
    scenario.calibration.source = e.target.value || "config/calibration.json";
  });

  document.getElementById("btn-new-scenario").addEventListener("click", newScenario);
  document.getElementById("btn-export-scenario").addEventListener("click", exportScenarioToFile);
  document.getElementById("btn-import-scenario").addEventListener("click", () => {
    document.getElementById("import-file-input").click();
  });
  document.getElementById("import-file-input").addEventListener("change", (e) => {
    const file = e.target.files && e.target.files[0];
    if (file) importScenarioFromFile(file);
    e.target.value = "";
  });

  document.getElementById("route-intent-select").addEventListener("change", (e) => {
    scenario.route_intent.intent = e.target.value;
  });
  document.getElementById("route-intent-seq").addEventListener("change", (e) => {
    scenario.route_intent.seq = parseInt(e.target.value, 10) || 1;
  });
  
  document.getElementById("frame-route-intent-select").addEventListener("change", (e) => {
    if (scenario.frames.length === 0) return;
    const frame = scenario.frames[currentFrameIdx];
    const val = e.target.value;
    if (val) {
      if (!frame.route_intent) frame.route_intent = { seq: 1 };
      frame.route_intent.intent = val;
    } else {
      delete frame.route_intent;
    }
  });

  document.getElementById("class-select").addEventListener("change", (e) => {
    selectedClassName = e.target.value;
    syncShapeSelectWithClass();
  });

  document.getElementById("btn-draw-start").addEventListener("click", startDrawing);
  document.getElementById("btn-draw-finish").addEventListener("click", finalizeShape);
  document.getElementById("btn-draw-cancel").addEventListener("click", cancelDrawing);
  document.getElementById("btn-select-mode").addEventListener("click", cancelDrawing);
  document.getElementById("btn-delete-object").addEventListener("click", () => {
    if (!selectedObjectId) {
      alert("Chưa chọn object nào.");
      return;
    }
    deleteObject(selectedObjectId);
  });

  document.getElementById("btn-frame-add").addEventListener("click", duplicateCurrentFrame);
  document.getElementById("btn-frame-delete").addEventListener("click", deleteCurrentFrame);

  CANVAS_EL.addEventListener("click", onCanvasClick);
  CANVAS_EL.addEventListener("dblclick", onCanvasDblClick);
  CANVAS_EL.addEventListener("mousedown", onCanvasMouseDown);
  CANVAS_EL.addEventListener("mousemove", onCanvasMouseMove);
  CANVAS_EL.addEventListener("mouseup", onCanvasMouseUp);
  CANVAS_EL.addEventListener("mouseleave", onCanvasMouseUp);

  document.addEventListener("keydown", (evt) => {
    const activeTag = document.activeElement ? document.activeElement.tagName : "";
    const typing = activeTag === "INPUT" || activeTag === "SELECT" || activeTag === "TEXTAREA";
    if (evt.key === "Enter" && drawMode === "drawing" && !typing) {
      evt.preventDefault();
      finalizeShape();
    } else if (evt.key === "Escape" && drawMode === "drawing") {
      cancelDrawing();
    } else if ((evt.key === "Delete" || evt.key === "Backspace") && selectedObjectId && !typing) {
      evt.preventDefault();
      deleteObject(selectedObjectId);
    }
  });

  document.getElementById("btn-load-scenario").addEventListener("click", loadScenarioToBackend);
  document.getElementById("btn-step-ipm").addEventListener("click", () => postRunnerAction("/api/scenarios/step_ipm"));
  document.getElementById("btn-step").addEventListener("click", () => postRunnerAction("/api/scenarios/step"));
  document.getElementById("btn-play").addEventListener("click", () => postRunnerAction("/api/scenarios/play"));
  document.getElementById("btn-pause").addEventListener("click", () => postRunnerAction("/api/scenarios/pause"));
  document.getElementById("btn-stop").addEventListener("click", () => postRunnerAction("/api/scenarios/stop"));
  
  document.getElementById("btn-view-report").addEventListener("click", fetchReport);
  document.getElementById("btn-close-report").addEventListener("click", () => {
    document.getElementById("report-modal").style.display = "none";
  });

  document.getElementById("bev-show-slices").addEventListener("change", refreshBevImage);
  document.getElementById("bev-show-raw").addEventListener("change", refreshBevImage);
  document.getElementById("bev-show-filtered").addEventListener("change", refreshBevImage);
  document.getElementById("bev-show-candidate").addEventListener("change", refreshBevImage);
  document.getElementById("bev-show-normalized").addEventListener("change", refreshBevImage);
  document.getElementById("bev-show-committed").addEventListener("change", refreshBevImage);
}

async function init() {
  await loadLabelMapping();
  populateRouteIntentSelect();
  bindStaticEvents();
  syncScenarioPanelFromState();
  resizeCanvasElement();
  renderFrameList();
  renderObjectTable();
  render();
  startStatusPolling();
  refreshPreviewImage();
  refreshBevImage();
}

window.addEventListener("DOMContentLoaded", init);
