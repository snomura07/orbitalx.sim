const history = [];
const maxHistorySec = 120;

const velocityValue = document.getElementById("velocityValue");
const desiredVelocityValue = document.getElementById("desiredVelocityValue");
const omegaValue = document.getElementById("omegaValue");
const batteryValue = document.getElementById("batteryValue");
const lapCountValue = document.getElementById("lapCountValue");
const totalDistanceValue = document.getElementById("totalDistanceValue");
const paramsView = document.getElementById("paramsView");
const xhatValue = document.getElementById("xhatValue");
const lineDetected = document.getElementById("lineDetected");
const poseX = document.getElementById("poseX");
const poseY = document.getElementById("poseY");
const poseTheta = document.getElementById("poseTheta");
const xMinInput = document.getElementById("xMin");
const xMaxInput = document.getElementById("xMax");
const yMinInput = document.getElementById("yMin");
const yMaxInput = document.getElementById("yMax");
const lineBars = document.getElementById("lineBars");
const windowSecSelect = document.getElementById("windowSec");
const velocityCanvas = document.getElementById("velocityChart");
const ctx = velocityCanvas.getContext("2d");
const omegaCanvas = document.getElementById("omegaChart");
const omegaCtx = omegaCanvas.getContext("2d");
const batteryCanvas = document.getElementById("batteryChart");
const batteryCtx = batteryCanvas.getContext("2d");
const poseCanvas = document.getElementById("poseChart");
const poseCtx = poseCanvas.getContext("2d");
const odometryCanvas = document.getElementById("odometryChart");
const odometryCtx = odometryCanvas.getContext("2d");
const speedPlanCanvas = document.getElementById("speedPlanChart");
const speedPlanCtx = speedPlanCanvas.getContext("2d");
const velocityGaugeCanvas = document.getElementById("velocityGauge");
const velocityGaugeCtx = velocityGaugeCanvas.getContext("2d");
const omegaGaugeCanvas = document.getElementById("omegaGauge");
const omegaGaugeCtx = omegaGaugeCanvas.getContext("2d");
const vehicleSprite = new Image();
let vehicleSpriteReady = false;
vehicleSprite.onload = () => {
  vehicleSpriteReady = true;
};
vehicleSprite.src = "/orbitalx.png";

function resizeCanvasToDisplaySize(canvas, context) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const displayWidth = Math.max(1, Math.round(rect.width * dpr));
  const displayHeight = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== displayWidth || canvas.height !== displayHeight) {
    canvas.width = displayWidth;
    canvas.height = displayHeight;
  }
  context.setTransform(dpr, 0, 0, dpr, 0, 0);
}

const wsProto = location.protocol === "https:" ? "wss" : "ws";
const ws = new WebSocket(`${wsProto}://${location.host}/ws`);

for (let i = 0; i < 15; i += 1) {
  const bar = document.createElement("div");
  bar.className = "bar";
  bar.style.height = "2px";
  lineBars.appendChild(bar);
}

function drawCourseOutline(context, latest, xOf, yOf) {
  const polyline = Array.isArray(latest?.course?.polyline) ? latest.course.polyline : [];
  context.strokeStyle = "#94a3b8";
  context.lineWidth = 2;
  context.setLineDash([7, 5]);
  context.beginPath();

  if (polyline.length >= 2) {
    context.moveTo(xOf(Number(polyline[0].x)), yOf(Number(polyline[0].y)));
    for (let i = 1; i < polyline.length; i += 1) {
      context.lineTo(xOf(Number(polyline[i].x)), yOf(Number(polyline[i].y)));
    }
  } else {
    const straightLength = Number(latest?.course?.straightLength ?? 5000);
    const curveRadius = Number(latest?.course?.curveRadius ?? 300);
    const halfStraight = straightLength * 0.5;
    const arcSegments = 48;
    context.moveTo(xOf(-halfStraight), yOf(0));
    context.lineTo(xOf(halfStraight), yOf(0));
    for (let i = 0; i <= arcSegments; i += 1) {
      const t = -Math.PI / 2 + (Math.PI * i) / arcSegments;
      context.lineTo(
        xOf(halfStraight + curveRadius * Math.cos(t)),
        yOf(curveRadius + curveRadius * Math.sin(t)),
      );
    }
    context.lineTo(xOf(-halfStraight), yOf(2 * curveRadius));
    for (let i = 0; i <= arcSegments; i += 1) {
      const t = Math.PI / 2 + (Math.PI * i) / arcSegments;
      context.lineTo(
        xOf(-halfStraight + curveRadius * Math.cos(t)),
        yOf(curveRadius + curveRadius * Math.sin(t)),
      );
    }
    context.closePath();
  }

  context.stroke();
  context.setLineDash([]);
}

function drawVelocityChart(windowSec) {
  const now = history.length ? history[history.length - 1].ts : 0;
  const startTs = now - windowSec;
  const data = history.filter((d) => d.ts >= startTs && d.ts <= now);

  const canvasW = velocityCanvas.width;
  const canvasH = velocityCanvas.height;
  ctx.clearRect(0, 0, canvasW, canvasH);
  ctx.fillStyle = "#f7fafc";
  ctx.fillRect(0, 0, canvasW, canvasH);

  if (data.length < 2) {
    return;
  }

  const velMax = Math.max(...data.map((d) => Math.max(d.velocity, d.desiredVelocity)), 1);
  const pad = { left: 56, right: 16, top: 16, bottom: 34 };
  const w = canvasW - pad.left - pad.right;
  const h = canvasH - pad.top - pad.bottom;

  const xOf = (t) => pad.left + ((t - startTs) / windowSec) * w;
  const yOf = (v) => pad.top + h - (v / velMax) * h;

  ctx.font = "11px IBM Plex Sans, sans-serif";
  ctx.fillStyle = "#6b7280";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  ctx.strokeStyle = "#e5e7eb";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = pad.top + (h / 4) * i;
    const tickValue = ((velMax * (4 - i)) / 4).toFixed(0);
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(pad.left + w, y);
    ctx.stroke();
    ctx.fillText(tickValue, pad.left - 8, y);
  }

  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  ctx.fillText(startTs.toFixed(1), pad.left, pad.top + h + 6);
  ctx.fillText(now.toFixed(1), pad.left + w, pad.top + h + 6);
  ctx.fillText("Time [s]", pad.left + w * 0.5, canvasH - 14);

  ctx.save();
  ctx.translate(16, pad.top + h * 0.5);
  ctx.rotate(-Math.PI / 2);
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  ctx.fillText("Velocity [mm/s]", 0, 0);
  ctx.restore();

  ctx.lineWidth = 2;
  ctx.strokeStyle = "#0f766e";
  ctx.beginPath();
  data.forEach((d, i) => {
    const x = xOf(d.ts);
    const y = yOf(d.velocity);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  ctx.strokeStyle = "#f97316";
  ctx.beginPath();
  data.forEach((d, i) => {
    const x = xOf(d.ts);
    const y = yOf(d.desiredVelocity);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

function drawPoseChart(windowSec) {
  resizeCanvasToDisplaySize(poseCanvas, poseCtx);
  const now = history.length ? history[history.length - 1].ts : 0;
  const startTs = now - windowSec;
  const data = history.filter((d) => d.ts >= startTs && d.ts <= now);

  const canvasW = poseCanvas.clientWidth;
  const canvasH = poseCanvas.clientHeight;
  poseCtx.clearRect(0, 0, canvasW, canvasH);
  poseCtx.fillStyle = "#f7fafc";
  poseCtx.fillRect(0, 0, canvasW, canvasH);

  if (data.length < 2) {
    return;
  }

  const xMin = Number(xMinInput.value);
  const xMax = Number(xMaxInput.value);
  const yMin = Number(yMinInput.value);
  const yMax = Number(yMaxInput.value);
  if (!(xMax > xMin) || !(yMax > yMin)) {
    return;
  }

  const pad = { left: 44, right: 22, top: 16, bottom: 34 };
  const w = canvasW - pad.left - pad.right;
  const h = canvasH - pad.top - pad.bottom;
  const xOf = (x) => pad.left + ((x - xMin) / (xMax - xMin)) * w;
  const yOf = (y) => pad.top + h - ((y - yMin) / (yMax - yMin)) * h;
  const pxPerMmX = w / (xMax - xMin);
  const pxPerMmY = h / (yMax - yMin);
  const pxPerMm = Math.min(pxPerMmX, pxPerMmY);

  poseCtx.font = "11px IBM Plex Sans, sans-serif";
  poseCtx.fillStyle = "#6b7280";
  poseCtx.strokeStyle = "#e5e7eb";
  poseCtx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const x = pad.left + (w / 4) * i;
    const y = pad.top + (h / 4) * i;
    poseCtx.beginPath();
    poseCtx.moveTo(x, pad.top);
    poseCtx.lineTo(x, pad.top + h);
    poseCtx.stroke();
    poseCtx.beginPath();
    poseCtx.moveTo(pad.left, y);
    poseCtx.lineTo(pad.left + w, y);
    poseCtx.stroke();

    poseCtx.textAlign = "center";
    poseCtx.textBaseline = "top";
    const xTick = xMin + ((xMax - xMin) * i) / 4;
    poseCtx.fillText(xTick.toFixed(0), x, pad.top + h + 6);

    poseCtx.textAlign = "right";
    poseCtx.textBaseline = "middle";
    const yTick = yMax - ((yMax - yMin) * i) / 4;
    poseCtx.fillText(yTick.toFixed(0), pad.left - 8, y);
  }

  poseCtx.textAlign = "center";
  poseCtx.textBaseline = "top";
  poseCtx.fillText("X [mm]", pad.left + w * 0.5, canvasH - 14);
  poseCtx.save();
  poseCtx.translate(14, pad.top + h * 0.5);
  poseCtx.rotate(-Math.PI / 2);
  poseCtx.textAlign = "center";
  poseCtx.textBaseline = "top";
  poseCtx.fillText("Y [mm]", 0, 0);
  poseCtx.restore();

  const latest = data[data.length - 1];
  drawCourseOutline(poseCtx, latest, xOf, yOf);

  poseCtx.strokeStyle = "#155e75";
  poseCtx.lineWidth = 2;
  poseCtx.beginPath();
  data.forEach((d, i) => {
    const x = xOf(d.pose.x);
    const y = yOf(d.pose.y);
    if (i === 0) poseCtx.moveTo(x, y);
    else poseCtx.lineTo(x, y);
  });
  poseCtx.stroke();

  if (data.some((d) => d.poseSensor)) {
    poseCtx.strokeStyle = "#f97316";
    poseCtx.lineWidth = 1.6;
    poseCtx.beginPath();
    data.forEach((d, i) => {
      const sx = Number(d.poseSensor?.x ?? d.pose.x);
      const sy = Number(d.poseSensor?.y ?? d.pose.y);
      const x = xOf(sx);
      const y = yOf(sy);
      if (i === 0) poseCtx.moveTo(x, y);
      else poseCtx.lineTo(x, y);
    });
    poseCtx.stroke();
  }

  const last = data[data.length - 1];
  poseCtx.fillStyle = "#f97316";
  poseCtx.beginPath();
  poseCtx.arc(xOf(last.pose.x), yOf(last.pose.y), 4, 0, Math.PI * 2);
  poseCtx.fill();

  if (last.poseSensor) {
    poseCtx.fillStyle = "#0ea5e9";
    poseCtx.beginPath();
    poseCtx.arc(xOf(last.poseSensor.x), yOf(last.poseSensor.y), 3.5, 0, Math.PI * 2);
    poseCtx.fill();
  }

  if (vehicleSpriteReady && last?.pose) {
    const sensorBaseMm = Number(last?.params?.sensor_base_mm ?? 115);
    const wheelTreadMm = Number(last?.params?.wheel_tread_mm ?? 98);
    const vehicleLengthMm = sensorBaseMm + 70;
    const vehicleWidthMm = wheelTreadMm + 40;
    const displayScale = 2.8;
    const spriteW = Math.max(vehicleLengthMm * pxPerMm * displayScale, 42);
    const spriteH = Math.max(vehicleWidthMm * pxPerMm * displayScale, 22);
    const cx = xOf(last.pose.x);
    const cy = yOf(last.pose.y);
    const headingRad = -Number(last.pose.theta ?? 0) + Math.PI;
    poseCtx.save();
    poseCtx.translate(cx, cy);
    poseCtx.rotate(headingRad);
    poseCtx.globalAlpha = 0.95;
    poseCtx.drawImage(vehicleSprite, -spriteW * 0.5, -spriteH * 0.5, spriteW, spriteH);
    poseCtx.globalAlpha = 1.0;
    poseCtx.restore();
  }
}

function drawOdometryChart() {
  resizeCanvasToDisplaySize(odometryCanvas, odometryCtx);
  const latest = history.length ? history[history.length - 1] : null;
  const canvasW = odometryCanvas.clientWidth;
  const canvasH = odometryCanvas.clientHeight;
  odometryCtx.clearRect(0, 0, canvasW, canvasH);
  odometryCtx.fillStyle = "#f7fafc";
  odometryCtx.fillRect(0, 0, canvasW, canvasH);

  if (!latest) {
    return;
  }

  const xMin = Number(xMinInput.value);
  const xMax = Number(xMaxInput.value);
  const yMin = Number(yMinInput.value);
  const yMax = Number(yMaxInput.value);
  if (!(xMax > xMin) || !(yMax > yMin)) {
    return;
  }

  const pad = { left: 44, right: 22, top: 16, bottom: 34 };
  const w = canvasW - pad.left - pad.right;
  const h = canvasH - pad.top - pad.bottom;
  const xOf = (x) => pad.left + ((x - xMin) / (xMax - xMin)) * w;
  const yOf = (y) => pad.top + h - ((y - yMin) / (yMax - yMin)) * h;

  odometryCtx.font = "11px IBM Plex Sans, sans-serif";
  odometryCtx.fillStyle = "#6b7280";
  odometryCtx.strokeStyle = "#e5e7eb";
  odometryCtx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const x = pad.left + (w / 4) * i;
    const y = pad.top + (h / 4) * i;
    odometryCtx.beginPath();
    odometryCtx.moveTo(x, pad.top);
    odometryCtx.lineTo(x, pad.top + h);
    odometryCtx.stroke();
    odometryCtx.beginPath();
    odometryCtx.moveTo(pad.left, y);
    odometryCtx.lineTo(pad.left + w, y);
    odometryCtx.stroke();

    odometryCtx.textAlign = "center";
    odometryCtx.textBaseline = "top";
    const xTick = xMin + ((xMax - xMin) * i) / 4;
    odometryCtx.fillText(xTick.toFixed(0), x, pad.top + h + 6);

    odometryCtx.textAlign = "right";
    odometryCtx.textBaseline = "middle";
    const yTick = yMax - ((yMax - yMin) * i) / 4;
    odometryCtx.fillText(yTick.toFixed(0), pad.left - 8, y);
  }

  odometryCtx.textAlign = "center";
  odometryCtx.textBaseline = "top";
  odometryCtx.fillText("X [mm]", pad.left + w * 0.5, canvasH - 14);
  odometryCtx.save();
  odometryCtx.translate(14, pad.top + h * 0.5);
  odometryCtx.rotate(-Math.PI / 2);
  odometryCtx.textAlign = "center";
  odometryCtx.textBaseline = "top";
  odometryCtx.fillText("Y [mm]", 0, 0);
  odometryCtx.restore();

  drawCourseOutline(odometryCtx, latest, xOf, yOf);

  const trace = Array.isArray(latest.odometryTracePoints) ? latest.odometryTracePoints : [];
  if (trace.length < 2) {
    odometryCtx.fillStyle = "#64748b";
    odometryCtx.textAlign = "left";
    odometryCtx.textBaseline = "top";
    odometryCtx.fillText("Run with --stg to collect odometry trace.", pad.left + 8, pad.top + 8);
    return;
  }
  const segState = Array.isArray(latest.odometrySegmentTypes) ? latest.odometrySegmentTypes : [];
  odometryCtx.lineWidth = 2;
  for (let i = 0; i < trace.length - 1; i += 1) {
    const p0 = trace[i];
    const p1 = trace[i + 1];
    odometryCtx.strokeStyle = Number(segState[i] ?? 0) !== 0 ? "#dc2626" : "#16a34a";
    odometryCtx.beginPath();
    odometryCtx.moveTo(xOf(Number(p0.x)), yOf(Number(p0.y)));
    odometryCtx.lineTo(xOf(Number(p1.x)), yOf(Number(p1.y)));
    odometryCtx.stroke();
  }

  odometryCtx.font = "11px IBM Plex Sans, sans-serif";
  odometryCtx.textAlign = "left";
  odometryCtx.textBaseline = "top";
  odometryCtx.fillStyle = "#16a34a";
  odometryCtx.fillText("Straight", pad.left + 8, pad.top + 6);
  odometryCtx.fillStyle = "#dc2626";
  odometryCtx.fillText("Curve", pad.left + 70, pad.top + 6);

  const last = trace[trace.length - 1];
  odometryCtx.fillStyle = "#f97316";
  odometryCtx.beginPath();
  odometryCtx.arc(xOf(Number(last.x)), yOf(Number(last.y)), 4, 0, Math.PI * 2);
  odometryCtx.fill();
}

function drawSpeedPlanChart() {
  resizeCanvasToDisplaySize(speedPlanCanvas, speedPlanCtx);
  const latest = history.length ? history[history.length - 1] : null;
  const canvasW = speedPlanCanvas.clientWidth;
  const canvasH = speedPlanCanvas.clientHeight;
  speedPlanCtx.clearRect(0, 0, canvasW, canvasH);
  speedPlanCtx.fillStyle = "#f7fafc";
  speedPlanCtx.fillRect(0, 0, canvasW, canvasH);

  const dist = Array.isArray(latest?.speedPlanDistanceMm) ? latest.speedPlanDistanceMm : [];
  const vel = Array.isArray(latest?.speedPlanVelocityMmS) ? latest.speedPlanVelocityMmS : [];
  if (dist.length < 2 || vel.length < 2 || dist.length !== vel.length) {
    speedPlanCtx.font = "12px IBM Plex Sans, sans-serif";
    speedPlanCtx.fillStyle = "#64748b";
    speedPlanCtx.textAlign = "left";
    speedPlanCtx.textBaseline = "top";
    speedPlanCtx.fillText("Speed plan will appear after first lap.", 14, 14);
    return;
  }

  const xMax = Math.max(Number(dist[dist.length - 1]), 1);
  const yMax = Math.max(
    ...vel.map((v) => Number(v)),
    Number(latest?.params?.vehicle_max_velocity_mm_s ?? 1),
    1,
  ) * 1.08;
  const pad = { left: 56, right: 16, top: 16, bottom: 34 };
  const w = canvasW - pad.left - pad.right;
  const h = canvasH - pad.top - pad.bottom;
  const xOf = (s) => pad.left + (Number(s) / xMax) * w;
  const yOf = (v) => pad.top + h - (Number(v) / yMax) * h;

  speedPlanCtx.font = "11px IBM Plex Sans, sans-serif";
  speedPlanCtx.fillStyle = "#6b7280";
  speedPlanCtx.textAlign = "right";
  speedPlanCtx.textBaseline = "middle";
  speedPlanCtx.strokeStyle = "#e5e7eb";
  speedPlanCtx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = pad.top + (h / 4) * i;
    const tick = yMax - ((yMax * i) / 4);
    speedPlanCtx.beginPath();
    speedPlanCtx.moveTo(pad.left, y);
    speedPlanCtx.lineTo(pad.left + w, y);
    speedPlanCtx.stroke();
    speedPlanCtx.fillText(tick.toFixed(0), pad.left - 8, y);
  }

  speedPlanCtx.textAlign = "center";
  speedPlanCtx.textBaseline = "top";
  speedPlanCtx.fillText("0", pad.left, pad.top + h + 6);
  speedPlanCtx.fillText(xMax.toFixed(0), pad.left + w, pad.top + h + 6);
  speedPlanCtx.fillText("Distance [mm]", pad.left + w * 0.5, canvasH - 14);

  speedPlanCtx.save();
  speedPlanCtx.translate(16, pad.top + h * 0.5);
  speedPlanCtx.rotate(-Math.PI / 2);
  speedPlanCtx.textAlign = "center";
  speedPlanCtx.textBaseline = "top";
  speedPlanCtx.fillText("Velocity [mm/s]", 0, 0);
  speedPlanCtx.restore();

  speedPlanCtx.lineWidth = 2;
  speedPlanCtx.strokeStyle = "#0f766e";
  speedPlanCtx.beginPath();
  for (let i = 0; i < dist.length; i += 1) {
    const x = xOf(dist[i]);
    const y = yOf(vel[i]);
    if (i === 0) speedPlanCtx.moveTo(x, y);
    else speedPlanCtx.lineTo(x, y);
  }
  speedPlanCtx.stroke();
}

function drawArcSegment(context, cx, cy, radius, lineWidth, start, end, gradient) {
  context.beginPath();
  context.lineWidth = lineWidth;
  context.lineCap = "round";
  context.strokeStyle = gradient;
  context.arc(cx, cy, radius, start, end, end < start);
  context.stroke();
}

function drawCircularGauge(context, canvas, config) {
  resizeCanvasToDisplaySize(canvas, context);
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  context.clearRect(0, 0, width, height);

  const cx = width * 0.5;
  const cy = height * 0.66;
  const radius = Math.max(34, Math.min(width * 0.38, height * 0.46));
  const lineWidth = Math.max(10, radius * 0.2);
  const startAngle = Math.PI * 0.75;
  const sweep = Math.PI * 1.5;
  const endAngle = startAngle + sweep;
  const range = Math.max(config.max - config.min, 1e-6);
  const toAngle = (v) => startAngle + ((v - config.min) / range) * sweep;
  const clamped = Math.max(config.min, Math.min(config.max, config.value));

  context.fillStyle = "#f8fafc";
  context.fillRect(0, 0, width, height);

  context.beginPath();
  context.lineWidth = lineWidth;
  context.lineCap = "round";
  context.strokeStyle = "#dbe6ef";
  context.arc(cx, cy, radius, startAngle, endAngle, false);
  context.stroke();

  if (config.min < 0 && config.max > 0) {
    const zero = toAngle(0);
    const positiveGradient = context.createLinearGradient(cx - radius, cy, cx + radius, cy);
    positiveGradient.addColorStop(0.0, "#3b82f6");
    positiveGradient.addColorStop(0.55, "#22d3ee");
    positiveGradient.addColorStop(0.8, "#f59e0b");
    positiveGradient.addColorStop(1.0, "#ef4444");
    const negativeGradient = context.createLinearGradient(cx - radius, cy, cx + radius, cy);
    negativeGradient.addColorStop(0.0, "#2563eb");
    negativeGradient.addColorStop(0.45, "#38bdf8");
    negativeGradient.addColorStop(1.0, "#93c5fd");
    if (clamped >= 0) {
      drawArcSegment(context, cx, cy, radius, lineWidth, zero, toAngle(clamped), positiveGradient);
    } else {
      drawArcSegment(context, cx, cy, radius, lineWidth, zero, toAngle(clamped), negativeGradient);
    }
    context.beginPath();
    context.strokeStyle = "#64748b";
    context.lineWidth = 2;
    const zx = cx + Math.cos(zero) * (radius + lineWidth * 0.7);
    const zy = cy + Math.sin(zero) * (radius + lineWidth * 0.7);
    const zix = cx + Math.cos(zero) * (radius - lineWidth * 0.8);
    const ziy = cy + Math.sin(zero) * (radius - lineWidth * 0.8);
    context.moveTo(zix, ziy);
    context.lineTo(zx, zy);
    context.stroke();
  } else {
    const activeGradient = context.createLinearGradient(cx - radius, cy, cx + radius, cy);
    activeGradient.addColorStop(0.0, "#2563eb");
    activeGradient.addColorStop(0.55, "#22d3ee");
    activeGradient.addColorStop(0.78, "#f59e0b");
    activeGradient.addColorStop(1.0, "#ef4444");
    drawArcSegment(context, cx, cy, radius, lineWidth, startAngle, toAngle(clamped), activeGradient);
  }

  if (Number.isFinite(config.reference)) {
    const ref = Math.max(config.min, Math.min(config.max, Number(config.reference)));
    const refAngle = toAngle(ref);
    context.beginPath();
    context.strokeStyle = "#0f172a";
    context.lineWidth = 2;
    const ox = cx + Math.cos(refAngle) * (radius + lineWidth * 0.65);
    const oy = cy + Math.sin(refAngle) * (radius + lineWidth * 0.65);
    const ix = cx + Math.cos(refAngle) * (radius - lineWidth * 0.9);
    const iy = cy + Math.sin(refAngle) * (radius - lineWidth * 0.9);
    context.moveTo(ix, iy);
    context.lineTo(ox, oy);
    context.stroke();
  }

  context.font = "12px IBM Plex Sans, sans-serif";
  context.fillStyle = "#64748b";
  context.textAlign = "center";
  context.textBaseline = "bottom";
  context.fillText(config.label, cx, cy - radius - lineWidth * 1.1);

  context.font = "700 24px IBM Plex Sans, sans-serif";
  context.fillStyle = "#0f172a";
  context.textBaseline = "middle";
  context.fillText(config.valueText, cx, cy - lineWidth * 0.2);

  context.font = "12px IBM Plex Sans, sans-serif";
  context.fillStyle = "#64748b";
  context.textBaseline = "top";
  context.fillText(config.unitText, cx, cy + lineWidth * 0.5);

  context.font = "11px IBM Plex Sans, sans-serif";
  context.fillStyle = "#64748b";
  context.textBaseline = "middle";
  context.textAlign = "left";
  context.fillText(config.minText, cx - radius, cy + 6);
  context.textAlign = "right";
  context.fillText(config.maxText, cx + radius, cy + 6);
}

function drawVelocityGauge(sample) {
  const maxVelocityMmS = Math.max(1, Number(sample?.params?.vehicle_max_velocity_mm_s ?? 1));
  const value = Math.abs(Number(sample?.velocity ?? 0));
  const desired = Math.abs(Number(sample?.desiredVelocity ?? 0));
  drawCircularGauge(velocityGaugeCtx, velocityGaugeCanvas, {
    label: "Velocity",
    unitText: "mm/s",
    value,
    valueText: value.toFixed(0),
    min: 0,
    max: maxVelocityMmS,
    minText: "0",
    maxText: `${maxVelocityMmS.toFixed(0)}`,
    reference: desired,
  });
}

function drawOmegaGauge(sample) {
  const maxVelocityMmS = Math.max(1, Number(sample?.params?.vehicle_max_velocity_mm_s ?? 1));
  const wheelTreadMm = Number(sample?.params?.wheel_tread_mm ?? 0);
  const omegaFromModel =
    wheelTreadMm > 1 ? (2 * (maxVelocityMmS / 1000.0)) / (wheelTreadMm / 1000.0) : 0;
  const observed = Math.abs(Number(sample?.omega ?? 0));
  const omegaAbsMax = Math.max(0.5, omegaFromModel, observed * 1.2);
  const value = Number(sample?.omega ?? 0);
  const ref = Number(sample?.omegaRef ?? 0);

  resizeCanvasToDisplaySize(omegaGaugeCanvas, omegaGaugeCtx);
  const context = omegaGaugeCtx;
  const width = omegaGaugeCanvas.clientWidth;
  const height = omegaGaugeCanvas.clientHeight;
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#f8fafc";
  context.fillRect(0, 0, width, height);

  const cx = width * 0.5;
  const cy = height * 0.8;
  const radius = Math.max(32, Math.min(width * 0.36, height * 0.5));
  const lineWidth = Math.max(10, radius * 0.2);
  const leftAngle = Math.PI;
  const zeroAngle = Math.PI * 1.5;
  const rightAngle = Math.PI * 2.0;
  const clampMag = (v) => Math.max(0, Math.min(1, Math.abs(v) / omegaAbsMax));
  const toAngle = (v) => {
    const ratio = Math.max(-1, Math.min(1, v / omegaAbsMax));
    return zeroAngle + ratio * (Math.PI * 0.5);
  };

  context.beginPath();
  context.lineWidth = lineWidth;
  context.lineCap = "round";
  context.strokeStyle = "#dbe6ef";
  context.arc(cx, cy, radius, leftAngle, rightAngle, false);
  context.stroke();

  if (value < -1e-4) {
    const grad = context.createLinearGradient(cx - radius, cy, cx, cy);
    grad.addColorStop(0, "#1d4ed8");
    grad.addColorStop(1, "#38bdf8");
    drawArcSegment(
      context,
      cx,
      cy,
      radius,
      lineWidth,
      zeroAngle,
      zeroAngle - (Math.PI * 0.5) * clampMag(value),
      grad,
    );
  } else if (value > 1e-4) {
    const grad = context.createLinearGradient(cx, cy, cx + radius, cy);
    grad.addColorStop(0, "#f59e0b");
    grad.addColorStop(1, "#ef4444");
    drawArcSegment(
      context,
      cx,
      cy,
      radius,
      lineWidth,
      zeroAngle,
      zeroAngle + (Math.PI * 0.5) * clampMag(value),
      grad,
    );
  }

  context.beginPath();
  context.strokeStyle = "#0f172a";
  context.lineWidth = 2;
  const zix = cx + Math.cos(zeroAngle) * (radius - lineWidth * 0.9);
  const ziy = cy + Math.sin(zeroAngle) * (radius - lineWidth * 0.9);
  const zox = cx + Math.cos(zeroAngle) * (radius + lineWidth * 0.7);
  const zoy = cy + Math.sin(zeroAngle) * (radius + lineWidth * 0.7);
  context.moveTo(zix, ziy);
  context.lineTo(zox, zoy);
  context.stroke();

  if (Number.isFinite(ref)) {
    const a = toAngle(ref);
    const ix = cx + Math.cos(a) * (radius - lineWidth * 0.95);
    const iy = cy + Math.sin(a) * (radius - lineWidth * 0.95);
    const ox = cx + Math.cos(a) * (radius + lineWidth * 0.72);
    const oy = cy + Math.sin(a) * (radius + lineWidth * 0.72);
    context.beginPath();
    context.strokeStyle = "#111827";
    context.lineWidth = 2;
    context.moveTo(ix, iy);
    context.lineTo(ox, oy);
    context.stroke();
  }

  context.font = "12px IBM Plex Sans, sans-serif";
  context.fillStyle = "#64748b";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText("Angular Velocity", cx, cy - radius - lineWidth * 1.0);

  context.font = "700 24px IBM Plex Sans, sans-serif";
  context.fillStyle = "#0f172a";
  context.textBaseline = "middle";
  context.fillText(value.toFixed(2), cx, cy - radius * 0.35);

  context.font = "12px IBM Plex Sans, sans-serif";
  context.fillStyle = "#64748b";
  context.textBaseline = "middle";
  context.fillText("rad/s", cx, cy - radius * 0.35 + 20);

  context.font = "11px IBM Plex Sans, sans-serif";
  context.fillStyle = "#64748b";
  context.textBaseline = "middle";
  context.textAlign = "left";
  context.fillText(`-${omegaAbsMax.toFixed(1)}`, cx - radius, cy + 8);
  context.textAlign = "right";
  context.fillText(`+${omegaAbsMax.toFixed(1)}`, cx + radius, cy + 8);
  context.textAlign = "center";
  context.fillText("0", cx, cy - radius - 8);
}

function drawLiveGauges(sample = history.length ? history[history.length - 1] : null) {
  if (!sample) {
    return;
  }
  drawVelocityGauge(sample);
  drawOmegaGauge(sample);
}

function drawOmegaChart(windowSec) {
  const now = history.length ? history[history.length - 1].ts : 0;
  const startTs = now - windowSec;
  const data = history.filter((d) => d.ts >= startTs && d.ts <= now);

  const canvasW = omegaCanvas.width;
  const canvasH = omegaCanvas.height;
  omegaCtx.clearRect(0, 0, canvasW, canvasH);
  omegaCtx.fillStyle = "#f7fafc";
  omegaCtx.fillRect(0, 0, canvasW, canvasH);

  if (data.length < 2) {
    return;
  }

  const omegaAbsMax = Math.max(
    ...data.map((d) =>
      Math.max(
        Math.abs(Number(d.omega ?? 0)),
        Math.abs(Number(d.omegaRef ?? 0)),
      ),
    ),
    0.1,
  );
  const omegaRange = omegaAbsMax * 1.1;
  const pad = { left: 56, right: 16, top: 16, bottom: 34 };
  const w = canvasW - pad.left - pad.right;
  const h = canvasH - pad.top - pad.bottom;
  const xOf = (t) => pad.left + ((t - startTs) / windowSec) * w;
  const yOf = (v) => pad.top + h - ((v + omegaRange) / (2 * omegaRange)) * h;

  omegaCtx.font = "11px IBM Plex Sans, sans-serif";
  omegaCtx.fillStyle = "#6b7280";
  omegaCtx.textAlign = "right";
  omegaCtx.textBaseline = "middle";
  omegaCtx.strokeStyle = "#e5e7eb";
  omegaCtx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = pad.top + (h / 4) * i;
    const tick = omegaRange - ((2 * omegaRange * i) / 4);
    omegaCtx.beginPath();
    omegaCtx.moveTo(pad.left, y);
    omegaCtx.lineTo(pad.left + w, y);
    omegaCtx.stroke();
    omegaCtx.fillText(tick.toFixed(2), pad.left - 8, y);
  }

  omegaCtx.textAlign = "center";
  omegaCtx.textBaseline = "top";
  omegaCtx.fillText(startTs.toFixed(1), pad.left, pad.top + h + 6);
  omegaCtx.fillText(now.toFixed(1), pad.left + w, pad.top + h + 6);
  omegaCtx.fillText("Time [s]", pad.left + w * 0.5, canvasH - 14);

  omegaCtx.save();
  omegaCtx.translate(16, pad.top + h * 0.5);
  omegaCtx.rotate(-Math.PI / 2);
  omegaCtx.textAlign = "center";
  omegaCtx.textBaseline = "top";
  omegaCtx.fillText("Omega [rad/s]", 0, 0);
  omegaCtx.restore();

  omegaCtx.lineWidth = 2;
  omegaCtx.strokeStyle = "#7c3aed";
  omegaCtx.beginPath();
  data.forEach((d, i) => {
    const x = xOf(d.ts);
    const y = yOf(Number(d.omega ?? 0));
    if (i === 0) omegaCtx.moveTo(x, y);
    else omegaCtx.lineTo(x, y);
  });
  omegaCtx.stroke();

  omegaCtx.strokeStyle = "#f97316";
  omegaCtx.beginPath();
  data.forEach((d, i) => {
    const x = xOf(d.ts);
    const y = yOf(Number(d.omegaRef ?? 0));
    if (i === 0) omegaCtx.moveTo(x, y);
    else omegaCtx.lineTo(x, y);
  });
  omegaCtx.stroke();

  omegaCtx.font = "11px IBM Plex Sans, sans-serif";
  omegaCtx.textAlign = "left";
  omegaCtx.textBaseline = "top";
  omegaCtx.fillStyle = "#7c3aed";
  omegaCtx.fillText("actual", pad.left + 4, pad.top + 2);
  omegaCtx.fillStyle = "#f97316";
  omegaCtx.fillText("desired", pad.left + 52, pad.top + 2);
}

function drawBatteryChart(windowSec) {
  const now = history.length ? history[history.length - 1].ts : 0;
  const startTs = now - windowSec;
  const data = history.filter((d) => d.ts >= startTs && d.ts <= now);

  const canvasW = batteryCanvas.width;
  const canvasH = batteryCanvas.height;
  batteryCtx.clearRect(0, 0, canvasW, canvasH);
  batteryCtx.fillStyle = "#f7fafc";
  batteryCtx.fillRect(0, 0, canvasW, canvasH);

  if (data.length < 2) {
    return;
  }

  const yMin = 7.0;
  const yMax = 8.5;

  const pad = { left: 56, right: 16, top: 16, bottom: 34 };
  const w = canvasW - pad.left - pad.right;
  const h = canvasH - pad.top - pad.bottom;
  const xOf = (t) => pad.left + ((t - startTs) / windowSec) * w;
  const yOf = (v) => pad.top + h - ((v - yMin) / (yMax - yMin)) * h;

  batteryCtx.font = "11px IBM Plex Sans, sans-serif";
  batteryCtx.fillStyle = "#6b7280";
  batteryCtx.textAlign = "right";
  batteryCtx.textBaseline = "middle";
  batteryCtx.strokeStyle = "#e5e7eb";
  batteryCtx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = pad.top + (h / 4) * i;
    const tick = yMax - ((yMax - yMin) * i) / 4;
    batteryCtx.beginPath();
    batteryCtx.moveTo(pad.left, y);
    batteryCtx.lineTo(pad.left + w, y);
    batteryCtx.stroke();
    batteryCtx.fillText(tick.toFixed(2), pad.left - 8, y);
  }

  batteryCtx.textAlign = "center";
  batteryCtx.textBaseline = "top";
  batteryCtx.fillText(startTs.toFixed(1), pad.left, pad.top + h + 6);
  batteryCtx.fillText(now.toFixed(1), pad.left + w, pad.top + h + 6);
  batteryCtx.fillText("Time [s]", pad.left + w * 0.5, canvasH - 14);

  batteryCtx.save();
  batteryCtx.translate(16, pad.top + h * 0.5);
  batteryCtx.rotate(-Math.PI / 2);
  batteryCtx.textAlign = "center";
  batteryCtx.textBaseline = "top";
  batteryCtx.fillText("Battery [V]", 0, 0);
  batteryCtx.restore();

  batteryCtx.lineWidth = 2;
  batteryCtx.strokeStyle = "#0ea5e9";
  batteryCtx.beginPath();
  data.forEach((d, i) => {
    const x = xOf(d.ts);
    const y = yOf(Number(d.batteryV ?? 0));
    if (i === 0) batteryCtx.moveTo(x, y);
    else batteryCtx.lineTo(x, y);
  });
  batteryCtx.stroke();
}

function render(data) {
  velocityValue.textContent = `${data.velocity.toFixed(1)} mm/s`;
  const desiredInput = Number(data.desiredVelocityInput ?? data.desiredVelocity);
  const desiredApplied = Number(data.desiredVelocity);
  desiredVelocityValue.textContent =
    `${desiredInput.toFixed(1)} mm/s (applied ${desiredApplied.toFixed(1)})`;
  omegaValue.textContent = `${Number(data.omega ?? 0).toFixed(3)} rad/s`;
  batteryValue.textContent = `${Number(data.batteryV ?? 0).toFixed(2)} V`;
  lapCountValue.textContent = `${Math.trunc(Number(data.lapCount ?? 0))}`;
  totalDistanceValue.textContent = `${Number(data.totalDistanceMm ?? 0).toFixed(1)} mm`;
  if (data.params) {
    const ordered = Object.keys(data.params)
      .sort()
      .reduce((acc, k) => {
        acc[k] = data.params[k];
        return acc;
      }, {});
    paramsView.textContent = JSON.stringify(ordered, null, 2);
  }
  xhatValue.textContent = data.lineDetected ? `${data.xHat.toFixed(2)} mm` : "UNDETECTED";
  lineDetected.textContent = data.lineDetected ? "YES" : "NO";
  poseX.textContent = `${data.pose.x.toFixed(2)} mm`;
  poseY.textContent = `${data.pose.y.toFixed(2)} mm`;
  poseTheta.textContent = `${data.pose.theta.toFixed(3)} rad`;

  const bars = lineBars.querySelectorAll(".bar");
  data.lineValues.forEach((v, idx) => {
    const h = Math.max(2, Math.round(v * 86));
    bars[idx].style.height = `${h}px`;
  });

  drawVelocityChart(Number(windowSecSelect.value));
  drawOmegaChart(Number(windowSecSelect.value));
  drawBatteryChart(Number(windowSecSelect.value));
  drawPoseChart(Number(windowSecSelect.value));
  drawOdometryChart();
  drawSpeedPlanChart();
  drawLiveGauges(data);
}

windowSecSelect.addEventListener("change", () => {
  drawVelocityChart(Number(windowSecSelect.value));
  drawOmegaChart(Number(windowSecSelect.value));
  drawBatteryChart(Number(windowSecSelect.value));
  drawPoseChart(Number(windowSecSelect.value));
  drawOdometryChart();
  drawSpeedPlanChart();
  drawLiveGauges();
});

[xMinInput, xMaxInput, yMinInput, yMaxInput].forEach((el) => {
  el.addEventListener("change", () => {
    drawPoseChart(Number(windowSecSelect.value));
    drawOdometryChart();
  });
});

window.addEventListener("resize", () => {
  drawPoseChart(Number(windowSecSelect.value));
  drawOdometryChart();
  drawSpeedPlanChart();
  drawLiveGauges();
});

ws.addEventListener("message", (ev) => {
  const msg = JSON.parse(ev.data);
  const lastTs = history.length ? history[history.length - 1].ts : null;
  if (lastTs !== null && msg.ts < lastTs) {
    history.length = 0;
  }
  history.push(msg);
  const now = msg.ts;
  while (history.length && history[0].ts < now - maxHistorySec) {
    history.shift();
  }
  render(msg);
});

ws.addEventListener("open", () => {
  history.length = 0;
});
