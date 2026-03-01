const history = [];
const maxHistorySec = 120;

const velocityValue = document.getElementById("velocityValue");
const desiredVelocityValue = document.getElementById("desiredVelocityValue");
const batteryValue = document.getElementById("batteryValue");
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
const poseCanvas = document.getElementById("poseChart");
const poseCtx = poseCanvas.getContext("2d");

const wsProto = location.protocol === "https:" ? "wss" : "ws";
const ws = new WebSocket(`${wsProto}://${location.host}/ws`);

for (let i = 0; i < 15; i += 1) {
  const bar = document.createElement("div");
  bar.className = "bar";
  bar.style.height = "2px";
  lineBars.appendChild(bar);
}

function drawVelocityChart(windowSec) {
  const now = history.length ? history[history.length - 1].ts : 0;
  const startTs = now - windowSec;
  const data = history.filter((d) => d.ts >= startTs && d.ts <= now);

  ctx.clearRect(0, 0, velocityCanvas.width, velocityCanvas.height);
  ctx.fillStyle = "#f7fafc";
  ctx.fillRect(0, 0, velocityCanvas.width, velocityCanvas.height);

  if (data.length < 2) {
    return;
  }

  const velMax = Math.max(...data.map((d) => Math.max(d.velocity, d.desiredVelocity)), 1);
  const pad = { left: 56, right: 16, top: 16, bottom: 34 };
  const w = velocityCanvas.width - pad.left - pad.right;
  const h = velocityCanvas.height - pad.top - pad.bottom;

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
  ctx.fillText("Time [s]", pad.left + w * 0.5, velocityCanvas.height - 14);

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
  const now = history.length ? history[history.length - 1].ts : 0;
  const startTs = now - windowSec;
  const data = history.filter((d) => d.ts >= startTs && d.ts <= now);

  poseCtx.clearRect(0, 0, poseCanvas.width, poseCanvas.height);
  poseCtx.fillStyle = "#f7fafc";
  poseCtx.fillRect(0, 0, poseCanvas.width, poseCanvas.height);

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
  const w = poseCanvas.width - pad.left - pad.right;
  const h = poseCanvas.height - pad.top - pad.bottom;
  const xOf = (x) => pad.left + ((x - xMin) / (xMax - xMin)) * w;
  const yOf = (y) => pad.top + h - ((y - yMin) / (yMax - yMin)) * h;

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
  poseCtx.fillText("X [mm]", pad.left + w * 0.5, poseCanvas.height - 14);
  poseCtx.save();
  poseCtx.translate(14, pad.top + h * 0.5);
  poseCtx.rotate(-Math.PI / 2);
  poseCtx.textAlign = "center";
  poseCtx.textBaseline = "top";
  poseCtx.fillText("Y [mm]", 0, 0);
  poseCtx.restore();

  const latest = data[data.length - 1];
  const straightLength = Number(latest?.course?.straightLength ?? 5000);
  const curveRadius = Number(latest?.course?.curveRadius ?? 300);
  const halfStraight = straightLength * 0.5;

  poseCtx.strokeStyle = "#94a3b8";
  poseCtx.lineWidth = 2;
  poseCtx.setLineDash([7, 5]);
  poseCtx.beginPath();

  const arcSegments = 48;
  const moveToPoint = (x, y) => {
    poseCtx.moveTo(xOf(x), yOf(y));
  };
  const lineToPoint = (x, y) => {
    poseCtx.lineTo(xOf(x), yOf(y));
  };

  moveToPoint(-halfStraight, 0);
  lineToPoint(halfStraight, 0);
  for (let i = 0; i <= arcSegments; i += 1) {
    const t = -Math.PI / 2 + (Math.PI * i) / arcSegments;
    lineToPoint(halfStraight + curveRadius * Math.cos(t), curveRadius + curveRadius * Math.sin(t));
  }
  lineToPoint(-halfStraight, 2 * curveRadius);
  for (let i = 0; i <= arcSegments; i += 1) {
    const t = Math.PI / 2 + (Math.PI * i) / arcSegments;
    lineToPoint(-halfStraight + curveRadius * Math.cos(t), curveRadius + curveRadius * Math.sin(t));
  }

  poseCtx.closePath();
  poseCtx.stroke();
  poseCtx.setLineDash([]);

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

  const last = data[data.length - 1];
  poseCtx.fillStyle = "#f97316";
  poseCtx.beginPath();
  poseCtx.arc(xOf(last.pose.x), yOf(last.pose.y), 4, 0, Math.PI * 2);
  poseCtx.fill();
}

function render(data) {
  velocityValue.textContent = `${data.velocity.toFixed(1)} mm/s`;
  const desiredInput = Number(data.desiredVelocityInput ?? data.desiredVelocity);
  const desiredApplied = Number(data.desiredVelocity);
  desiredVelocityValue.textContent =
    `${desiredInput.toFixed(1)} mm/s (applied ${desiredApplied.toFixed(1)})`;
  batteryValue.textContent = `${data.batterySoc.toFixed(1)} %`;
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
  drawPoseChart(Number(windowSecSelect.value));
}

windowSecSelect.addEventListener("change", () => {
  drawVelocityChart(Number(windowSecSelect.value));
  drawPoseChart(Number(windowSecSelect.value));
});

[xMinInput, xMaxInput, yMinInput, yMaxInput].forEach((el) => {
  el.addEventListener("change", () => {
    drawPoseChart(Number(windowSecSelect.value));
  });
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
