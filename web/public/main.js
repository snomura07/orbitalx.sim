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
const lineBars = document.getElementById("lineBars");
const windowSecSelect = document.getElementById("windowSec");
const velocityCanvas = document.getElementById("velocityChart");
const ctx = velocityCanvas.getContext("2d");

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
  const data = history.filter((d) => d.ts >= startTs);

  ctx.clearRect(0, 0, velocityCanvas.width, velocityCanvas.height);
  ctx.fillStyle = "#f7fafc";
  ctx.fillRect(0, 0, velocityCanvas.width, velocityCanvas.height);

  if (data.length < 2) {
    return;
  }

  const velMax = Math.max(...data.map((d) => Math.max(d.velocity, d.desiredVelocity)), 1);
  const pad = 24;
  const w = velocityCanvas.width - pad * 2;
  const h = velocityCanvas.height - pad * 2;

  const xOf = (t) => pad + ((t - startTs) / windowSec) * w;
  const yOf = (v) => pad + h - (v / velMax) * h;

  ctx.strokeStyle = "#e5e7eb";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = pad + (h / 4) * i;
    ctx.beginPath();
    ctx.moveTo(pad, y);
    ctx.lineTo(pad + w, y);
    ctx.stroke();
  }

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

function render(data) {
  velocityValue.textContent = `${data.velocity.toFixed(1)} mm/s`;
  desiredVelocityValue.textContent = `${data.desiredVelocity.toFixed(1)} mm/s`;
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
}

windowSecSelect.addEventListener("change", () => {
  drawVelocityChart(Number(windowSecSelect.value));
});

ws.addEventListener("message", (ev) => {
  const msg = JSON.parse(ev.data);
  history.push(msg);
  const now = msg.ts;
  while (history.length && history[0].ts < now - maxHistorySec) {
    history.shift();
  }
  render(msg);
});

