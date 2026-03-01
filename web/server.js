import http from "http";
import express from "express";
import { WebSocketServer } from "ws";

const app = express();
const port = Number(process.env.PORT || 8080);

app.use(express.static("public"));
app.get("/healthz", (_req, res) => res.json({ ok: true }));

const server = http.createServer(app);
const ingestWss = new WebSocketServer({ noServer: true });
const uiWss = new WebSocketServer({ noServer: true });

const state = {
  latest: null,
};

ingestWss.on("connection", (ws) => {
  ws.on("message", (raw) => {
    try {
      const data = JSON.parse(raw.toString());
      state.latest = data;
      const payload = JSON.stringify(data);
      for (const client of uiWss.clients) {
        if (client.readyState === 1) {
          client.send(payload);
        }
      }
    } catch (_e) {
      // ignore invalid message
    }
  });
});

uiWss.on("connection", (ws) => {
  if (state.latest) {
    ws.send(JSON.stringify(state.latest));
  }
});

server.on("upgrade", (req, socket, head) => {
  const url = req.url || "";
  if (url === "/ingest") {
    ingestWss.handleUpgrade(req, socket, head, (ws) => {
      ingestWss.emit("connection", ws, req);
    });
    return;
  }
  if (url === "/ws") {
    uiWss.handleUpgrade(req, socket, head, (ws) => {
      uiWss.emit("connection", ws, req);
    });
    return;
  }
  socket.destroy();
});

server.listen(port, () => {
  console.log(`web server listening on :${port}`);
});

