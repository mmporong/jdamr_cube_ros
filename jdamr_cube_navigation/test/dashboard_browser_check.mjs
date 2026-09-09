import fs from "node:fs";
const tabs = await (await fetch("http://127.0.0.1:9225/json")).json();
const ws = new WebSocket(
  tabs.find((t) => t.type === "page").webSocketDebuggerUrl,
);
await new Promise((r) => ws.addEventListener("open", r, { once: true }));
let id = 0;
const pending = new Map();
ws.addEventListener("message", (event) => {
  const m = JSON.parse(event.data);
  if (m.id) {
    const p = pending.get(m.id);
    pending.delete(m.id);
    m.error ? p.reject(m.error) : p.resolve(m.result);
  }
});
const send = (method, params = {}) =>
  new Promise((resolve, reject) => {
    const next = ++id;
    pending.set(next, { resolve, reject });
    ws.send(JSON.stringify({ id: next, method, params }));
  });
const evaluate = async (expression) => {
  const r = await send("Runtime.evaluate", {
    expression,
    awaitPromise: true,
    returnByValue: true,
  });
  if (r.exceptionDetails) throw Error(JSON.stringify(r.exceptionDetails));
  return r.result.value;
};
try {
  await send("Page.enable");
  await send("Emulation.setDeviceMetricsOverride", {
    width: 1536,
    height: 900,
    deviceScaleFactor: 1,
    mobile: false,
  });
  await send("Page.navigate", { url: "http://127.0.0.1:8765/" });
  await evaluate(
    `new Promise((resolve,reject)=>{let n=0;const t=setInterval(()=>{if(typeof replay!=='undefined'&&replay&&video.readyState>=2){clearInterval(t);resolve(true);}else if(++n>100){clearInterval(t);reject(Error('loading timeout'));}},100);})`,
  );
  const autoplay = await evaluate(
    "({enabled:video.autoplay,muted:video.muted,playing:!video.paused})",
  );
  if (!autoplay.enabled || !autoplay.muted || !autoplay.playing)
    throw Error("muted autoplay failed");
  const seek = async (time) =>
    evaluate(
      `new Promise(resolve=>{video.pause();video.addEventListener('seeked',()=>{update();resolve({time:video.currentTime,state:$('behavior').textContent,front:$('front').textContent,cmd:$('cmd').textContent});},{once:true});video.currentTime=${time};})`,
    );
  const moving = await evaluate(
    `replay.samples.find(s=>s.command?.linear_mps>0.1).time_s`,
  );
  const stopped = await evaluate(
    `replay.samples.find(s=>s.monitor?.action==='STOP'&&Math.abs(s.command?.linear_mps||0)<0.005).time_s`,
  );
  const a = await seek(moving + 0.01),
    b = await seek(stopped + 0.01);
  const layouts = [];
  for (const [width, height] of [
    [1366, 768],
    [1280, 720],
    [1920, 1080],
  ]) {
    await send("Emulation.setDeviceMetricsOverride", {
      width,
      height,
      deviceScaleFactor: 1,
      mobile: false,
    });
    const layout = await evaluate(`({
      width: innerWidth, height: innerHeight,
      pageWidth: document.documentElement.scrollWidth,
      pageHeight: document.documentElement.scrollHeight,
      removed: !document.querySelector('details') && !document.getElementById('live'),
      videoHeight: video.clientHeight,
      panelsFit: [...document.querySelectorAll('.state-panel,.flow-panel')]
        .every(e=>e.scrollHeight<=e.clientHeight+1)
    })`);
    if (
      !layout.removed ||
      !layout.panelsFit ||
      layout.videoHeight < 200 ||
      layout.pageWidth > width ||
      layout.pageHeight > height
    )
      throw Error("single-screen layout failed " + JSON.stringify(layout));
    layouts.push(layout);
  }
  await send("Emulation.setDeviceMetricsOverride", {
    width: 1536,
    height: 900,
    deviceScaleFactor: 1,
    mobile: false,
  });
  if (a.front === "미수신" || b.front === "미수신" || a.state === b.state)
    throw Error("replay state failed");
  await evaluate(`new Promise(r=>setTimeout(r,400))`);
  const paused = await evaluate(
    `({time:video.currentTime,state:$('behavior').textContent})`,
  );
  if (paused.time !== b.time || paused.state !== b.state)
    throw Error("pause sync failed");
  let shot = await send("Page.captureScreenshot", {
    format: "png",
    captureBeyondViewport: true,
  });
  fs.writeFileSync(
    "/tmp/jdamr_white_desktop.png",
    Buffer.from(shot.data, "base64"),
  );
  await send("Emulation.setDeviceMetricsOverride", {
    width: 390,
    height: 844,
    deviceScaleFactor: 1,
    mobile: true,
  });
  await evaluate("update()");
  const width = await evaluate(
    "({page:document.documentElement.scrollWidth,viewport:innerWidth})",
  );
  if (width.page > width.viewport)
    throw Error("mobile overflow " + JSON.stringify(width));
  shot = await send("Page.captureScreenshot", {
    format: "png",
    captureBeyondViewport: true,
  });
  fs.writeFileSync(
    "/tmp/jdamr_white_mobile.png",
    Buffer.from(shot.data, "base64"),
  );
  await evaluate(`(async()=>{
    const originalFetch=window.fetch, originalInterval=window.setInterval;
    try {
      window.setInterval=()=>0;
      window.fetch=async url=>url==='/api/config'
        ? {ok:true,json:async()=>({video_available:false,replay_available:false})}
        : originalFetch(url);
      await start();
    } finally { window.fetch=originalFetch; window.setInterval=originalInterval; }
  })()`);
  if ((await evaluate("mode")) !== "live")
    throw Error("no-replay source must automatically select live observation");
  const live = await evaluate(
    `({state:$('behavior').textContent,command:$('cmd').textContent})`,
  );
  if (live.command !== "미수신") throw Error("missing ROS misrepresented");
  const stale = await evaluate(
    `(async()=>{const original=window.fetch;try{window.fetch=async()=>({ok:true,json:async()=>({live:false,command:{linear_mps:0.18},pose:{linear_mps:0.18},navigation:{status:'SUCCEEDED'},collision_monitor:{action:'STOP'},freshness:{scan_age_s:30,odom_age_s:30,cmd_age_s:30}})});await poll();return {state:$('behavior').textContent,command:$('cmd').textContent,goal:$('goal').textContent};}finally{window.fetch=original;}})()`,
  );
  if (
    stale.command !== "미수신" ||
    stale.goal !== "미수신" ||
    stale.state !== "관측 대기"
  )
    throw Error("stale ROS misrepresented");
  const result = {
    status: "PASS",
    autoplay,
    layouts,
    moving: a,
    stopped: b,
    paused,
    mobile: width,
    live,
    stale,
  };
  fs.writeFileSync(
    "/tmp/jdamr_white_browser_verification.json",
    JSON.stringify(result, null, 2),
  );
  console.log(JSON.stringify(result, null, 2));
} finally {
  ws.close();
}
