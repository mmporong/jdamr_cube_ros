import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

const html = fs.readFileSync(new URL('../evaluation/navigation_dashboard.html', import.meta.url), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script);
const source = script.slice(script.indexOf('function speedSignal('), script.indexOf('function detailedEvent('));
const signal = vm.runInNewContext(`(${source})`, { finite: Number.isFinite });
const state = { command: { linear_mps: 0.18, angular_rps: 0 }, pose: { linear_mps: 0.18, angular_rps: 0 } };
const check = (s, kind, tone, label) => {
  const result = signal(s, kind);
  assert.equal(result[0], tone);
  assert.equal(result[1], label);
};
check(state, 'cmd', 'good', '주행 명령');
check(state, 'speed', 'good', '주행 관측');
check({ ...state, monitor: { action: 'SLOWDOWN' } }, 'speed', 'caution', '감속 요청 중');
check({ ...state, monitor: { action: 'STOP' } }, 'speed', 'caution', '정지 확인 중');
const stopped = { ...state, command: { linear_mps: 0, angular_rps: 0 }, pose: { linear_mps: 0, angular_rps: 0 } };
check({ ...stopped, monitor: { action: 'STOP' } }, 'cmd', 'stop', '충돌 감시 정지');
check({ ...stopped, monitor: { action: 'STOP' } }, 'speed', 'stop', '정지 관측');
check(stopped, 'cmd', 'neutral', '정지 명령');
check({ ...state, command: stopped.command }, 'speed', 'caution', '정지 확인 중');
check({ ...stopped, navigation: { status: 'SUCCEEDED' } }, 'speed', 'good', '도착 · 정지');
check({ ...state, observation_unavailable: true }, 'cmd', 'neutral', '미수신');
check({}, 'speed', 'neutral', '미수신');
assert.ok(!html.includes('id="mode-label"'));
assert.ok(!script.includes('$("mode-label")'));
assert.ok(!html.includes('id="connection"'));
assert.ok(!script.includes('$("connection")'));
assert.ok(html.includes('Gazebo recording'));
console.log('PASS: 11 speed signal cases, removed labels, English heading, JavaScript syntax');
const historySource = script.slice(script.indexOf('function speedHistorySamples('), script.indexOf('function speedHistoryValue('));
const history = vm.runInNewContext(`(${historySource})`, { finite: Number.isFinite });
const historyInput = [-1, 0, 4, 8, 9, NaN].map(time_s => ({ time_s }));
assert.equal(JSON.stringify(history(historyInput, 8)), JSON.stringify(historyInput.slice(1, 4)));
assert.equal(JSON.stringify(history(historyInput, 0)), JSON.stringify(historyInput.slice(0, 2)));
assert.equal(history([], 8).length, 0);
console.log('PASS: speed history window excludes future samples and supports backward seek');
const valueSource = script.slice(script.indexOf('function speedHistoryValue('), script.indexOf('function drawSpeedHistory('));
const historyValue = vm.runInNewContext(`(${valueSource})`, { finite: Number.isFinite });
assert.equal(historyValue({ ...state, ages_s: { cmd: 0.2, odom: 2 } }, 'command'), 0.18);
assert.equal(historyValue({ ...state, ages_s: { cmd: 0.2, odom: 2 } }, 'pose'), null);
assert.equal(historyValue({ ...state, ages_s: { cmd: 2 } }, 'command'), null);
assert.equal(historyValue(state, 'command'), null);
assert.equal(historyValue({ ...state, ages_s: { cmd: -1 } }, 'command'), null);
console.log('PASS: stale held values and unknown timestamps create history gaps');
const eventSource = script.slice(script.indexOf('function detailedEvent('), script.indexOf('function speedHistorySamples('));
const eventLog = vm.runInNewContext(`${eventSource}; buildEventLog`, {
  finite: Number.isFinite,
  number: (v, d = 3) => Number.isFinite(v) ? v.toFixed(d) : '미수신',
  observation: s => [s.monitor?.action || '대기', '상태 관측'],
});
const eventSamples = [0.18, 0.1, 0, 0.1].map((v, i) => ({
  time_s: i, command: { linear_mps: v, angular_rps: 0 }, ages_s: { cmd: 0, monitor: 0 },
  monitor: { action: i === 2 ? 'STOP' : 'DO_NOTHING' },
  map_pose: [i, 0, 0],
}));
const log = eventLog(eventSamples, [[1, 0]]);
assert.ok(log.some(e => e.label === '속도 감소 관측' && e.detail.includes('명령 속도 감소')));
assert.ok(log.every(e => !/미확인|미기록|미제공|미수신/.test(e.detail)));
assert.ok(log.some(e => e.label === '정지 후 이동 재개'));
assert.ok(log.some(e => e.label === '마스크 인근 진입'));
assert.ok(log.some(e => e.label === '마스크 인근 이탈'));
assert.ok(log.every(e => e.detail && e.time_s <= 3));
assert.ok(!eventLog(eventSamples.map(s => ({ ...s, ages_s: { cmd: 2 } }))).some(e => e.label === '속도 감소 관측'));
console.log('PASS: detailed event evidence, mask proximity, resume and stale-command filtering');
const turning = [
  { ...eventSamples[2], command: { linear_mps: 0, angular_rps: 0.2 } },
  eventSamples[3],
];
assert.ok(!eventLog(turning).some(e => e.label === '정지 후 이동 재개'));
assert.ok(eventLog([eventSamples[2], { ...eventSamples[3], command: { linear_mps: 0, angular_rps: 0.2 } }]).some(e => e.label === '정지 후 이동 재개'));
const playbackSource = script.slice(script.indexOf('function applyPlaybackRate()'), script.indexOf('$("playback-rate").addEventListener'));
const playbackVideo = { duration: 108.6 };
const playbackSelect = { value: '1' };
const applyRate = vm.runInNewContext(`(${playbackSource})`, { video: playbackVideo, $: () => playbackSelect, finite: Number.isFinite });
for (const [choice, rate] of [['0.5', 1.5], ['1', 3], ['2', 6], ['minute', 108.6 / 60]]) {
  playbackSelect.value = choice;
  applyRate();
  assert.equal(playbackVideo.playbackRate, rate);
  assert.equal(playbackVideo.defaultPlaybackRate, rate);
}
console.log('PASS: presentation speed baseline and one-minute playback');
