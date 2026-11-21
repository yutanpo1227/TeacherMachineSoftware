const $ = id => document.getElementById(id);
const data = new Telemetry();
let connected = false, demoTimer = null, logCount = 0, lastLine = 0;
const angle = value => typeof value === 'number' && value >= 0 && value < 360 ? value : null;
const val = name => data.get(name);
const degrees = value => angle(value) === null ? '未検出' : `${Math.round(value)}°`;
// 機体の前方90度を画面の上に描く。半径は表示用で、実際の距離には換算しない。
const coords = (deg, radius) => ({ x: 350 + Math.cos(deg * Math.PI / 180) * radius, y: 305 - Math.sin(deg * Math.PI / 180) * radius });
const states = { GUARD: '守備', ATTACK: '攻撃', RETURN: '復帰', GOAL_MISSING: 'ゴール未検出', CAMERA_LOST: 'カメラ通信なし', OFFENSE: '攻撃', TEST: '姿勢テスト' };
function row(icon, color, bg, label, sub, value, muted = false) {
  return `<div class="sensor-row"><span class="sensor-icon" style="background:${bg};color:${color}">${icon}</span><div><div class="sensor-name">${label}</div><div class="sensor-sub">${sub}</div></div><span class="sensor-value ${muted ? 'muted' : ''}">${value}</span></div>`;
}
function draw() {
  const now = Date.now();
  const ball = angle(val('ball')), yellow = angle(val('yellow')), blue = angle(val('blue'));
  const rear = angle(val('ownRear')), line = angle(val('line')), move = angle(val('move'));
  const yaw = val('yaw'), error = val('error');
  let svg = `<defs><pattern id="grid" width="36" height="36" patternUnits="userSpaceOnUse"><path d="M36 0H0V36" fill="none" stroke="#edf2ea" stroke-width="1"/></pattern></defs><rect x="20" y="8" width="660" height="595" rx="13" fill="#f8faf6"/><rect x="20" y="8" width="660" height="595" fill="url(#grid)"/>`;
  svg += `<circle cx="350" cy="305" r="230" fill="#f4f8ef" stroke="#d8e6d6" stroke-width="1"/><circle cx="350" cy="305" r="170" fill="none" stroke="#d6e5d3" stroke-dasharray="4 6"/><circle cx="350" cy="305" r="110" fill="none" stroke="#d6e5d3" stroke-dasharray="4 6"/><path d="M120 305H580 M350 75V535" stroke="#d6e4d3" stroke-dasharray="3 7"/><text x="350" y="49" text-anchor="middle" fill="#6c8b70" font-size="12" font-weight="700">前方 90°</text><text x="350" y="576" text-anchor="middle" fill="#91a893" font-size="11">後方 270°</text><text x="96" y="309" text-anchor="end" fill="#91a893" font-size="11">左 180°</text><text x="604" y="309" fill="#91a893" font-size="11">右 0°</text>`;
  function ray(a, col, label, type) {
    if (a === null) return;
    const p = coords(a, type === 'line' ? 168 : 215);
    svg += `<line x1="350" y1="305" x2="${p.x}" y2="${p.y}" stroke="${col}" stroke-width="2" stroke-dasharray="5 6" opacity=".8"/>`;
    if (type === 'ball') svg += `<circle cx="${p.x}" cy="${p.y}" r="15" fill="#f39a5c" stroke="#d8753c" stroke-width="2"/><path d="M${p.x-9} ${p.y-3}q9 5 18 0 M${p.x-7} ${p.y+6}q7 -4 14 0" fill="none" stroke="#d7753b" stroke-width="1"/>`;
    else if (type === 'line') svg += `<circle cx="${p.x}" cy="${p.y}" r="15" fill="#fff" stroke="#a9bda8" stroke-width="2"/><path d="M${p.x-7} ${p.y}h14" stroke="#819985" stroke-width="3"/>`;
    else svg += `<rect x="${p.x-16}" y="${p.y-10}" width="32" height="20" rx="4" fill="${col}" stroke="#fff" stroke-width="2"/>`;
    const labelY = p.y < 140 ? p.y + 30 : p.y - 23;
    svg += `<text x="${p.x}" y="${labelY}" text-anchor="middle" fill="#5a715f" font-size="10" font-weight="700">${label} ${Math.round(a)}°</text>`;
  }
  ray(yellow, '#e3c357', '黄ゴール', 'goal'); ray(blue, '#78ace0', '青ゴール', 'goal');
  if (yellow === null && blue === null && rear !== null) ray(rear, '#e3c357', '自ゴール', 'goal');
  ray(ball, '#e98950', 'ボール', 'ball'); ray(line, '#a3b6a4', '白線', 'line');
  if (move !== null && val('power') > 0) { const p = coords(move, 95); svg += `<line x1="350" y1="305" x2="${p.x}" y2="${p.y}" stroke="#70a965" stroke-width="4"/>`; }
  svg += `<circle cx="350" cy="305" r="53" fill="#d8e9d1" opacity=".45"/><circle cx="350" cy="305" r="39" fill="#203b2c" stroke="#fff" stroke-width="5"/><path d="M350 270l-9 15h18z" fill="#c9ed90"/><circle cx="350" cy="306" r="19" fill="#385b43"/><text x="350" y="310" text-anchor="middle" fill="#e3f0de" font-size="9" font-weight="700">ROBOT</text><text x="350" y="366" text-anchor="middle" fill="#6a806b" font-size="10" font-weight="700">ロボット</text>`;
  $('radar').innerHTML = svg;
  const lineUsed = val('lineUsed');
  $('sensorList').innerHTML = [
    row('●','#d77942','#fff0e6','ボールの方向',`IR ${degrees(val('ir'))}　/　カメラ ${degrees(val('cameraBall'))}`,degrees(ball),ball===null),
    row('▰','#b69629','#faf4d9','黄色ゴール','カメラで検出した方向',degrees(yellow),yellow===null),
    row('▰','#5b98cf','#eaf4fb','青色ゴール','カメラで検出した方向',degrees(blue),blue===null),
    row('╱','#81998a','#eff3ee','白線の方向', lineUsed === 1 ? '制御で使用中' : lineUsed === 0 ? '観測のみ・守備制御には使わない' : '制御状態は未取得',degrees(line),line===null),
    row('⌖','#739f64','#edf6e9','機体の姿勢','ジャイロの向き / 起動時基準',yaw === null ? '未取得' : `${Math.round(yaw)}°`,yaw===null),
    row('↻','#749487','#eef5f1','姿勢のずれ','制御で使用する目標との差',error === null ? '未取得' : `${Math.round(error)}°`,error===null)
  ].join('');
  $('mode').textContent = states[val('state')] || (demoTimer ? 'デモ' : '待機中');
  $('lastUpdate').textContent = lastLine ? `${Math.floor((now-lastLine)/1000)} 秒前に受信` : 'データ待機中';
}
function receive(line) {
  if (logCount === 0) $('log').textContent = '';
  logCount++; $('logCount').textContent = `${logCount} 行`;
  const log = $('log'); log.textContent += line + '\n';
  if (log.textContent.length > 45000) log.textContent = log.textContent.slice(-35000);
  log.scrollTop = log.scrollHeight;
  if (data.parse(line)) lastLine = Date.now();
  draw();
}
function setStatus(value, message) {
  connected = value; $('statusDot').classList.toggle('on', value); $('status').textContent = value ? '接続中' : '未接続';
  $('connect').innerHTML = value ? '切断する <span>×</span>' : '接続する <span>↗</span>';
  $('message').textContent = message;
}
async function refresh() {
  try {
    const previous = $('ports').value;
    const ports = await window.serial.list();
    $('ports').replaceChildren(new Option('ポートを選択', ''), ...ports.map(p => new Option(`${p.path}${p.manufacturer ? ` — ${p.manufacturer}` : ''}`, p.path)));
    if (ports.some(p => p.path === previous)) $('ports').value = previous;
    else if (ports.length === 1) $('ports').value = ports[0].path;
    $('message').textContent = ports.length ? `${ports.length} 件のポートが見つかりました` : 'ポートが見つかりません。USB 接続を確認してください。';
  } catch (error) { $('message').textContent = `一覧取得エラー: ${error.message}`; }
}
$('refresh').onclick = refresh;
$('connect').onclick = async () => {
  if (demoTimer) stopDemo();
  if (connected) { await window.serial.disconnect(); data.reset(); setStatus(false, '切断しました'); draw(); return; }
  if (!$('ports').value) { $('message').textContent = 'ポートを選択してください。'; return; }
  $('connect').disabled = true;
  try { data.reset(); lastLine = 0; await window.serial.connect($('ports').value, Number($('baud').value)); setStatus(true, `${$('ports').value} に接続しました`); }
  catch (error) { setStatus(false, `接続エラー: ${error.message}`); }
  finally { $('connect').disabled = false; }
};
window.serial.onLine(receive);
window.serial.onStatus(status => { if (!status.connected) { data.reset(); setStatus(false, status.error || '切断しました'); draw(); } });
$('clear').onclick = () => { $('log').textContent = ''; logCount = 0; $('logCount').textContent = '0 行'; };
function stopDemo() { clearInterval(demoTimer); demoTimer = null; data.reset(); lastLine = 0; $('demo').textContent = 'デモ表示'; setStatus(false, 'デモを終了しました'); draw(); }
$('demo').onclick = async () => {
  if (demoTimer) { stopDemo(); return; }
  if (connected) await window.serial.disconnect();
  data.reset(); setStatus(false, 'サンプルデータを表示中。実機の値ではありません。'); $('status').textContent = 'デモ'; $('demo').textContent = 'デモを終了';
  let n = 0;
  const sample = () => { const ball = Math.round(90 + 55 * Math.sin(n++ / 8)); receive(`telemetry role=DEFENSE state=GUARD yaw=2 ball=${ball} ir=${ball} cameraBall=-1 line=215 lineStrength=75 yellow=268 blue=85 ownRear=268 camera=1 error=-2 bottomGap=65 lineUsed=0 attackYellow=0 lineEnabled=1`); };
  sample(); demoTimer = setInterval(sample, 250);
};
setInterval(draw, 500); draw(); refresh();
