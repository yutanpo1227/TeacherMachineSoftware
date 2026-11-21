const { app, BrowserWindow, ipcMain } = require('electron');
const { SerialPort } = require('serialport');
const { StringDecoder } = require('node:string_decoder');
const path = require('node:path');

// ElectronのメインプロセスがUSBポートを管理し、画面へは受信した1行だけを渡す。
let window;
let port = null;
function send(channel, data) {
  if (window && !window.isDestroyed()) window.webContents.send(channel, data);
}
async function disconnect() {
  const active = port;
  port = null;
  if (active && active.isOpen) await new Promise((resolve) => active.close(() => resolve()));
}
function authorized(event) { return window && event.sender === window.webContents; }
app.whenReady().then(() => {
  window = new BrowserWindow({ width: 1420, height: 950, minWidth: 980, minHeight: 700,
    backgroundColor: '#101d1a', title: 'PITCH / ロボットデバッガー',
    webPreferences: { preload: path.join(__dirname, 'preload.js'), contextIsolation: true, nodeIntegration: false, sandbox: true } });
  window.webContents.setWindowOpenHandler(() => ({ action: 'deny' }));
  window.webContents.on('will-navigate', event => event.preventDefault());
  window.loadFile('index.html');
  ipcMain.handle('serial:list', async event => {
    if (!authorized(event)) throw Error('操作できません');
    return (await SerialPort.list()).map(({ path, manufacturer, vendorId, productId }) => ({ path, manufacturer, vendorId, productId }));
  });
  ipcMain.handle('serial:disconnect', async event => {
    if (!authorized(event)) throw Error('操作できません');
    await disconnect();
  });
  ipcMain.handle('serial:connect', async (event, request) => {
    if (!authorized(event)) throw Error('操作できません');
    if (!request || typeof request.path !== 'string' || ![9600, 57600, 115200, 230400].includes(request.baudRate)) throw Error('接続設定が不正です');
    const found = (await SerialPort.list()).some(item => item.path === request.path);
    if (!found) throw Error('ポートが見つかりません。一覧を更新してください。');
    await disconnect();
    const current = new SerialPort({ path: request.path, baudRate: request.baudRate, autoOpen: false });
    port = current;
    // 受信データは行や日本語文字の途中で分割されるため、復元してから画面に渡す。
    const decoder = new StringDecoder('utf8');
    let pending = '';
    current.on('data', chunk => {
      pending += decoder.write(chunk);
      if (pending.length > 32768) pending = pending.slice(-8192);
      let i;
      while ((i = pending.indexOf('\n')) !== -1) {
        const line = pending.slice(0, i).replace(/\r$/, '');
        pending = pending.slice(i + 1);
        send('serial:line', line);
      }
    });
    current.on('error', error => send('serial:status', { connected: false, error: error.message }));
    current.on('close', () => {
      if (port === current) { port = null; send('serial:status', { connected: false, error: 'ポートが閉じられました' }); }
    });
    await new Promise((resolve, reject) => current.open(error => error ? reject(error) : resolve()));
    return { path: request.path };
  });
});
app.on('window-all-closed', () => { disconnect().finally(() => app.quit()); });
