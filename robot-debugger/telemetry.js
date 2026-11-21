// Arduino の既存の人間向けログを解析する。各値に独立した有効期限を設ける。
(function (root) {
  const labelMap = { 'ジャイロ角度': 'yaw', 'ライン角度': 'line', 'ラインベクトル大きさ': 'lineStrength', 'ボール角度': 'ball', 'ボール距離': 'ballDistance', 'カメラボール角度': 'cameraBall' };
  const allowed = new Set(['role', 'state', 'ownRear', 'span', 'ball', 'ir', 'cameraBall', 'yaw', 'error', 'homeOffset', 'lateralLimited', 'homeReady', 'line', 'lineStrength', 'lineUsed', 'lineEnabled', 'attackYellow', 'bottomGap', 'return', 'move', 'power', 'camera', 'source', 'yellow', 'blue', 'turn', 'headingValid', 'attackWaitMs']);
  class Telemetry {
    constructor() { this.fields = {}; }
    put(key, value, now) { this.fields[key] = { value, at: now }; }
    // 通信が途切れたとき、古い検出位置を現在の位置として表示し続けない。
    get(key, now = Date.now(), maxAge = 1800) { const v = this.fields[key]; return v && now - v.at <= maxAge ? v.value : null; }
    reset() { this.fields = {}; }
    parse(line, now = Date.now()) {
      // 日本語の旧ログと、現在の key=value 形式の両方を受け付ける。
      const labeled = line.match(/^([^:]+):\s*(-?\d+(?:\.\d+)?)\s*$/);
      if (labeled && labelMap[labeled[1]]) { this.put(labelMap[labeled[1]], Number(labeled[2]), now); return true; }
      if (!/^(telemetry|defense|heading-test)\s/.test(line)) return false;
      const pairs = [...line.matchAll(/(?:^|\s)([A-Za-z]+)=([^\s]+)/g)];
      for (const [, key, value] of pairs) {
        if (!allowed.has(key)) continue;
        const n = Number(value);
        this.put(key, value !== '' && Number.isFinite(n) ? n : value, now);
      }
      if (this.get('state', now) === 'GOAL_MISSING' && line.startsWith('defense ')) {
        for (const key of ['ownRear', 'span', 'bottomGap', 'move', 'power', 'error', 'homeOffset', 'lateralLimited', 'homeReady', 'return']) delete this.fields[key];
      }
      return true;
    }
  }
  if (typeof module !== 'undefined') module.exports = { Telemetry };
  else root.Telemetry = Telemetry;
})(typeof window === 'undefined' ? globalThis : window);
