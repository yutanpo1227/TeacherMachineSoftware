const { test } = require('node:test');
const assert = require('node:assert/strict');
const { Telemetry } = require('../telemetry');
test('structured telemetry parses all values, including missing sensors', () => {
  const t = new Telemetry();
  assert.equal(t.parse('telemetry role=DEFENSE state=GUARD yaw=3 ball=90 ir=89 cameraBall=-1 line=180 lineStrength=45 yellow=270 blue=-1 ownRear=270 camera=1 error=-4 bottomGap=65 lineUsed=0', 100), true);
  assert.equal(t.get('ball', 100), 90); assert.equal(t.get('blue', 100), -1); assert.equal(t.get('lineUsed', 100), 0);
  assert.equal(t.get('ball', 2000), null);
});
test('legacy logs and goal loss invalidate obsolete readings', () => {
  const t = new Telemetry();
  t.parse('ボール角度: 110', 10); t.parse('defense state=GUARD ownRear=270 move=90 power=200', 10);
  assert.equal(t.get('ball', 10), 110); assert.equal(t.get('ownRear', 10), 270);
  t.parse('defense state=GOAL_MISSING yaw=0 camera=1', 20);
  assert.equal(t.get('ownRear', 20), null); assert.equal(t.get('move', 20), null);
  t.reset(); assert.equal(t.get('ball', 20), null);
});
