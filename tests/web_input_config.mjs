import assert from 'node:assert/strict';
import test from 'node:test';
import {
  DEFAULT_INPUT_CONFIG,
  DeviceActivityLatch,
  captureRestCalibration,
  inputConfigFromSources,
  inputConfigForDevice,
  normalizeCenteredAxis,
  normalizePedalAxis,
  sharedInputConfigForPersistence,
} from '../web/input-config.mjs';

test('URL values override persisted calibration and remain bounded', () => {
  const stored = JSON.stringify({
    gamepadDeadzone: 0.15,
    gamepadExponent: 1.2,
    keyboardAdaptive: false,
    steeringCenter: 0.03,
    calibratedDevice: 'Stored Wheel',
  });
  const config = inputConfigFromSources('?inputDeadzone=0.06&inputExpo=1.8&inputMode=adaptive&steerCenter=.02', stored);
  assert.equal(config.gamepadDeadzone, 0.06);
  assert.equal(config.gamepadExponent, 1.8);
  assert.equal(config.keyboardAdaptive, true);
  assert.equal(config.driveProfile, 'sport');
  assert.equal(config.steeringCenter, 0.02);
  assert.equal(config.calibratedDevice, '', 'explicit URL calibration is portable rather than tied to stored device id');
  assert.equal(inputConfigFromSources('?inputDeadzone=99').gamepadDeadzone, 0.35);
});

test('arcade profile storage can share hardware calibration without poisoning simulation profile', () => {
  const arcade = inputConfigFromSources('?driveProfile=arcade&steerCenter=.025&brakeReleased=.88');
  const shared = sharedInputConfigForPersistence(arcade, 'simulation', true);
  assert.equal(shared.driveProfile, 'simulation');
  assert.equal(shared.keyboardAdaptive, false);
  assert.equal(shared.steeringCenter, 0.025);
  assert.equal(shared.brakeReleased, 0.88);
  assert.equal(sharedInputConfigForPersistence(arcade, 'simulation', false).driveProfile, 'arcade');
});

test('drive profile persists, accepts URL override, and migrates legacy raw mode', () => {
  assert.equal(inputConfigFromSources('').driveProfile, 'sport');
  assert.equal(inputConfigFromSources('', JSON.stringify({ driveProfile: 'accessible' })).driveProfile, 'accessible');
  assert.equal(
    inputConfigFromSources('?driveProfile=simulation', JSON.stringify({ driveProfile: 'accessible' })).driveProfile,
    'simulation',
  );
  const migrated = inputConfigFromSources('', JSON.stringify({ keyboardAdaptive: false }));
  assert.equal(migrated.driveProfile, 'simulation');
  assert.equal(migrated.keyboardAdaptive, false);
  assert.equal(inputConfigFromSources('?driveProfile=invalid').driveProfile, 'sport');
});

test('gamepad steering has symmetric deadzones/expo while wheel stays calibrated linear', () => {
  const config = inputConfigFromSources('');
  assert.equal(normalizeCenteredAxis(0.04, config, false), 0);
  assert.equal(normalizeCenteredAxis(-0.04, config, false), 0);
  assert.equal(normalizeCenteredAxis(1, config, false), 1);
  assert.equal(normalizeCenteredAxis(-1, config, false), -1);
  assert.ok(Math.abs(normalizeCenteredAxis(0.5, config, false)) < Math.abs(normalizeCenteredAxis(0.5, config, true)));
  assert.equal(normalizeCenteredAxis(0.5, config, true), 0.5);
  assert.ok(Math.abs(normalizeCenteredAxis(0.5, config, true) + normalizeCenteredAxis(-0.5, config, true)) < 1e-12);
});

test('pedal calibration supports either axis direction', () => {
  assert.equal(normalizePedalAxis(1, 1, -1), 0);
  assert.equal(normalizePedalAxis(-1, 1, -1), 1);
  assert.equal(normalizePedalAxis(0, 1, -1), 0.5);
  assert.equal(normalizePedalAxis(-1, -1, 1), 0);
  assert.equal(normalizePedalAxis(1, -1, 1), 1);
});

test('activity latch ignores idle controllers and permits deliberate device changes', () => {
  const latch = new DeviceActivityLatch();
  assert.equal(latch.select([{ key: 'keyboard', magnitude: 0 }, { key: 'pad:0', magnitude: 0.04 }], 0), 'keyboard');
  assert.equal(latch.select([{ key: 'keyboard', magnitude: 0 }, { key: 'pad:0', magnitude: 0.8 }], 1), 'pad:0');
  assert.equal(latch.select([{ key: 'keyboard', magnitude: 0 }, { key: 'pad:0', magnitude: 0.7 }], 200), 'pad:0');
  assert.equal(latch.select([{ key: 'keyboard', magnitude: 1, priority: true }, { key: 'pad:0', magnitude: 0 }], 300), 'keyboard');
});

test('rest calibration captures center and released pedals without altering endpoints', () => {
  const config = captureRestCalibration(DEFAULT_INPUT_CONFIG, { id: 'Test Wheel' }, {
    steering: 0.025,
    throttle: 0.91,
    brake: 0.88,
    clutch: 0.93,
  });
  assert.equal(config.calibratedDevice, 'Test Wheel');
  assert.equal(config.steeringCenter, 0.025);
  assert.equal(config.brakeReleased, 0.88);
  assert.equal(config.brakePressed, -1);
  assert.equal(inputConfigForDevice(config, 'Test Wheel').steeringCenter, 0.025);
  assert.equal(inputConfigForDevice(config, 'Different Wheel').steeringCenter, 0);
  assert.equal(inputConfigForDevice(config, 'Different Wheel').brakeReleased, 1);
});
