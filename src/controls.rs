//! Driver input and electronic aids. This module is intentionally separate
//! from the physical plant so the same vehicle works with or without assists.

use crate::math::clamp01;

/// Maximum normalized road-wheel request for a bounded dry lateral-
/// acceleration target. This input policy does not change the physical rack.
pub fn speed_sensitive_steering_limit(speed_mps: f64) -> f64 {
    const WHEELBASE_M: f64 = 2.51;
    const TARGET_LATERAL_ACCEL_MPS2: f64 = 10.5;
    const PHYSICAL_MAX_STEER_RAD: f64 = 0.54;
    if speed_mps <= 3.0 {
        1.0
    } else {
        ((WHEELBASE_M * TARGET_LATERAL_ACCEL_MPS2 / (speed_mps * speed_mps)).atan() / PHYSICAL_MAX_STEER_RAD)
            .clamp(0.03, 1.0)
    }
}

/// Deterministic digital-steering adapter stepped on the physics clock.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KeyboardSteeringAssist {
    output: f64,
}

impl KeyboardSteeringAssist {
    pub fn output(&self) -> f64 {
        self.output
    }

    pub fn reset(&mut self) {
        self.output = 0.0;
    }

    /// Seeds the adapter from the currently applied rack request. Input-mode
    /// changes use this for a bumpless transfer without changing rack travel.
    pub fn set_output(&mut self, output: f64) {
        self.output = output.clamp(-1.0, 1.0);
    }

    pub fn update(&mut self, direction: f64, speed_mps: f64, dt_s: f64) -> f64 {
        let target = direction.clamp(-1.0, 1.0) * speed_sensitive_steering_limit(speed_mps);
        let rate_per_s = if target == 0.0 {
            4.8
        } else if self.output != 0.0 && target.signum() != self.output.signum() {
            3.8
        } else {
            2.8
        };
        let delta = (target - self.output).clamp(-rate_per_s * dt_s, rate_per_s * dt_s);
        self.output = (self.output + delta).clamp(-1.0, 1.0);
        self.output
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DriverInput {
    pub steering: f64,
    pub throttle: f64,
    pub brake: f64,
    pub clutch: f64,
    pub handbrake: f64,
    pub gear_request: i8,
}

impl DriverInput {
    pub fn sanitized(self) -> Self {
        Self {
            steering: self.steering.clamp(-1.0, 1.0),
            throttle: clamp01(self.throttle),
            brake: clamp01(self.brake),
            clutch: clamp01(self.clutch),
            handbrake: clamp01(self.handbrake),
            gear_request: self.gear_request.clamp(-1, 6),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlOutput {
    pub steering: f64,
    pub throttle: f64,
    pub brake_per_wheel: [f64; 4],
    pub clutch: f64,
    pub gear_request: i8,
    pub abs_active: [bool; 4],
    pub tc_active: bool,
    pub esc_active: bool,
}

impl Default for ControlOutput {
    fn default() -> Self {
        Self {
            steering: 0.0,
            throttle: 0.0,
            brake_per_wheel: [0.0; 4],
            clutch: 0.0,
            gear_request: 0,
            abs_active: [false; 4],
            tc_active: false,
            esc_active: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AidSensors {
    pub wheel_slip: [f64; 4],
    pub speed_mps: f64,
    pub yaw_rate_rad_s: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DriverAids {
    pub abs_enabled: bool,
    pub traction_control_enabled: bool,
    pub stability_control_enabled: bool,
    abs_pressure: [f64; 4],
    tc_integrator: f64,
}

impl Default for DriverAids {
    fn default() -> Self {
        Self {
            abs_enabled: true,
            traction_control_enabled: true,
            stability_control_enabled: true,
            abs_pressure: [0.0; 4],
            tc_integrator: 0.0,
        }
    }
}

impl DriverAids {
    pub(crate) fn abs_pressure(&self) -> [f64; 4] {
        self.abs_pressure
    }
    pub(crate) fn set_abs_pressure(&mut self, pressure: [f64; 4]) {
        self.abs_pressure = pressure.map(|value| value.clamp(0.0, 1.0));
    }
    pub(crate) fn integrator(&self) -> f64 {
        self.tc_integrator
    }
    pub(crate) fn set_integrator(&mut self, value: f64) {
        self.tc_integrator = value.clamp(0.0, 0.5);
    }
    pub fn update(&mut self, input: DriverInput, s: AidSensors, dt: f64) -> ControlOutput {
        let i = input.sanitized();
        let mut out = ControlOutput {
            steering: i.steering,
            throttle: i.throttle,
            brake_per_wheel: [i.brake; 4],
            clutch: i.clutch,
            gear_request: i.gear_request,
            ..ControlOutput::default()
        };
        out.brake_per_wheel[2] = (out.brake_per_wheel[2] + i.handbrake).min(1.0);
        out.brake_per_wheel[3] = (out.brake_per_wheel[3] + i.handbrake).min(1.0);
        if self.abs_enabled && s.speed_mps > 2.0 {
            for (n, slip) in s.wheel_slip.iter().copied().enumerate() {
                let requested = out.brake_per_wheel[n];
                let pressure = &mut self.abs_pressure[n];
                if requested <= 0.0 {
                    *pressure = (*pressure - 18.0 * dt).max(0.0);
                    out.brake_per_wheel[n] = 0.0;
                    continue;
                }

                // Stateful pressure modulation keeps the tire on the broad
                // dry-force peak instead of switching between full pressure
                // and a single reduced value after the wheel has locked.
                let rate_per_s = if slip < -0.30 {
                    -28.0
                } else if slip < -0.18 {
                    -12.0
                } else if slip < -0.14 {
                    -3.0
                } else if slip <= -0.08 {
                    2.5
                } else if slip <= -0.05 {
                    7.0
                } else {
                    20.0
                };
                *pressure = (*pressure + rate_per_s * dt).clamp(0.0, requested);
                out.brake_per_wheel[n] = *pressure;
                out.abs_active[n] = (*pressure - requested).abs() > 1.0e-9 || slip < -0.14;
            }
        } else {
            self.abs_pressure = out.brake_per_wheel;
        }
        if self.traction_control_enabled {
            let driven_slip = s.wheel_slip[2].max(s.wheel_slip[3]);
            let error = (driven_slip - 0.11).max(0.0);
            self.tc_integrator = (self.tc_integrator + error * dt).clamp(0.0, 0.5);
            if error > 0.0 {
                out.throttle = (out.throttle - error * 2.8 - self.tc_integrator).max(0.0);
                out.tc_active = true;
            } else {
                self.tc_integrator = (self.tc_integrator - dt * 0.5).max(0.0);
            }
        }
        if self.stability_control_enabled && s.speed_mps > 5.0 {
            // A bicycle-model reference is bounded by the yaw rate that the
            // available tire friction can physically sustain. The previous
            // normalized-steering shortcut requested several radians/second
            // at motorway speeds and then braked the wheel that reinforced
            // the error, so full keyboard lock suppressed the turn.
            let road_wheel_angle = out.steering * 0.54;
            let kinematic_yaw = -s.speed_mps * road_wheel_angle.tan() / 2.51;
            let friction_limited_yaw = 1.15 * 9.80665 / s.speed_mps.max(1.0);
            let desired_yaw = kinematic_yaw.clamp(-friction_limited_yaw, friction_limited_yaw);
            let actual_yaw = s.yaw_rate_rad_s;
            let same_turn_direction = desired_yaw * actual_yaw > 0.0;
            let oversteer = same_turn_direction && actual_yaw.abs() > desired_yaw.abs() + 0.16;
            let opposite_yaw = desired_yaw.abs() > 0.05 && desired_yaw * actual_yaw < -0.01;
            let uncommanded_yaw = desired_yaw.abs() <= 0.05 && actual_yaw.abs() > 0.22;

            let correction = if oversteer || uncommanded_yaw {
                // Counter the current yaw with the outside front wheel.
                Some((if actual_yaw > 0.0 { 1 } else { 0 }, actual_yaw.abs() - desired_yaw.abs()))
            } else if opposite_yaw {
                // Establish the requested yaw with the inside rear wheel.
                Some((if desired_yaw > 0.0 { 2 } else { 3 }, (actual_yaw - desired_yaw).abs()))
            } else {
                None
            };

            if let Some((wheel, error)) = correction {
                let intervention = ((error - 0.06).max(0.0) * 0.32).min(0.38);
                out.brake_per_wheel[wheel] = (out.brake_per_wheel[wheel] + intervention).min(1.0);
                out.esc_active = true;
            }
        }
        out
    }
}
