//! Driver input and electronic aids. This module is intentionally separate
//! from the physical plant so the same vehicle works with or without assists.

use crate::math::clamp01;

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
    tc_integrator: f64,
}

impl Default for DriverAids {
    fn default() -> Self {
        Self { abs_enabled: true, traction_control_enabled: true, stability_control_enabled: true, tc_integrator: 0.0 }
    }
}

impl DriverAids {
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
            for (n, slip) in s.wheel_slip.iter().enumerate() {
                if *slip < -0.16 && out.brake_per_wheel[n] > 0.0 {
                    out.brake_per_wheel[n] *= 0.35;
                    out.abs_active[n] = true;
                }
            }
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
            let desired_yaw = -out.steering * s.speed_mps / 8.5;
            let yaw_error = s.yaw_rate_rad_s - desired_yaw;
            if yaw_error.abs() > 0.22 {
                let wheel = if yaw_error > 0.0 { 0 } else { 1 };
                out.brake_per_wheel[wheel] = (out.brake_per_wheel[wheel] + yaw_error.abs() * 0.18).min(1.0);
                out.esc_active = true;
            }
        }
        out
    }
}
