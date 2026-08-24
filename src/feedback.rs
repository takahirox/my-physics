//! Rendering-independent continuous audio/force-feedback signals and discrete
//! physical events. Consumers choose their own sample rate and synthesis.

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AudioFrame {
    pub engine_rpm: f64,
    pub engine_load: f64,
    pub intake: f64,
    pub exhaust: f64,
    pub tire_scrub: [f64; 4],
    pub road_noise: [f64; 4],
    pub suspension_activity: [f64; 4],
    pub wind: f64,
    pub impact: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ForceFeedbackFrame {
    pub steering_torque_nm: f64,
    pub aligning_moment_nm: f64,
    pub rack_force_n: f64,
    pub road_vibration: f64,
    pub tire_scrub: f64,
    pub abs_pulse: f64,
    pub impact: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeedbackEventKind {
    GearShift,
    Impact,
    TireFailure,
    EngineFailure,
    ClutchFailure,
    GearboxFailure,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedbackEvent {
    pub time_s: f64,
    pub kind: FeedbackEventKind,
    pub magnitude: f64,
    pub wheel: Option<u8>,
}
