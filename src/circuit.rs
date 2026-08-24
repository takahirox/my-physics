//! Shared deterministic geometry for the v0.1 demonstration circuit.

use crate::controls::DriverInput;
use crate::math::{Quat, Vec3};
use std::sync::OnceLock;

pub const CIRCUIT_SEGMENT_COUNT: usize = 160;
pub const CIRCUIT_HALF_WIDTH_M: f64 = 5.6;

const CONTROL_POINTS: [(f64, f64); 16] = [
    (10.0, -92.0),
    (-35.0, -92.0),
    (-78.0, -72.0),
    (-108.0, -38.0),
    (-112.0, 5.0),
    (-92.0, 48.0),
    (-55.0, 72.0),
    (-28.0, 105.0),
    (10.0, 92.0),
    (42.0, 105.0),
    (72.0, 72.0),
    (108.0, 58.0),
    (116.0, 15.0),
    (92.0, -18.0),
    (110.0, -58.0),
    (62.0, -88.0),
];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CircuitSegment {
    pub center_m: Vec3,
    pub forward: Vec3,
    pub right: Vec3,
    pub yaw_rad: f64,
    pub length_m: f64,
    pub distance_m: f64,
}

static SEGMENTS: OnceLock<Vec<CircuitSegment>> = OnceLock::new();

pub fn segments() -> &'static [CircuitSegment] {
    SEGMENTS.get_or_init(build_segments)
}

pub fn total_length_m() -> f64 {
    let list = segments();
    list.last().map_or(0.0, |segment| segment.distance_m + segment.length_m)
}

pub fn nearest_segment(position_m: Vec3) -> usize {
    segments()
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            planar_distance_squared(a.center_m, position_m).total_cmp(&planar_distance_squared(b.center_m, position_m))
        })
        .map_or(0, |(index, _)| index)
}

pub fn ai_driver_input(position_m: Vec3, orientation: Quat, speed_mps: f64, lane_offset_m: f64) -> DriverInput {
    let list = segments();
    let nearest = nearest_segment(position_m);
    let lookahead_m = 7.0 + speed_mps.max(0.0) * 0.3;
    let step_m = (total_length_m() / list.len() as f64).max(1.0);
    let lookahead = (lookahead_m / step_m).round().max(2.0) as usize;
    let target_segment = list[(nearest + lookahead) % list.len()];
    let target = target_segment.center_m + target_segment.right * lane_offset_m;
    let nearest_segment = list[nearest];
    let lateral_error_m = (position_m - nearest_segment.center_m).dot(nearest_segment.right) - lane_offset_m;
    let desired = Vec3::new(target.x - position_m.x, 0.0, target.z - position_m.z).normalized();
    let forward = orientation.rotate(Vec3::FORWARD);
    let heading_error = forward.cross(desired).y.atan2(forward.dot(desired));

    let curve_a = list[(nearest + lookahead) % list.len()].forward;
    let curve_b = list[(nearest + lookahead + 7) % list.len()].forward;
    let turn_angle = curve_a.dot(curve_b).clamp(-1.0, 1.0).acos();
    let target_speed = (31.0 - turn_angle * 55.0).clamp(10.0, 31.0);
    let speed_error = target_speed - speed_mps;

    DriverInput {
        steering: (-heading_error * 2.0 - lateral_error_m * 0.24).clamp(-1.0, 1.0),
        throttle: (0.34 + speed_error * 0.075).clamp(0.0, 0.82),
        brake: (-speed_error * 0.11).clamp(0.0, 0.8),
        ..DriverInput::default()
    }
}

fn build_segments() -> Vec<CircuitSegment> {
    let points: Vec<Vec3> = (0..CIRCUIT_SEGMENT_COUNT)
        .map(|sample| {
            let u = sample as f64 * CONTROL_POINTS.len() as f64 / CIRCUIT_SEGMENT_COUNT as f64;
            let index = u.floor() as usize;
            let t = u - index as f64;
            let len = CONTROL_POINTS.len();
            catmull_rom(
                point(CONTROL_POINTS[(index + len - 1) % len]),
                point(CONTROL_POINTS[index % len]),
                point(CONTROL_POINTS[(index + 1) % len]),
                point(CONTROL_POINTS[(index + 2) % len]),
                t,
            )
        })
        .collect();

    let mut distance_m = 0.0;
    let mut result = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        let center_m = points[index];
        let delta = points[(index + 1) % points.len()] - center_m;
        let length_m = delta.length();
        let forward = delta / length_m;
        let right = Vec3::new(-forward.z, 0.0, forward.x);
        let yaw_rad = (-forward.x).atan2(-forward.z);
        result.push(CircuitSegment { center_m, forward, right, yaw_rad, length_m, distance_m });
        distance_m += length_m;
    }
    result
}

fn point((x, z): (f64, f64)) -> Vec3 {
    Vec3::new(x, 0.0, z)
}

fn catmull_rom(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, t: f64) -> Vec3 {
    let t2 = t * t;
    let t3 = t2 * t;
    (p1 * 2.0 + (p2 - p0) * t + (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2 + (-p0 + p1 * 3.0 - p2 * 3.0 + p3) * t3)
        * 0.5
}

fn planar_distance_squared(a: Vec3, b: Vec3) -> f64 {
    let dx = a.x - b.x;
    let dz = a.z - b.z;
    dx * dx + dz * dz
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circuit_is_closed_and_f1_sized() {
        let list = segments();
        assert_eq!(list.len(), CIRCUIT_SEGMENT_COUNT);
        assert!((550.0..850.0).contains(&total_length_m()));
        let closure = (list[0].center_m - list[list.len() - 1].center_m).length();
        assert!(closure < 8.0, "closure gap {closure}");
        assert!(list.iter().all(|segment| (segment.forward.length() - 1.0).abs() < 1.0e-9));
    }

    #[test]
    fn ai_steers_toward_a_point_to_its_right() {
        let input = ai_driver_input(Vec3::ZERO, Quat::IDENTITY, 10.0, 0.0);
        assert!(input.steering.is_finite());
        assert!((0.0..=1.0).contains(&input.throttle));
    }
}
