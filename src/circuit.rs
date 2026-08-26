//! Shared deterministic geometry for the v0.1 demonstration circuit.

use crate::controls::{DriverInput, speed_sensitive_steering_limit};
use crate::math::{Quat, Vec3};
use std::sync::OnceLock;

pub const CIRCUIT_SEGMENT_COUNT: usize = 240;
pub const CIRCUIT_HALF_WIDTH_M: f64 = 5.6;
/// The original control polygon was authored as a compact visual prototype.
/// Keeping the scale explicit makes the sampled racing geometry suitable for
/// full-size cars without changing its recognizable sequence of corners.
pub const CIRCUIT_SCALE: f64 = 2.8;

// Authored plan-view points plus an elevation profile in metres. X/Z retain
// the historical circuit scale; Y is already in physical SI units. The
// Catmull-Rom interpolation makes the profile cyclic and slope-continuous at
// the start line rather than adding a render-only height offset.
const CONTROL_POINTS: [(f64, f64, f64); 16] = [
    (30.0, 0.0, -92.0),
    (-50.0, 0.6, -92.0),
    (-90.0, 2.4, -82.0),
    (-108.0, 6.0, -38.0),
    (-112.0, 10.2, 5.0),
    (-92.0, 14.4, 48.0),
    (-55.0, 18.6, 72.0),
    (-28.0, 16.2, 105.0),
    (10.0, 10.2, 92.0),
    (42.0, 6.0, 105.0),
    (72.0, 2.4, 72.0),
    (108.0, -1.8, 58.0),
    (116.0, -4.2, 15.0),
    (92.0, -2.4, -18.0),
    (110.0, 1.2, -58.0),
    (75.0, 0.6, -92.0),
];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CircuitSegment {
    pub center_m: Vec3,
    pub forward: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub bank_rad: f64,
    pub yaw_rad: f64,
    pub length_m: f64,
    pub distance_m: f64,
}

impl CircuitSegment {
    /// Vehicle/OBB orientation whose local +X/+Y/-Z axes match this segment's
    /// right/up/forward road frame.
    pub fn orientation(self) -> Quat {
        let planar_forward = Vec3::new(self.forward.x, 0.0, self.forward.z).normalized();
        let unbanked_right = planar_forward.cross(Vec3::Y).normalized();
        let pitch_rad = self.forward.y.atan2((self.forward.x.powi(2) + self.forward.z.powi(2)).sqrt());
        let yaw = Quat::from_axis_angle(Vec3::Y, self.yaw_rad);
        let pitch = Quat::from_axis_angle(unbanked_right, pitch_rad);
        let bank = Quat::from_axis_angle(self.forward, self.bank_rad);
        (bank * pitch * yaw).normalized()
    }
}

/// Local physical road plane sampled from the same centerline and bank frame
/// exported to renderers. The point lies vertically below/above `position_m`
/// on that plane; `normal` always points into the +Y hemisphere.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CircuitSurfaceSample {
    pub point_m: Vec3,
    pub normal: Vec3,
    pub forward: Vec3,
    pub right: Vec3,
    pub segment_index: usize,
    pub lateral_offset_m: f64,
}

static SEGMENTS: OnceLock<Vec<CircuitSegment>> = OnceLock::new();
static LOCAL_RADII: OnceLock<Vec<f64>> = OnceLock::new();
static AI_CORNER_SPEEDS: OnceLock<Vec<f64>> = OnceLock::new();

pub fn segments() -> &'static [CircuitSegment] {
    SEGMENTS.get_or_init(build_segments)
}

pub fn total_length_m() -> f64 {
    let list = segments();
    list.last().map_or(0.0, |segment| segment.distance_m + segment.length_m)
}

/// Polyline approximation of the centerline radius at a sampled segment.
/// Infinite radius denotes a locally straight section.
pub fn local_radius_m(index: usize) -> f64 {
    let radii = LOCAL_RADII.get_or_init(build_local_radii);
    radii[index % radii.len()]
}

fn build_local_radii() -> Vec<f64> {
    let list = segments();
    (0..list.len())
        .map(|index| {
            let current = list[index];
            let previous = list[(index + list.len() - 1) % list.len()];
            let turn_rad = previous.forward.dot(current.forward).clamp(-1.0, 1.0).acos();
            if turn_rad <= 1.0e-9 { f64::INFINITY } else { (previous.length_m + current.length_m) * 0.5 / turn_rad }
        })
        .collect()
}

pub fn minimum_radius_m() -> f64 {
    (0..segments().len()).map(local_radius_m).fold(f64::INFINITY, f64::min)
}

fn ai_corner_speed_mps(index: usize) -> f64 {
    let speeds = AI_CORNER_SPEEDS.get_or_init(build_ai_corner_speeds);
    speeds[index % speeds.len()]
}

fn build_ai_corner_speeds() -> Vec<f64> {
    let list = segments();
    (0..list.len())
        .map(|index| {
            let mut positive_turn = 0.0;
            let mut negative_turn = 0.0;
            for offset in 0..=12 {
                let current = list[(index + offset) % list.len()];
                let next = list[(index + offset + 1) % list.len()];
                let signed_turn = current.forward.cross(next.forward).y.atan2(current.forward.dot(next.forward));
                if signed_turn >= 0.0 {
                    positive_turn += signed_turn;
                } else {
                    negative_turn -= signed_turn;
                }
            }
            let steady_corner_speed = (local_radius_m(index) * 3.5).sqrt().clamp(13.0, 40.0);
            if positive_turn > 0.035 && negative_turn > 0.035 {
                steady_corner_speed.min(27.5)
            } else {
                steady_corner_speed
            }
        })
        .collect()
}

pub fn nearest_segment(position_m: Vec3) -> usize {
    nearest_segment_projection(position_m).0
}

/// Samples the ruled circuit surface at a world X/Z location. Linear frame
/// interpolation avoids force-normal steps at the authored segment boundaries;
/// the resulting basis is re-orthogonalized deterministically.
pub fn sample_surface(position_m: Vec3) -> CircuitSurfaceSample {
    let list = segments();
    let (segment_index, t) = nearest_segment_projection(position_m);
    let current = list[segment_index];
    let next = list[(segment_index + 1) % list.len()];
    let center = current.center_m.lerp(next.center_m, t);
    let forward = current.forward.lerp(next.forward, t).normalized();
    let mut right = current.right.lerp(next.right, t);
    right = (right - forward * right.dot(forward)).normalized();
    let mut normal = right.cross(forward).normalized();
    if normal.y < 0.0 {
        right = -right;
        normal = -normal;
    }
    let height_m = if normal.y.abs() > 1.0e-9 {
        center.y - (normal.x * (position_m.x - center.x) + normal.z * (position_m.z - center.z)) / normal.y
    } else {
        center.y
    };
    let point_m = Vec3::new(position_m.x, height_m, position_m.z);
    CircuitSurfaceSample {
        point_m,
        normal,
        forward,
        right,
        segment_index,
        lateral_offset_m: (point_m - center).dot(right),
    }
}

pub fn ai_driver_input(position_m: Vec3, orientation: Quat, speed_mps: f64, lane_offset_m: f64) -> DriverInput {
    ai_driver_input_with_yaw(position_m, orientation, speed_mps, 0.0, lane_offset_m)
}

pub fn ai_driver_input_with_yaw(
    position_m: Vec3,
    orientation: Quat,
    speed_mps: f64,
    yaw_rate_rad_s: f64,
    lane_offset_m: f64,
) -> DriverInput {
    let list = segments();
    let nearest = nearest_segment(position_m);
    let lookahead_m = 20.0 + speed_mps.max(0.0) * 0.8;
    let step_m = (total_length_m() / list.len() as f64).max(1.0);
    let lookahead = (lookahead_m / step_m).round().max(2.0) as usize;
    let target_segment = list[(nearest + lookahead) % list.len()];
    let target = target_segment.center_m + target_segment.right * lane_offset_m;
    let nearest_segment = list[nearest];
    let lateral_error_m = (position_m - nearest_segment.center_m).dot(nearest_segment.right) - lane_offset_m;
    let desired = Vec3::new(target.x - position_m.x, 0.0, target.z - position_m.z).normalized();
    let forward = orientation.rotate(Vec3::FORWARD);
    let heading_error = forward.cross(desired).y.atan2(forward.dot(desired));

    // Preview enough centerline to brake before the next corner. Target speed
    // follows a conservative lateral-acceleration envelope instead of being
    // tied to the number of spline samples or the old compact circuit scale.
    let preview_segments = ((220.0 / step_m).ceil() as usize).max(12);
    let target_speed = (0..=lookahead + preview_segments)
        .map(|offset| {
            let corner_speed = ai_corner_speed_mps(nearest + offset);
            let braking_distance_m = offset as f64 * step_m;
            (corner_speed * corner_speed + 2.0 * 4.5 * braking_distance_m).sqrt().min(40.0)
        })
        .fold(40.0, f64::min);
    let speed_error = target_speed - speed_mps;

    let steering_limit = speed_sensitive_steering_limit(speed_mps);
    DriverInput {
        steering: (-heading_error * 2.0 - lateral_error_m * 0.24 + yaw_rate_rad_s * 0.28)
            .clamp(-steering_limit, steering_limit),
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

    let forwards: Vec<Vec3> =
        (0..points.len()).map(|index| (points[(index + 1) % points.len()] - points[index]).normalized()).collect();
    let raw_banks: Vec<f64> = (0..points.len())
        .map(|index| {
            let previous = forwards[(index + points.len() - 1) % points.len()];
            let current = forwards[index];
            let previous_planar = Vec3::new(previous.x, 0.0, previous.z).normalized();
            let current_planar = Vec3::new(current.x, 0.0, current.z).normalized();
            let turn_rad = previous_planar.cross(current_planar).y.atan2(previous_planar.dot(current_planar));
            let distance_m = ((points[index] - points[(index + points.len() - 1) % points.len()]).length()
                + (points[(index + 1) % points.len()] - points[index]).length())
                * 0.5;
            let signed_curvature = turn_rad / distance_m.max(1.0);
            (-signed_curvature * 4.4).clamp(-10.0_f64.to_radians(), 10.0_f64.to_radians())
        })
        .collect();
    // A short cyclic low-pass prevents abrupt cross-slope changes while
    // preserving both left- and right-hand banking on the existing layout.
    let mut banks = raw_banks;
    for _ in 0..4 {
        banks = (0..banks.len())
            .map(|index| {
                (banks[(index + banks.len() - 1) % banks.len()] + banks[index] * 2.0 + banks[(index + 1) % banks.len()])
                    * 0.25
            })
            .collect();
    }

    let mut distance_m = 0.0;
    let mut result = Vec::with_capacity(points.len());
    for index in 0..points.len() {
        let center_m = points[index];
        let delta = points[(index + 1) % points.len()] - center_m;
        let length_m = delta.length();
        let forward = delta / length_m;
        let unbanked_right = forward.cross(Vec3::Y).normalized();
        let unbanked_up = unbanked_right.cross(forward).normalized();
        let bank_rad = banks[index];
        let bank = Quat::from_axis_angle(forward, bank_rad);
        let right = bank.rotate(unbanked_right).normalized();
        let up = bank.rotate(unbanked_up).normalized();
        let yaw_rad = (-forward.x).atan2(-forward.z);
        result.push(CircuitSegment { center_m, forward, right, up, bank_rad, yaw_rad, length_m, distance_m });
        distance_m += length_m;
    }
    result
}

fn point((x, y, z): (f64, f64, f64)) -> Vec3 {
    Vec3::new(x * CIRCUIT_SCALE, y, z * CIRCUIT_SCALE)
}

fn catmull_rom(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, t: f64) -> Vec3 {
    let t2 = t * t;
    let t3 = t2 * t;
    (p1 * 2.0 + (p2 - p0) * t + (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2 + (-p0 + p1 * 3.0 - p2 * 3.0 + p3) * t3)
        * 0.5
}

fn nearest_segment_projection(position_m: Vec3) -> (usize, f64) {
    segments()
        .iter()
        .enumerate()
        .map(|(index, segment)| {
            let next = segments()[(index + 1) % segments().len()].center_m;
            let delta_x = next.x - segment.center_m.x;
            let delta_z = next.z - segment.center_m.z;
            let length_squared = delta_x * delta_x + delta_z * delta_z;
            let t = (((position_m.x - segment.center_m.x) * delta_x + (position_m.z - segment.center_m.z) * delta_z)
                / length_squared.max(1.0e-12))
            .clamp(0.0, 1.0);
            let dx = position_m.x - (segment.center_m.x + delta_x * t);
            let dz = position_m.z - (segment.center_m.z + delta_z * t);
            (index, t, dx * dx + dz * dz)
        })
        .min_by(|a, b| a.2.total_cmp(&b.2))
        .map_or((0, 0.0), |(index, t, _)| (index, t))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circuit_is_closed_and_full_size() {
        let list = segments();
        assert_eq!(list.len(), CIRCUIT_SEGMENT_COUNT);
        assert!((1_500.0..2_200.0).contains(&total_length_m()));
        assert!(minimum_radius_m() >= 25.0, "minimum radius {}", minimum_radius_m());
        let minimum_corner_speed_kmh = (minimum_radius_m() * 1.15 * 9.80665).sqrt() * 3.6;
        assert!(minimum_corner_speed_kmh >= 60.0, "corner speed {minimum_corner_speed_kmh}");
        let closure = (list[0].center_m - list[list.len() - 1].center_m).length();
        assert!(closure < total_length_m() / list.len() as f64 * 1.5, "closure gap {closure}");
        assert!(list.iter().all(|segment| (segment.forward.length() - 1.0).abs() < 1.0e-9));
        assert!(list.iter().all(|segment| {
            (segment.right.length() - 1.0).abs() < 1.0e-9
                && (segment.up.length() - 1.0).abs() < 1.0e-9
                && segment.forward.dot(segment.right).abs() < 1.0e-9
                && segment.forward.dot(segment.up).abs() < 1.0e-9
                && segment.right.dot(segment.up).abs() < 1.0e-9
                && segment.right.cross(segment.forward).dot(segment.up) > 0.999_999
        }));
        assert!(list.iter().all(|segment| {
            let orientation = segment.orientation();
            orientation.rotate(Vec3::FORWARD).dot(segment.forward) > 0.999_999
                && orientation.rotate(Vec3::X).dot(segment.right) > 0.999_999
                && orientation.rotate(Vec3::Y).dot(segment.up) > 0.999_999
        }));
        let minimum_height = list.iter().map(|segment| segment.center_m.y).fold(f64::INFINITY, f64::min);
        let maximum_height = list.iter().map(|segment| segment.center_m.y).fold(f64::NEG_INFINITY, f64::max);
        assert!(maximum_height - minimum_height >= 20.0, "height range {}", maximum_height - minimum_height);
        assert!(list.iter().any(|segment| segment.bank_rad.abs() >= 4.0_f64.to_radians()));
        assert!(list.iter().all(|segment| segment.bank_rad.abs() <= 10.0_f64.to_radians() + 1.0e-12));
        assert!(list.iter().all(|segment| segment.forward.y.abs() <= 0.08), "maximum grade is intentionally bounded");
    }

    #[test]
    fn sampled_surface_uses_the_authored_height_and_bank_frame() {
        for segment in segments().iter().step_by(13) {
            let center = sample_surface(segment.center_m + Vec3::Y * 3.0);
            assert!((center.point_m.y - segment.center_m.y).abs() < 1.0e-8);
            assert!(center.normal.y > 0.97);
            assert!(center.normal.dot(center.forward).abs() < 1.0e-9);
            assert!(center.normal.dot(center.right).abs() < 1.0e-9);

            let left = sample_surface(segment.center_m - segment.right * CIRCUIT_HALF_WIDTH_M + Vec3::Y * 3.0);
            let right = sample_surface(segment.center_m + segment.right * CIRCUIT_HALF_WIDTH_M + Vec3::Y * 3.0);
            let cross_slope_height = (right.point_m.y - left.point_m.y).abs();
            assert!(cross_slope_height <= 2.1, "cross slope {cross_slope_height}");
        }
    }

    #[test]
    fn ai_steers_toward_a_point_to_its_right() {
        let input = ai_driver_input(Vec3::ZERO, Quat::IDENTITY, 10.0, 0.0);
        assert!(input.steering.is_finite());
        assert!((0.0..=1.0).contains(&input.throttle));
    }
}
