//! Collision primitives and deterministic impulse helpers. Convex collision in
//! v0.1 uses a conservative bounding sphere; this approximation is explicit.

use crate::math::{Quat, Vec3};

#[derive(Clone, Debug, PartialEq)]
pub enum CollisionShape {
    Box { half_extents_m: Vec3 },
    Capsule { radius_m: f64, half_height_m: f64 },
    Convex { points_local_m: Vec<Vec3> },
}

impl CollisionShape {
    pub fn bounding_radius(&self) -> f64 {
        match self {
            Self::Box { half_extents_m } => half_extents_m.length(),
            Self::Capsule { radius_m, half_height_m } => radius_m + half_height_m,
            Self::Convex { points_local_m } => points_local_m.iter().map(|p| p.length()).fold(0.0, f64::max),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct StaticCollider {
    pub position_m: Vec3,
    pub orientation: Quat,
    pub shape: CollisionShape,
    pub restitution: f64,
    pub friction: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DetachedBody {
    pub position_m: Vec3,
    pub orientation: Quat,
    pub linear_velocity_mps: Vec3,
    pub angular_velocity_rad_s: Vec3,
    pub mass_kg: f64,
    pub shape: CollisionShape,
    pub damage: f64,
}

pub(crate) fn vehicle_planar_contact(
    position_a: Vec3,
    orientation_a: Quat,
    position_b: Vec3,
    orientation_b: Quat,
) -> Option<(Vec3, f64)> {
    let right_a = orientation_a.rotate(Vec3::X);
    let forward_a = orientation_a.rotate(Vec3::FORWARD);
    let right_b = orientation_b.rotate(Vec3::X);
    let forward_b = orientation_b.rotate(Vec3::FORWARD);
    let axes = [right_a, forward_a, right_b, forward_b];
    let delta = position_b - position_a;
    let mut least_overlap = f64::INFINITY;
    let mut normal = Vec3::ZERO;
    for raw_axis in axes {
        let axis = Vec3::new(raw_axis.x, 0.0, raw_axis.z).normalized();
        let radius_a = 0.95 * right_a.dot(axis).abs() + 2.15 * forward_a.dot(axis).abs();
        let radius_b = 0.95 * right_b.dot(axis).abs() + 2.15 * forward_b.dot(axis).abs();
        let signed_distance = delta.dot(axis);
        let overlap = radius_a + radius_b - signed_distance.abs();
        if overlap <= 0.0 {
            return None;
        }
        if overlap < least_overlap {
            least_overlap = overlap;
            normal = axis * signed_distance.signum();
        }
    }
    Some((normal, least_overlap))
}
