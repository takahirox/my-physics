//! Collision primitives and deterministic planar narrow-phase helpers.

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

pub(crate) fn oriented_box_contact(
    position_a: Vec3,
    orientation_a: Quat,
    half_a: Vec3,
    position_b: Vec3,
    orientation_b: Quat,
    half_b: Vec3,
) -> Option<(Vec3, f64)> {
    if (position_b.y - position_a.y).abs() > half_a.y + half_b.y {
        return None;
    }
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
        let radius_a = half_a.x * right_a.dot(axis).abs() + half_a.z * forward_a.dot(axis).abs();
        let radius_b = half_b.x * right_b.dot(axis).abs() + half_b.z * forward_b.dot(axis).abs();
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

pub(crate) fn vehicle_static_contact(
    vehicle_position: Vec3,
    vehicle_orientation: Quat,
    vehicle_half: Vec3,
    collider: &StaticCollider,
) -> Option<(Vec3, f64)> {
    match &collider.shape {
        CollisionShape::Box { half_extents_m } => oriented_box_contact(
            collider.position_m,
            collider.orientation,
            *half_extents_m,
            vehicle_position,
            vehicle_orientation,
            vehicle_half,
        ),
        CollisionShape::Capsule { radius_m, half_height_m } => {
            let axis = collider.orientation.rotate(Vec3::Y);
            let along = (vehicle_position - collider.position_m).dot(axis).clamp(-*half_height_m, *half_height_m);
            let center = collider.position_m + axis * along;
            circle_box_contact(center, *radius_m, vehicle_position, vehicle_orientation, vehicle_half)
        }
        CollisionShape::Convex { points_local_m } => {
            convex_box_contact(points_local_m, collider, vehicle_position, vehicle_orientation, vehicle_half)
        }
    }
}

fn circle_box_contact(
    circle_center: Vec3,
    radius: f64,
    box_position: Vec3,
    box_orientation: Quat,
    box_half: Vec3,
) -> Option<(Vec3, f64)> {
    if (circle_center.y - box_position.y).abs() > box_half.y + radius {
        return None;
    }
    let local = box_orientation.conjugate().rotate(circle_center - box_position);
    let closest = Vec3::new(local.x.clamp(-box_half.x, box_half.x), local.y, local.z.clamp(-box_half.z, box_half.z));
    let delta_local = local - closest;
    let planar = Vec3::new(delta_local.x, 0.0, delta_local.z);
    let distance = planar.length();
    if distance >= radius {
        return None;
    }
    let local_normal = if distance > 1.0e-9 {
        -planar / distance
    } else {
        let dx = box_half.x - local.x.abs();
        let dz = box_half.z - local.z.abs();
        if dx < dz { Vec3::new(-local.x.signum(), 0.0, 0.0) } else { Vec3::new(0.0, 0.0, -local.z.signum()) }
    };
    Some((box_orientation.rotate(local_normal), radius - distance))
}

fn convex_box_contact(
    points: &[Vec3],
    collider: &StaticCollider,
    box_position: Vec3,
    box_orientation: Quat,
    box_half: Vec3,
) -> Option<(Vec3, f64)> {
    if points.len() < 3 {
        return None;
    }
    let polygon: Vec<Vec3> =
        points.iter().map(|point| collider.position_m + collider.orientation.rotate(*point)).collect();
    let min_y = polygon.iter().map(|point| point.y).fold(f64::INFINITY, f64::min);
    let max_y = polygon.iter().map(|point| point.y).fold(f64::NEG_INFINITY, f64::max);
    if box_position.y + box_half.y < min_y || box_position.y - box_half.y > max_y {
        return None;
    }
    let right = box_orientation.rotate(Vec3::X);
    let forward = box_orientation.rotate(Vec3::FORWARD);
    let mut axes = vec![right, forward];
    for index in 0..polygon.len() {
        let edge = polygon[(index + 1) % polygon.len()] - polygon[index];
        let axis = Vec3::new(-edge.z, 0.0, edge.x).normalized();
        if axis.length_squared() > 0.5 {
            axes.push(axis);
        }
    }
    let mut least_overlap = f64::INFINITY;
    let mut normal = Vec3::ZERO;
    for axis in axes {
        let (poly_min, poly_max) = project_points(&polygon, axis);
        let center = box_position.dot(axis);
        let radius = box_half.x * right.dot(axis).abs() + box_half.z * forward.dot(axis).abs();
        let overlap = poly_max.min(center + radius) - poly_min.max(center - radius);
        if overlap <= 0.0 {
            return None;
        }
        if overlap < least_overlap {
            least_overlap = overlap;
            let polygon_center = (poly_min + poly_max) * 0.5;
            normal = axis * (center - polygon_center).signum();
        }
    }
    Some((normal, least_overlap))
}

fn project_points(points: &[Vec3], axis: Vec3) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for point in points {
        let projection = point.dot(axis);
        min = min.min(projection);
        max = max.max(projection);
    }
    (min, max)
}
