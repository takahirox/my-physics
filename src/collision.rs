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

/// Geometric contact returned by the deterministic narrow phase. `normal`
/// points from shape A (the first argument) toward shape B, and `point_m` is
/// the world-space point at which the impulse is applied.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Contact {
    pub normal: Vec3,
    pub penetration_m: f64,
    pub point_m: Vec3,
}

pub(crate) fn oriented_box_contact(
    position_a: Vec3,
    orientation_a: Quat,
    half_a: Vec3,
    position_b: Vec3,
    orientation_b: Quat,
    half_b: Vec3,
) -> Option<Contact> {
    let axes_a = box_axes(orientation_a);
    let axes_b = box_axes(orientation_b);
    let delta = position_b - position_a;
    let mut least_overlap = f64::INFINITY;
    let mut normal = Vec3::ZERO;
    let mut candidate_axes = Vec::with_capacity(15);
    candidate_axes.extend(axes_a);
    candidate_axes.extend(axes_b);
    for axis_a in axes_a {
        for axis_b in axes_b {
            candidate_axes.push(axis_a.cross(axis_b));
        }
    }
    for raw_axis in candidate_axes {
        if raw_axis.length_squared() < 1.0e-12 {
            continue;
        }
        let axis = raw_axis.normalized();
        let radius_a = box_projection_radius(axes_a, half_a, axis);
        let radius_b = box_projection_radius(axes_b, half_b, axis);
        let signed_distance = delta.dot(axis);
        let overlap = radius_a + radius_b - signed_distance.abs();
        if overlap <= 0.0 {
            return None;
        }
        if overlap < least_overlap {
            least_overlap = overlap;
            normal = axis * if signed_distance >= 0.0 { 1.0 } else { -1.0 };
        }
    }
    if normal.length_squared() < 0.5 {
        return None;
    }
    let point_a = box_support(position_a, orientation_a, half_a, normal);
    let point_b = box_support(position_b, orientation_b, half_b, -normal);
    Some(Contact { normal, penetration_m: least_overlap, point_m: (point_a + point_b) * 0.5 })
}

pub(crate) fn vehicle_static_contact(
    vehicle_position: Vec3,
    vehicle_orientation: Quat,
    vehicle_half: Vec3,
    collider: &StaticCollider,
) -> Option<Contact> {
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
) -> Option<Contact> {
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
    let normal = box_orientation.rotate(local_normal).normalized();
    let box_point = box_position + box_orientation.rotate(closest);
    let circle_point = circle_center + normal * radius;
    Some(Contact { normal, penetration_m: radius - distance, point_m: (box_point + circle_point) * 0.5 })
}

fn convex_box_contact(
    points: &[Vec3],
    collider: &StaticCollider,
    box_position: Vec3,
    box_orientation: Quat,
    box_half: Vec3,
) -> Option<Contact> {
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
    let polygon_point = support_points(&polygon, normal);
    let box_point = box_support(box_position, box_orientation, box_half, -normal);
    Some(Contact { normal, penetration_m: least_overlap, point_m: (polygon_point + box_point) * 0.5 })
}

fn box_axes(orientation: Quat) -> [Vec3; 3] {
    [orientation.rotate(Vec3::X), orientation.rotate(Vec3::Y), orientation.rotate(Vec3::new(0.0, 0.0, 1.0))]
}

fn box_projection_radius(axes: [Vec3; 3], half: Vec3, direction: Vec3) -> f64 {
    half.x * axes[0].dot(direction).abs()
        + half.y * axes[1].dot(direction).abs()
        + half.z * axes[2].dot(direction).abs()
}

fn support_component(direction: f64, extent: f64) -> f64 {
    if direction.abs() < 1.0e-12 { 0.0 } else { direction.signum() * extent }
}

fn box_support(position: Vec3, orientation: Quat, half: Vec3, direction_world: Vec3) -> Vec3 {
    let local = orientation.conjugate().rotate(direction_world);
    position
        + orientation.rotate(Vec3::new(
            support_component(local.x, half.x),
            support_component(local.y, half.y),
            support_component(local.z, half.z),
        ))
}

fn support_points(points: &[Vec3], direction: Vec3) -> Vec3 {
    points.iter().copied().max_by(|a, b| a.dot(direction).total_cmp(&b.dot(direction))).unwrap_or(Vec3::ZERO)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tilted_boxes_use_full_three_dimensional_sat() {
        let half = Vec3::new(1.0, 1.0, 1.0);
        let tilted = Quat::from_axis_angle(Vec3::X, core::f64::consts::FRAC_PI_4);
        let contact = oriented_box_contact(Vec3::ZERO, tilted, half, Vec3::new(0.0, 2.2, 0.0), Quat::IDENTITY, half)
            .expect("the tilted upper face extends beyond the unrotated half-height");
        assert!(contact.normal.y > 0.7);

        assert!(
            oriented_box_contact(Vec3::ZERO, tilted, half, Vec3::new(0.0, 2.5, 0.0), Quat::IDENTITY, half,).is_none()
        );
    }
}
