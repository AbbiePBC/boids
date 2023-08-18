use crate::{FrameDimensions, TIME_PER_FRAME};
use macroquad::prelude::*;
use std::ops::AddAssign;

fn clamp_position_to_stay_in_frame(co_ord: f32, max_in_direction: &f32) -> f32 {
    let mut current_distance_in_direction = co_ord;
    if current_distance_in_direction < 0.0 {
        current_distance_in_direction = 0.1;
    } else if current_distance_in_direction > *max_in_direction {
        current_distance_in_direction = max_in_direction - 0.1;
    }
    current_distance_in_direction
}

pub(crate) fn limit_speed(x_y_velocities: (f32, f32), max_boid_speed: f32) -> (f32, f32) {
    let x_vel = x_y_velocities.0;
    let y_vel = x_y_velocities.1;
    let speed = (x_vel.powi(2) + y_vel.powi(2)).sqrt();
    if speed < max_boid_speed {
        return (x_vel, y_vel);
    }
    let scaling_factor = max_boid_speed / speed;
    return (x_vel * &scaling_factor, y_vel * &scaling_factor);
}

// This reflects the boid if it will go out-of-bounds after the velocity is updated.
// It kind-of acts as an electric fence, so we don't necessarily bounce off the fence
// And how far before the boundary the boid gets reflected is dependent on the velocity
pub(crate) fn maybe_reflect_off_boundaries(
    boid_to_update: &Boid,
    dimensions: &FrameDimensions,
) -> (f32, f32) {
    // previous code assumed (0,0) was centre, but that's not the case.
    let mut new_x_vel = boid_to_update.x_y_velocities.0.clone();
    let mut new_y_vel = boid_to_update.x_y_velocities.1.clone();

    let projected_x_position: f32 = &boid_to_update.x_y_positions.0 + (new_x_vel * TIME_PER_FRAME);
    let projected_y_position: f32 = &boid_to_update.x_y_positions.1 + (new_y_vel * TIME_PER_FRAME);

    // update x velocity
    if projected_x_position >= dimensions.width || projected_x_position <= 0.0 {
        new_x_vel = &boid_to_update.x_y_velocities.0 * -1.0;
    }
    // update y velocity
    if projected_y_position >= dimensions.height || projected_y_position <= 0.0 {
        new_y_vel = &boid_to_update.x_y_velocities.1 * -1.0;
    }

    return (new_x_vel.to_owned(), new_y_vel.to_owned());
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct Boid {
    pub(crate) x_y_positions: (f32, f32),
    pub(crate) x_y_velocities: (f32, f32),
}

impl Boid {
    pub(crate) fn new(x_pos: f32, y_pos: f32, x_vel: f32, y_vel: f32) -> Boid {
        return Boid {
            x_y_positions: (x_pos, y_pos),
            x_y_velocities: (x_vel, y_vel),
        };
    }

    pub(crate) fn is_crowded_by_boid(
        self,
        other_boid: &Boid,
        max_dist_before_boid_is_no_longer_crowded: &f32,
    ) -> bool {
        return (self.x_y_positions.0 - &other_boid.x_y_positions.0).abs() < *max_dist_before_boid_is_no_longer_crowded
            && (self.x_y_positions.1 - &other_boid.x_y_positions.1).abs() < *max_dist_before_boid_is_no_longer_crowded;
    }

    pub(crate) fn is_within_sight_of_local_boid(
        self,
        other_boid: &Boid,
        max_dist_of_local_boid: &f32,
    ) -> bool {
        return (self.x_y_positions.0 - &other_boid.x_y_positions.0).abs() < *max_dist_of_local_boid
            && (self.x_y_positions.1 - &other_boid.x_y_positions.1).abs() < *max_dist_of_local_boid;
    }

    pub(crate) fn align_boid(
        &self,
        num_local_boids: i32,
        total_x_y_local_velocities: (f32, f32),
        adhesion_factor: &f32,
    ) -> (f32, f32) {
        let average_x_vel: f32 = total_x_y_local_velocities.0 / num_local_boids.clone() as f32;
        let average_y_vel: f32 = total_x_y_local_velocities.1 / num_local_boids.clone() as f32;
        // update the boid's velocity to move towards the average velocity of the local flock, by some adhesion factor
        return (
            &self.x_y_velocities.0 + ((average_x_vel - &self.x_y_velocities.0 ) * adhesion_factor) / TIME_PER_FRAME,
            &self.x_y_velocities.1 + ((average_y_vel - &self.x_y_velocities.1) * adhesion_factor) / TIME_PER_FRAME,
        );
    }

    pub(crate) fn uncrowd_boid(
        &self,
        num_crowding_boids: i32,
        total_x_y_dist_of_crowding_boids: (f32, f32),
        repulsion_factor: &f32,
    ) -> (f32, f32) {
        // move away from the average position of the crowding boids
        let dist_to_ave_x_pos_of_crowding_boids: f32 =
            &self.x_y_positions.0 - (total_x_y_dist_of_crowding_boids.0 / num_crowding_boids as f32);
        let dist_to_ave_y_pos_of_crowding_boids: f32 = &self.x_y_positions.1
            - (total_x_y_dist_of_crowding_boids.1 as f32 / num_crowding_boids.clone() as f32);

        // update velocity to move away from the average boid position within the crowding flock
        return (
            &self.x_y_velocities.0 + (dist_to_ave_x_pos_of_crowding_boids * repulsion_factor) / TIME_PER_FRAME,
            &self.x_y_velocities.1 + (dist_to_ave_y_pos_of_crowding_boids * repulsion_factor) / TIME_PER_FRAME,
        );
    }

    pub(crate) fn cohere_boid(
        &self,
        num_local_boids: i32,
        total_x_y_dist_of_local_boids: (f32, f32),
        cohesion_factor: &f32,
    ) -> (f32, f32) {
        // move towards the ave position of the local flock, so this is the reverse of uncrowding
        let dist_to_ave_x_pos_of_local_boids: f32 =
            (total_x_y_dist_of_local_boids.0 / num_local_boids as f32) - &self.x_y_positions.0;
        let dist_to_ave_y_pos_of_local_boids: f32 =
            (total_x_y_dist_of_local_boids.1 / num_local_boids.clone() as f32) - &self.x_y_positions.1;

        // update the boid's position to move towards the average position of the local flock, by some cohesion factor

        return (
            &self.x_y_velocities.0 + (dist_to_ave_x_pos_of_local_boids * cohesion_factor) / TIME_PER_FRAME,
            &self.x_y_velocities.1 + (dist_to_ave_y_pos_of_local_boids * cohesion_factor) / TIME_PER_FRAME,
        );
    }

    pub(crate) fn move_boid(&self, frame_dimensions: &FrameDimensions) -> Boid {
        // d=tv
        let mut new_x_pos = self.x_y_positions.0.clone() + (self.x_y_velocities.0.clone() * TIME_PER_FRAME);
        let mut new_y_pos = self.x_y_positions.1.clone() + (self.x_y_velocities.1.clone() * TIME_PER_FRAME);
        new_x_pos = clamp_position_to_stay_in_frame(new_x_pos, &frame_dimensions.width);
        new_y_pos = clamp_position_to_stay_in_frame(new_y_pos, &frame_dimensions.height);
        return Boid {
            x_y_positions: (new_x_pos, new_y_pos),
            ..self.clone()
        };
    }
}

impl AddAssign for Boid {
    fn add_assign(&mut self, other: Self) {
        self.x_y_positions.0 += other.x_y_positions.0;
        self.x_y_positions.1 += other.x_y_positions.1;
        self.x_y_velocities.0 += other.x_y_velocities.0;
        self.x_y_velocities.1 += other.x_y_velocities.1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adhesion() {
        let adhesion_factor = 1.0;
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);

        let (new_x_vel, new_y_vel) = Boid::align_boid(&boid, 2, (20.0, 0.0), &adhesion_factor);
        assert_eq!(new_x_vel, 10.0);
        assert_eq!(new_y_vel, 0.0);
    }

    #[test]
    fn test_no_adhesion() {
        let adhesion_factor = 0.0;
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);

        let (new_x_vel, new_y_vel) = Boid::align_boid(&boid, 2, (20.0, 0.0), &adhesion_factor);
        assert_eq!(new_x_vel, 1.0);
        assert_eq!(new_y_vel, 5.0);
    }

    #[test]
    fn test_half_adhesion() {
        let adhesion_factor = 0.5;
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);

        let (new_x_vel, new_y_vel) = Boid::align_boid(&boid, 2, (20.0, 0.0), &adhesion_factor);
        assert_eq!(new_x_vel, 5.5);
        assert_eq!(new_y_vel, 2.5);
    }

    #[test]
    fn test_boundary_reflected_when_velocity_is_zero() {
        // boid would escape boundary on next frame
        // but the velocity is such that reflecting the boid will not help
        let boid_to_update = Boid::new(1.0, 0.0, -2.0, 0.0);
        let dimensions = FrameDimensions {
            width: 1000.0,
            height: 1000.0,
        };
        let updated_boid_velocities = maybe_reflect_off_boundaries(&boid_to_update, &dimensions);
        assert!(updated_boid_velocities.0 > 0.0);
    }
    #[test]
    fn test_boundary_reflected_when_boid_at_boundary() {
        let dimensions = FrameDimensions {
            width: 1000.0,
            height: 1000.0,
        };

        let boid_to_update = Boid::new(dimensions.width, 0.0, 100.0, 0.0);
        let updated_boid_velocities = maybe_reflect_off_boundaries(&boid_to_update, &dimensions);
        assert!(updated_boid_velocities.0 < 0.0);
    }

    #[test]
    fn test_crowded_boid_has_updated_velocity() {
        let mut boid = Boid::new(1.0, 1.0, 1.0, 1.0);
        let other_boid = Boid::new(10.0, 10.0, 1.0, 5.0);

        let repulsion_factor = 0.0;
        let (new_x_vel, new_y_vel) = Boid::uncrowd_boid(
            &mut boid,
            1,
            other_boid.x_y_positions,
            &repulsion_factor,
        );

        assert_eq!(new_x_vel, boid.x_y_velocities.0);
        assert_eq!(new_y_vel, boid.x_y_velocities.1);
    }
}
