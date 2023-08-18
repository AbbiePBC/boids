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

pub(crate) fn limit_speed(x_vel: f32, y_vel: f32, max_boid_speed: f32) -> (f32, f32) {
    let speed = (x_vel.powi(2) + y_vel.powi(2)).sqrt();
    if speed < max_boid_speed {
        return (x_vel, y_vel);
    }
    let scaling_factor = max_boid_speed / speed;
    return (x_vel * &scaling_factor, y_vel * &scaling_factor);
}

pub(crate) fn maybe_reflect_off_boundaries(
    boid_to_update: &Boid,
    dimensions: &FrameDimensions,
) -> Boid {
    // previous code assumed (0,0) was centre, but that's not the case.
    let mut new_x_vel = boid_to_update.x_vel.clone();
    let mut new_y_vel = boid_to_update.y_vel.clone();

    if boid_to_update.x_pos >= dimensions.frame_width || boid_to_update.x_pos <= 0.0 {
        new_x_vel = &boid_to_update.x_vel * -1.0;
    }
    if boid_to_update.y_pos >= dimensions.frame_height || boid_to_update.y_pos <= 0.0 {
        new_y_vel = &boid_to_update.y_vel * -1.0;
    }
    let new_x_pos: f32 = &boid_to_update.x_pos + (new_x_vel * TIME_PER_FRAME);
    let new_y_pos: f32 = &boid_to_update.y_pos + (new_y_vel * TIME_PER_FRAME);

    return Boid {
        x_vel: new_x_vel.to_owned(),
        y_vel: new_y_vel.to_owned(),
        x_pos: clamp_position_to_stay_in_frame(new_x_pos, &dimensions.frame_width),
        y_pos: clamp_position_to_stay_in_frame(new_y_pos, &dimensions.frame_height),
    };
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct Boid {
    pub(crate) x_pos: f32,
    pub(crate) y_pos: f32,
    pub(crate) x_vel: f32,
    pub(crate) y_vel: f32,
}

impl Boid {
    pub(crate) fn new(x_pos: f32, y_pos: f32, x_vel: f32, y_vel: f32) -> Boid {
        return Boid {
            x_pos,
            y_pos,
            x_vel,
            y_vel,
        };
    }

    pub(crate) fn is_crowded_by_boid(
        self,
        other_boid: &Boid,
        max_dist_before_boid_is_no_longer_crowded: &f32,
    ) -> bool {
        return (self.x_pos - &other_boid.x_pos).abs() < *max_dist_before_boid_is_no_longer_crowded
            && (self.y_pos - &other_boid.y_pos).abs() < *max_dist_before_boid_is_no_longer_crowded;
    }

    pub(crate) fn is_within_sight_of_local_boid(
        self,
        other_boid: &Boid,
        max_dist_of_local_boid: &f32,
    ) -> bool {
        return (self.x_pos - &other_boid.x_pos).abs() < *max_dist_of_local_boid
            && (self.y_pos - &other_boid.y_pos).abs() < *max_dist_of_local_boid;
    }

    pub(crate) fn align_boid(
        &self,
        num_local_boids: i32,
        total_x_vel_of_local_boids: f32,
        total_y_vel_of_local_boids: f32,
        adhesion_factor: &f32,
    ) -> (f32, f32) {
        let average_x_vel: f32 = total_x_vel_of_local_boids / num_local_boids.clone() as f32;
        let average_y_vel: f32 = total_y_vel_of_local_boids / num_local_boids.clone() as f32;
        // update the boid's velocity to move towards the average velocity of the local flock, by some adhesion factor
        return (
            &self.x_vel + ((average_x_vel - &self.x_vel) * adhesion_factor) / TIME_PER_FRAME,
            &self.y_vel + ((average_y_vel - &self.y_vel) * adhesion_factor) / TIME_PER_FRAME,
        );
    }

    pub(crate) fn uncrowd_boid(
        &self,
        num_crowding_boids: i32,
        total_x_dist_of_crowding_boids: f32,
        total_y_dist_of_crowding_boids: f32,
        repulsion_factor: &f32,
    ) -> (f32, f32) {
        // move away from the average position of the crowding boids
        let dist_to_ave_x_pos_of_crowding_boids: f32 =
            &self.x_pos - (total_x_dist_of_crowding_boids / num_crowding_boids as f32);
        let dist_to_ave_y_pos_of_crowding_boids: f32 = &self.y_pos
            - (total_y_dist_of_crowding_boids as f32 / num_crowding_boids.clone() as f32);

        // update velocity to move away from the average boid position within the crowding flock
        return (
            &self.x_vel + (dist_to_ave_x_pos_of_crowding_boids * repulsion_factor) / TIME_PER_FRAME,
            &self.y_vel + (dist_to_ave_y_pos_of_crowding_boids * repulsion_factor) / TIME_PER_FRAME,
        );
    }

    pub(crate) fn cohere_boid(
        &self,
        num_local_boids: i32,
        total_x_dist_of_local_boids: f32,
        total_y_dist_of_local_boids: f32,
        cohesion_factor: &f32,
    ) -> (f32, f32) {
        // move towards the ave position of the local flock, so this is the reverse of uncrowding
        let dist_to_ave_x_pos_of_local_boids: f32 =
            (total_x_dist_of_local_boids / num_local_boids as f32) - &self.x_pos;
        let dist_to_ave_y_pos_of_local_boids: f32 =
            (total_y_dist_of_local_boids / num_local_boids.clone() as f32) - &self.y_pos;

        // update the boid's position to move towards the average position of the local flock, by some cohesion factor

        return (
            &self.x_vel + (dist_to_ave_x_pos_of_local_boids * cohesion_factor) / TIME_PER_FRAME,
            &self.y_vel + (dist_to_ave_y_pos_of_local_boids * cohesion_factor) / TIME_PER_FRAME,
        );
    }

    pub(crate) fn move_boid(&self) -> Boid {
        // d=tv
        let new_x_pos = self.x_pos.clone() + (self.x_vel.clone() * TIME_PER_FRAME);
        let new_y_pos = self.y_pos.clone() + (self.y_vel.clone() * TIME_PER_FRAME);
        return Boid {
            x_pos: new_x_pos,
            y_pos: new_y_pos,
            ..self.clone()
        };
    }
}

impl AddAssign for Boid {
    fn add_assign(&mut self, other: Self) {
        self.x_pos += other.x_pos;
        self.y_pos += other.y_pos;
        self.x_vel += other.x_vel;
        self.y_vel += other.y_vel;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adhesion() {
        let adhesion_factor = 1.0;
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);

        let (new_x_vel, new_y_vel) = Boid::align_boid(&boid, 2, 20.0, 0.0, &adhesion_factor);
        assert_eq!(new_x_vel, 10.0);
        assert_eq!(new_y_vel, 0.0);
    }

    #[test]
    fn test_no_adhesion() {
        let adhesion_factor = 0.0;
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);

        let (new_x_vel, new_y_vel) = Boid::align_boid(&boid, 2, 20.0, 0.0, &adhesion_factor);
        assert_eq!(new_x_vel, 1.0);
        assert_eq!(new_y_vel, 5.0);
    }

    #[test]
    fn test_half_adhesion() {
        let adhesion_factor = 0.5;
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);

        let (new_x_vel, new_y_vel) = Boid::align_boid(&boid, 2, 20.0, 0.0, &adhesion_factor);
        assert_eq!(new_x_vel, 5.5);
        assert_eq!(new_y_vel, 2.5);
    }

    #[test]
    fn test_boundary_reflected_when_velocity_is_zero() {
        // boid is past the boundary
        // but the velocity is such that reflecting the boid will not help
        let boid_to_update = Boid {
            x_pos: -1.0,
            y_pos: 0.0,
            x_vel: 0.0,
            y_vel: 0.0,
        };
        let dimensions = FrameDimensions {
            frame_width: 1000.0,
            frame_height: 1000.0,
        };
        let updated_boid = maybe_reflect_off_boundaries(&boid_to_update, &dimensions);
        assert!(updated_boid.x_pos > 0.0);
    }
    #[test]
    fn test_boundary_reflected_when_boid_at_boundary() {
        let dimensions = FrameDimensions {
            frame_width: 1000.0,
            frame_height: 1000.0,
        };

        let boid_to_update = Boid {
            x_pos: dimensions.frame_width,
            y_pos: 0.0,
            x_vel: 100000.0,
            y_vel: 0.0,
        };
        let updated_boid = maybe_reflect_off_boundaries(&boid_to_update, &dimensions);
        assert!(updated_boid.x_pos > 0.0);
    }

    #[test]
    fn test_crowded_boid_has_updated_velocity() {
        let mut boid = Boid::new(1.0, 1.0, 1.0, 1.0);
        let other_boid = Boid::new(10.0, 10.0, 1.0, 5.0);

        let repulsion_factor = 0.0;
        let (new_x_vel, new_y_vel) = Boid::uncrowd_boid(
            &mut boid,
            1,
            other_boid.x_pos,
            other_boid.y_pos,
            &repulsion_factor,
        );

        assert_eq!(new_x_vel, boid.x_vel);
        assert_eq!(new_y_vel, boid.y_vel);
    }
}
