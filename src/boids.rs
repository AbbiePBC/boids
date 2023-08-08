use macroquad::prelude::*;
use std::error;
use std::fmt;
use std::ops::AddAssign;

fn clamp_position_to_stay_in_frame(co_ord: f32, max_in_direction: f32) -> f32 {
    let mut current_distance_in_direction = co_ord;
    if current_distance_in_direction < 0.0 {
        current_distance_in_direction = 0.1;
    } else if current_distance_in_direction > max_in_direction {
        current_distance_in_direction = max_in_direction - 0.1;
    }
    current_distance_in_direction
}

// TODO: separate this out for ease of testing
// TODO: currently there are still boids that escape the frame, so this should be tested comprehensively
// maybe_reflect_off_boundaries(x_pos, x_vel, y_pos, y_vel, frame_width, frame_height)

pub(crate) fn maybe_reflect_off_boundaries(
    mut boid_to_update: Boid,
    frame_width: f32,
    frame_height: f32,
    time_per_frame: f32,
) -> Boid {
    // previous code assumed (0,0) was centre, but that's not the case.
    // note that 5 is used here as it is the diameter of the boid
    // which really should be a constant somewhere
    if boid_to_update.x_pos >= (frame_width - 5.0) || boid_to_update.x_pos <= 5.0 {
        boid_to_update.x_vel = -boid_to_update.x_vel;
    }
    if boid_to_update.y_pos >= (frame_height - 5.0) || boid_to_update.y_pos <= 5.0 {
        boid_to_update.y_vel = -boid_to_update.y_vel;
    }
    let new_x_pos: f32 = boid_to_update.x_pos + (boid_to_update.x_vel * time_per_frame as f32);
    let new_y_pos: f32 = boid_to_update.y_pos + (boid_to_update.y_vel * time_per_frame as f32);

    return Boid {
        x_vel: boid_to_update.x_vel,
        y_vel: boid_to_update.y_vel,
        x_pos: clamp_position_to_stay_in_frame(new_x_pos, frame_width),
        y_pos: clamp_position_to_stay_in_frame(new_y_pos, frame_height),
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
        &self,
        other_boid: &Boid,
        max_dist_before_boid_is_no_longer_crowded: f32,
    ) -> bool {
        return (self.x_pos - other_boid.x_pos).abs() < max_dist_before_boid_is_no_longer_crowded
            && (self.y_pos - other_boid.y_pos).abs() < max_dist_before_boid_is_no_longer_crowded;
    }

    pub(crate) fn is_within_sight_of_local_boid(
        &self,
        other_boid: &Boid,
        max_dist_of_local_boid: f32,
    ) -> bool {
        return (self.x_pos - other_boid.x_pos).abs() < max_dist_of_local_boid
            && (self.y_pos - other_boid.y_pos).abs() < max_dist_of_local_boid;
    }

    pub(crate) fn align_boid(
        &self,
        num_local_boids: i32,
        total_x_vel_of_local_boids: f32,
        total_y_vel_of_local_boids: f32,
        adhesion_factor: f32,
        time_per_frame: f32,
    ) -> Boid {
        let average_x_vel: f32 = total_x_vel_of_local_boids as f32 / num_local_boids as f32;
        let average_y_vel: f32 = total_y_vel_of_local_boids as f32 / num_local_boids as f32;
        // update the boid's velocity to move towards the average velocity of the local flock, by some adhesion factor
        return Boid {
            x_vel: self.x_vel + ((average_x_vel - self.x_vel) * adhesion_factor),
            y_vel: self.y_vel + ((average_y_vel - self.y_vel) * adhesion_factor),
            x_pos: self.x_pos + (self.x_vel * time_per_frame as f32),
            y_pos: self.y_pos + (self.y_vel * time_per_frame as f32),
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

        let updated_boid = Boid::align_boid(&boid, 2, 20.0, 0.0, adhesion_factor, 1.0);
        assert_eq!(updated_boid.x_vel, 10.0);
        assert_eq!(updated_boid.y_vel, 0.0);
    }

    #[test]
    fn test_no_adhesion() {
        let adhesion_factor = 0.0;
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);

        let updated_boid = Boid::align_boid(&boid, 2, 20.0, 0.0, adhesion_factor, 1.0);
        assert_eq!(updated_boid.x_vel, 1.0);
        assert_eq!(updated_boid.y_vel, 5.0);
    }

    #[test]
    fn test_half_adhesion() {
        let adhesion_factor = 0.5;
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);

        let updated_boid = Boid::align_boid(&boid, 2, 20.0, 0.0, adhesion_factor, 1.0);
        assert_eq!(updated_boid.x_vel, 5.5);
        assert_eq!(updated_boid.y_vel, 2.5);
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
        let max_x_position = 1000.0;
        let max_y_position = 1000.0;
        let updated_boid =
            maybe_reflect_off_boundaries(boid_to_update, max_x_position, max_y_position, 1.0);
        assert!(updated_boid.x_pos > 0.0);
    }
    #[test]
    fn test_boundary_reflected_when_boid_at_boundary() {
        let max_x_position = 1000.0;
        let max_y_position = 1000.0;
        let boid_to_update = Boid {
            x_pos: max_x_position,
            y_pos: 0.0,
            x_vel: 100000.0,
            y_vel: 0.0,
        };
        let updated_boid =
            maybe_reflect_off_boundaries(boid_to_update, max_x_position, max_y_position, 1.0);
        assert!(updated_boid.x_pos > 0.0);
    }
}
