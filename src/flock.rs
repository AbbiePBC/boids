use macroquad::prelude::*;
extern crate rand;
use crate::boids::{Boid, limit_speed, maybe_reflect_off_boundaries};
use crate::validate::{validate_distances, validate_factors, InvalidFlockConfig};
use crate::TIME_PER_FRAME;
use rand::{thread_rng, Rng};

#[derive(Debug)]
/// A struct that represents the dimensions of the frame in which the boids are moving.
// probably excessive but oh well
pub(crate) struct FrameDimensions {
    pub(crate) frame_width: f32,
    pub(crate) frame_height: f32,
}

#[derive(Debug)]
pub(crate) struct Flock {
    pub(crate) flock_size: usize,
    pub(crate) boids: Vec<Boid>,
    max_dist_before_boid_is_no_longer_crowded: f32,
    max_dist_of_local_boid: f32, // i.e. the radius of the local flock; far boids in the flock don't influence a boid's behaviour
    repulsion_factor: f32,       // how much a boid wants to move away from other boids
    adhesion_factor: f32,        // how much a boid wants to stay with the flock
    cohesion_factor: f32, // how much a boid wants to move towards the average position of the flock
    boid_max_speed: f32,
}

impl Flock {
    /// the pattern of Flock{}, flock.validate()?, flock.init() is used to avoid
    /// initializing the list of boids if the input is invalid,
    /// required by making "validate()" a method
    pub(crate) fn new(
        flock_size: usize,
        max_dist_before_boid_is_crowded: f32,
        max_dist_of_local_boid: f32,
        repulsion_factor: f32,
        adhesion_factor: f32,
        cohesion_factor: f32,
    ) -> Result<Flock, InvalidFlockConfig> {
        let mut flock = Flock {
            flock_size,
            boids: Vec::new(),
            max_dist_before_boid_is_no_longer_crowded: max_dist_before_boid_is_crowded,
            max_dist_of_local_boid,
            repulsion_factor,
            adhesion_factor,
            cohesion_factor,
            boid_max_speed: 8.0,
        };
        let _ = flock.validate()?;
        flock.init();
        Ok(flock)
    }

    fn validate(&self) -> Result<(), InvalidFlockConfig> {
        let mut errors = validate_factors(
            self.repulsion_factor.clone(),
            self.adhesion_factor.clone(),
            self.cohesion_factor.clone(),
        );

        if let Some(creation_error) = validate_distances(
            &self.max_dist_before_boid_is_no_longer_crowded,
            &self.max_dist_of_local_boid,
        ) {
            errors.push(creation_error);
        }

        if errors.len() > 0 {
            return Err(InvalidFlockConfig { errors });
            // the "into()" will use the From trait to convert the InvalidFlockConfig into an Error
        }

        return Ok(());
    }

    /// Not necessary to split this out for a single fn call
    /// But done to show how initialisation can be done in a separate function
    fn init(&mut self) {
        self.boids = self.generate_boids();
    }

    fn generate_boids(&self) -> Vec<Boid> {
        let mut boids = Vec::new();
        for _ in 0..self.flock_size {
            boids.push(Boid::new(0.0, 0.0, 0.0, 0.0));
        }
        return boids;
    }
    pub(crate) fn randomly_generate_boids(&mut self, dimensions: &FrameDimensions) {
        let mut rng = thread_rng();
        let mut boids = Vec::new();
        let mid_frame_x = &dimensions.frame_width / 2.0;
        let mid_frame_y = &dimensions.frame_height / 2.0;
        let max_starting_dist_from_mid_x = &dimensions.frame_width / 10.0;
        let max_starting_dist_from_mid_y = &dimensions.frame_height / 10.0;
        for _ in 0..self.flock_size {
            boids.push(Boid::new(
                &mid_frame_x
                    + rng.gen_range(-&max_starting_dist_from_mid_x..max_starting_dist_from_mid_x),
                &mid_frame_y
                    + rng.gen_range(-&max_starting_dist_from_mid_y..max_starting_dist_from_mid_y),
                rng.gen_range(-self.boid_max_speed.clone()..self.boid_max_speed.clone()),
                rng.gen_range(-self.boid_max_speed.clone()..self.boid_max_speed.clone()),
            ));
        }

        self.boids = boids;
    }

    // todo: on reflection, this should probably be
    // Boids::update_boids(&self, boid_idx, flock, dimensions: &FrameDimensions)
    // -> Boid()
    pub(crate) fn update_boid(&mut self, boid_to_update: usize, dimensions: &FrameDimensions) {
        let mut total_x_dist_of_crowding_boids: f32 = 0.0;
        let mut total_y_dist_of_crowding_boids: f32 = 0.0;
        let mut num_crowding_boids: i32 = 0;

        let mut total_of_local_boids: Boid = Boid::new(0.0, 0.0, 0.0, 0.0);
        let mut num_local_boids: i32 = 0;

        let mut boid_idx = 0;
        for other_boid in &self.boids {
            if boid_idx == boid_to_update {
                boid_idx += 1;
                continue;
            }
            boid_idx += 1;
            if self.boids[boid_to_update.clone()]
                .is_crowded_by_boid(other_boid, &self.max_dist_before_boid_is_no_longer_crowded)
            {
                num_crowding_boids += 1;
                total_x_dist_of_crowding_boids += &other_boid.x_pos;
                total_y_dist_of_crowding_boids += &other_boid.y_pos;
            } else if self.boids[boid_to_update.clone()]
                .is_within_sight_of_local_boid(&other_boid, &self.max_dist_of_local_boid)
            {
                num_local_boids += 1;
                total_of_local_boids += other_boid.clone();
            }
            // else, the other_boid is too far away to affect the boid we're updating
        }

        // todo: these functions should affect the boid velocity and not position,
        // and then the boid position should be updated later
        // once the boid velocity is capped at the maximum
        if num_crowding_boids > 0 {
            self.boids[boid_to_update] = Boid::uncrowd_boid(
                &self.boids[boid_to_update.clone()].clone(),
                num_crowding_boids,
                total_x_dist_of_crowding_boids,
                total_y_dist_of_crowding_boids,
                &self.repulsion_factor,
            );
        }
        if num_local_boids > 0 {
            self.boids[boid_to_update.clone()] = Boid::align_boid(
                &self.boids[boid_to_update.clone()],
                num_local_boids,
                total_of_local_boids.x_vel,
                total_of_local_boids.y_vel,
                &self.adhesion_factor,
            );
            self.boids[boid_to_update.clone()] = Boid::cohere_boid(
                &self.boids[boid_to_update.clone()],
                num_local_boids.clone(),
                total_of_local_boids.x_pos,
                total_of_local_boids.y_pos,
                &self.cohesion_factor,
            );
        }
        if num_local_boids == 0 && num_crowding_boids == 0 {
            self.boids[boid_to_update.clone()].continue_moving_as_unaffected_by_other_boids();
        }

        self.boids[boid_to_update.clone()] = Boid::move_boid(&self.boids[boid_to_update.clone()]);
        // todo: without the above function, the boids do not move
        // in theory this is fine, bc we're updating vel not position as we go
        // but check this is the case rather than a bug somewhere
        self.boids[boid_to_update.clone()] =
            maybe_reflect_off_boundaries(&self.boids[boid_to_update.clone()], &dimensions);

    }

}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_no_crowding_by_boid_outside_of_crowding_zone() {
        let mut flock = Flock::new(0, 4.0, 5.0, 0.0, 0.0, 0.0).unwrap();
        let boid = Boid::new(1.0, 1.0, 1.0, 1.0);
        let other_boid = Boid::new(10.0, 10.0, 2.0, 2.0);
        flock.boids = vec![boid, other_boid];

        assert!(!flock.boids[0].is_crowded_by_boid(
            &flock.boids[1],
            &flock.max_dist_before_boid_is_no_longer_crowded
        ));
        assert!(!flock.boids[1].is_crowded_by_boid(
            &flock.boids[0],
            &flock.max_dist_before_boid_is_no_longer_crowded
        ));
    }

    #[test]
    fn test_crowding_by_boid_inside_of_crowding_zone() {
        let mut flock = Flock::new(0, 40.0, 500.0, 0.0, 0.0, 0.0).unwrap();
        let boid = Boid::new(1.0, 1.0, 1.0, 1.0);
        let other_boid = Boid::new(10.0, 10.0, 2.0, 2.0);
        flock.boids = vec![boid, other_boid];

        assert!(flock.boids[0].is_crowded_by_boid(
            &flock.boids[1],
            &flock.max_dist_before_boid_is_no_longer_crowded
        ));
        assert!(flock.boids[1].is_crowded_by_boid(
            &flock.boids[0],
            &flock.max_dist_before_boid_is_no_longer_crowded
        ));
    }

    #[test]
    fn test_boid_outside_of_local_zone() {
        let mut flock = Flock::new(0, 1.0, 50.0, 0.0, 0.0, 0.0).unwrap();
        let boid = Boid::new(1.0, 1.0, 1.0, 1.0);
        let other_boid = Boid::new(10.0, 10.0, 2.0, 2.0);
        flock.boids = vec![boid, other_boid];

        // not crowded
        assert!(!flock.boids[0].is_crowded_by_boid(
            &flock.boids[1],
            &flock.max_dist_before_boid_is_no_longer_crowded
        ));
        assert!(!flock.boids[1].is_crowded_by_boid(
            &flock.boids[0],
            &flock.max_dist_before_boid_is_no_longer_crowded
        ));

        // but still within local zone
        assert!(flock.boids[0]
            .is_within_sight_of_local_boid(&flock.boids[1], &flock.max_dist_of_local_boid.clone()));
        assert!(flock.boids[1]
            .is_within_sight_of_local_boid(&flock.boids[0], &flock.max_dist_of_local_boid.clone()));
    }

    #[test]
    fn test_flock_validated_correctly() {
        // flock has invalid distances
        let flock = Flock::new(0, 20.0, 2.0, 0.0, 0.2, 1.0);
        assert!(flock.is_err());

        // flock has invalid factors
        let flock = Flock::new(0, 1.0, 50.0, 2.0, -20.2, 1.0);
        assert!(flock.is_err());
    }

    #[test]
    fn test_all_creation_errors_reported() {
        let result = Flock::new(0, 2.0, -4.9, 3.0, 20.0, 2.0);
        assert!(result.is_err());
        let error = result.unwrap_err();
        assert_eq!(error.errors.len(), 4);
    }
}
