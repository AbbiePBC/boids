use anyhow::{anyhow, Error, Result};
use macroquad::prelude::*;
use std::error;
use std::fmt;
extern crate rand;
use crate::boids::maybe_reflect_off_boundaries;
use crate::boids::Boid;
use rand::{thread_rng, Rng};

#[derive(Debug)]
// todo: some of these inputs aren't about the flock so should be extracted
// e.g. time per frame, frame dimensions
// and almost all of the methods don't need flock info but boid only.
pub(crate) struct Flock {
    pub(crate) flock_size: usize,
    pub(crate) boids: Vec<Boid>,
    max_dist_before_boid_is_no_longer_crowded: f32,
    max_dist_of_local_boid: f32, // i.e. the radius of the local flock; far boids in the flock don't influence a boid's behaviour
    repulsion_factor: f32,       // how much a boid wants to move away from other boids
    adhesion_factor: f32,        // how much a boid wants to stay with the flock
    cohesion_factor: f32, // how much a boid wants to move towards the average position of the flock
    time_per_frame: f32,
    boid_max_speed: f32,
    pub(crate) frame_width: f32,
    pub(crate) frame_height: f32,
}

#[derive(PartialEq, Debug)]
enum CreationError {
    FactorShouldBeMoreThanZero(String),
    FactorShouldBeLessThanOne(String),
    LocalEnvironmentIsSmallerThanCrowdingEnvironment,
}

// This is required so that `CreationError` can implement `error::Error`.
impl fmt::Display for CreationError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let description = match self {
            CreationError::FactorShouldBeMoreThanZero(factor_name) => {
                factor_name.to_owned() + " factor is negative"
            }
            CreationError::FactorShouldBeLessThanOne(factor_name) => {
                factor_name.to_owned() + " factor is too large and should be below zero"
            }
            CreationError::LocalEnvironmentIsSmallerThanCrowdingEnvironment => {
                "local environment is smaller than (or equal to) crowding environment".to_owned()
            }
        };
        f.write_str(&description)
    }
}

impl error::Error for CreationError {}

fn check_float_between_zero_and_one(value: f32, name: String) -> Option<CreationError> {
    match value {
        x if x < 0.0 => Some(CreationError::FactorShouldBeMoreThanZero(name)),
        x if x > 1.0 => Some(CreationError::FactorShouldBeLessThanOne(name)),
        _ => None,
    }
}

fn validate_factors(
    repulsion_factor: f32,
    adhesion_factor: f32,
    cohesion_factor: f32,
) -> Vec<CreationError> {
    let repulsion = check_float_between_zero_and_one(repulsion_factor, "repulsion".to_string());
    let adhesion = check_float_between_zero_and_one(adhesion_factor, "adhesion".to_string());
    let cohesion = check_float_between_zero_and_one(cohesion_factor, "cohesion".to_string());

    let mut errors: Vec<_> = [repulsion, adhesion, cohesion]
        .into_iter()
        .filter_map(|option| option)
        .collect();
    errors
}

fn validate_distances(
    max_dist_before_boid_is_crowded: f32,
    max_dist_of_local_boid: f32,
) -> Option<CreationError> {
    if max_dist_before_boid_is_crowded >= max_dist_of_local_boid {
        return Some(CreationError::LocalEnvironmentIsSmallerThanCrowdingEnvironment);
    }
    return None;
}

#[derive(Debug)]
pub(crate) struct InvalidFlockConfig {
    errors: Vec<CreationError>,
}

impl From<InvalidFlockConfig> for Error {
    fn from(invalid_flock_config: InvalidFlockConfig) -> Self {
        anyhow!("Invalid Flock input: {:?}", invalid_flock_config.errors)
    }
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
            time_per_frame: 1.0,
            boid_max_speed: 8.0,
            frame_width: 800.0,
            frame_height: 500.0,
        };
        let _ = flock.validate()?;
        flock.init();
        Ok(flock)
    }

    
    // todo: move to new file for input parsing and validation
    // as should the validations above
    fn validate(&self) -> Result<(), InvalidFlockConfig> {
        let mut errors = validate_factors(
            self.repulsion_factor,
            self.adhesion_factor,
            self.cohesion_factor,
        );

        if let Some(creation_error) = validate_distances(
            self.max_dist_before_boid_is_no_longer_crowded,
            self.max_dist_of_local_boid,
        ) {
            errors.push(creation_error);
        }

        if errors.len() > 0 {
            return Err(InvalidFlockConfig { errors }); // the "into()" will use the From trait to convert the InvalidFlockConfig into an Error
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
    pub(crate) fn randomly_generate_boids(&mut self) {
        let mut rng = thread_rng();
        let mut boids = Vec::new();
        let frame_width = self.frame_width.clone();
        let mid_frame_x = &frame_width / 2.0;
        let mid_frame_y = &frame_width / 2.0;
        let max_starting_dist_from_mid_x = &frame_width / 10.0;
        let max_starting_dist_from_mid_y = &frame_width / 10.0;
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

    
    // todo: move to Boid::.
    fn cohere_boid(
        &mut self,
        boid_to_update: usize,
        num_local_boids: i32,
        total_x_dist_of_local_boids: f32,
        total_y_dist_of_local_boids: f32,
    ) {
        // move towards the ave position of the local flock, so this is the reverse of uncrowding
        let dist_to_ave_x_pos_of_local_boids: f32 = (total_x_dist_of_local_boids as f32
            / num_local_boids as f32)
            - self.boids[boid_to_update].x_pos;
        let dist_to_ave_y_pos_of_local_boids: f32 = (total_y_dist_of_local_boids as f32
            / num_local_boids as f32)
            - self.boids[boid_to_update].y_pos;

        // update the boid's position to move towards the average position of the local flock, by some cohesion factor
        self.boids[boid_to_update] = Boid {
            x_vel: self.boids[boid_to_update].x_vel
                + (dist_to_ave_x_pos_of_local_boids * self.cohesion_factor)
                    / self.time_per_frame as f32,
            y_vel: self.boids[boid_to_update].y_vel
                + (dist_to_ave_y_pos_of_local_boids * self.cohesion_factor)
                    / self.time_per_frame as f32,
            x_pos: self.boids[boid_to_update].x_pos
                + (self.boids[boid_to_update].x_vel * self.time_per_frame as f32),
            y_pos: self.boids[boid_to_update].y_pos
                + (self.boids[boid_to_update].y_vel * self.time_per_frame as f32),
        }
    }
    pub(crate) fn update_boid(&mut self, boid_to_update: usize) {
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
            if self.boids[boid_to_update]
                .is_crowded_by_boid(&other_boid, self.max_dist_before_boid_is_no_longer_crowded)
            {
                num_crowding_boids += 1;
                total_x_dist_of_crowding_boids += other_boid.x_pos;
                total_y_dist_of_crowding_boids += other_boid.y_pos;
            } else if self.boids[boid_to_update]
                .is_within_sight_of_local_boid(&other_boid, self.max_dist_of_local_boid)
            {
                num_local_boids += 1;
                total_of_local_boids += *other_boid;
            }
            // else, the other_boid is too far away to affect the boid we're updating
        }

        if num_crowding_boids > 0 {
            self.boids[boid_to_update] = Boid::uncrowd_boid(
                self.boids[boid_to_update],
                num_crowding_boids,
                total_x_dist_of_crowding_boids,
                total_y_dist_of_crowding_boids,
                self.repulsion_factor,
                self.time_per_frame
            );
        }
        if num_local_boids > 0 {
            self.boids[boid_to_update] = Boid::align_boid(
                &self.boids[boid_to_update].clone(),
                num_local_boids,
                total_of_local_boids.x_vel,
                total_of_local_boids.y_vel,
                self.adhesion_factor,
                self.time_per_frame,
            );
            // todo: move to Boid::.
            Flock::cohere_boid(
                self,
                boid_to_update,
                num_local_boids,
                total_of_local_boids.x_pos,
                total_of_local_boids.y_pos,
            );
        }
        // todo: test the following
        if num_local_boids == 0 && num_crowding_boids == 0 {
            // boid is unaffected by other boids, continue moving in same direction
            // maybe a random direction would be more realistic idk
            self.boids[boid_to_update] = Boid {
                x_vel: self.boids[boid_to_update].x_vel,
                y_vel: self.boids[boid_to_update].y_vel,
                x_pos: self.boids[boid_to_update].x_pos
                    + (self.boids[boid_to_update].x_vel * self.time_per_frame as f32),
                y_pos: self.boids[boid_to_update].y_pos
                    + (self.boids[boid_to_update].y_vel * self.time_per_frame as f32),
            }
        }

        self.limit_speed(boid_to_update);
        self.boids[boid_to_update] = maybe_reflect_off_boundaries(
            self.boids[boid_to_update],
            self.frame_width,
            self.frame_height,
            self.time_per_frame,
        );
    }

    // todo: this is not really correct as doing this at the end allows instantaneous speed to be higher than max
    // so every time the velocity is updated, should be done as a max_absolute_value_of(new_velocity, max_speed)
    // well ... really the position shouldn't be continually updated, just the speed, which is then capped and the dist then calculated/
    fn limit_speed(&mut self, boid_to_update: usize) {
        let speed = (self.boids[boid_to_update.clone()].x_vel.powi(2)
            + self.boids[boid_to_update.clone()].y_vel.powi(2))
        .sqrt();
        if speed > self.boid_max_speed {
            self.boids[boid_to_update.clone()].x_vel =
                (self.boids[boid_to_update.clone()].x_vel / speed) * self.boid_max_speed;
            self.boids[boid_to_update.clone()].y_vel =
                (self.boids[boid_to_update.clone()].y_vel / speed) * self.boid_max_speed;
        }
        self.boids[boid_to_update.clone()] = Boid {
            x_vel: self.boids[boid_to_update.clone()].x_vel,
            y_vel: self.boids[boid_to_update.clone()].y_vel,
            x_pos: self.boids[boid_to_update.clone()].x_pos
                + (self.boids[boid_to_update.clone()].x_vel * self.time_per_frame as f32),
            y_pos: self.boids[boid_to_update.clone()].y_pos
                + (self.boids[boid_to_update.clone()].y_vel * self.time_per_frame as f32),
        }
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
            flock.max_dist_before_boid_is_no_longer_crowded
        ));
        assert!(!flock.boids[1].is_crowded_by_boid(
            &flock.boids[0],
            flock.max_dist_before_boid_is_no_longer_crowded
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
            flock.max_dist_before_boid_is_no_longer_crowded
        ));
        assert!(flock.boids[1].is_crowded_by_boid(
            &flock.boids[0],
            flock.max_dist_before_boid_is_no_longer_crowded
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
            flock.max_dist_before_boid_is_no_longer_crowded
        ));
        assert!(!flock.boids[1].is_crowded_by_boid(
            &flock.boids[0],
            flock.max_dist_before_boid_is_no_longer_crowded
        ));

        // but still within local zone
        assert!(flock.boids[0]
            .is_within_sight_of_local_boid(&flock.boids[1], flock.max_dist_of_local_boid));
        assert!(flock.boids[1]
            .is_within_sight_of_local_boid(&flock.boids[0], flock.max_dist_of_local_boid));
    }

    #[test]
    fn test_incorrect_factor_inputs() {
        let flock = Flock::new(0, 1.0, 50.0, 2.0, -20.2, 1.0);
        assert!(flock.is_err());

        let result = validate_factors(2.0, -4.9, 1.0);
        let expected_errors = vec![
            CreationError::FactorShouldBeLessThanOne("repulsion".to_string()),
            CreationError::FactorShouldBeMoreThanZero("adhesion".to_string()),
        ];
        assert_eq!(result, expected_errors);
    }
    #[test]
    fn test_incorrect_distance_inputs() {
        let flock = Flock::new(0, 20.0, 2.0, 2.0, -20.2, 1.0);
        assert!(flock.is_err());

        let result = validate_distances(20.0, 2.0);
        assert_eq!(
            result,
            Some(CreationError::LocalEnvironmentIsSmallerThanCrowdingEnvironment)
        );
    }
    #[test]
    fn test_all_creation_errors_reported() {
        let result = Flock::new(0, 2.0, -4.9, 3.0, 20.0, 2.0);
        assert!(result.is_err());
        let error = result.unwrap_err();
        assert_eq!(error.errors.len(), 4);
    }
    #[test]
    fn test_error_display() {
        assert_eq!(
            CreationError::FactorShouldBeLessThanOne("adhesion".to_string()).to_string(),
            "adhesion factor is too large and should be below zero".to_string()
        );
        assert_eq!(
            CreationError::FactorShouldBeMoreThanZero("repulsion".to_string()).to_string(),
            "repulsion factor is negative".to_string()
        );
        assert_eq!(
            CreationError::LocalEnvironmentIsSmallerThanCrowdingEnvironment.to_string(),
            "local environment is smaller than (or equal to) crowding environment".to_string()
        );
    }
}
