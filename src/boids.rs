use std::ops::AddAssign;
use std::error;
use std::fmt;
use macroquad::prelude::*;
use anyhow::{bail, Result, Error, anyhow};

#[derive(Debug)]
pub(crate) struct Flock {
    pub(crate) boids: Vec<Boid>,
    max_dist_before_boid_is_no_longer_crowded: f32,
    max_dist_of_local_boid: f32, // i.e. the radius of the local flock; far boids in the flock don't influence a boid's behaviour
    repulsion_factor: f32, // how much a boid wants to move away from other boids
    adhesion_factor: f32, // how much a boid wants to stay with the flock
    cohesion_factor: f32, // how much a boid wants to move towards the average position of the flock
    time_per_frame: i32,
    pub(crate) frame_width: i32,
    pub(crate) frame_height: i32,
}

fn validate_factors(repulsion_factor: f32, adhesion_factor: f32, cohesion_factor: f32) -> Vec<CreationError> {
    let repulsion = check_float_between_zero_and_one(repulsion_factor, "repulsion".to_string());
    let adhesion =  check_float_between_zero_and_one(adhesion_factor, "adhesion".to_string());
    let cohesion =  check_float_between_zero_and_one(cohesion_factor, "cohesion".to_string());

    let mut errors: Vec<_> = [repulsion, adhesion, cohesion]
        .into_iter()
        .filter_map(|option| option)
        .collect();
    errors
}

fn validate_distances(max_dist_before_boid_is_crowded: f32, max_dist_of_local_boid: f32) -> Option<CreationError> {
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
    pub(crate) fn new(flock_size: usize,
                      max_dist_before_boid_is_crowded: f32,
                      max_dist_of_local_boid: f32,
                      repulsion_factor: f32,
                      adhesion_factor: f32,
                      cohesion_factor: f32,

    ) -> Result<Flock, InvalidFlockConfig> {
        let mut flock = Flock {
            boids: Vec::new(),
            max_dist_before_boid_is_no_longer_crowded: max_dist_before_boid_is_crowded,
            max_dist_of_local_boid,
            repulsion_factor,
            adhesion_factor,
            cohesion_factor,
            time_per_frame: 1,
            frame_width: 800,
            frame_height: 500,
        };
        let _ = flock.validate()?;
        flock.init(flock_size);
        Ok(flock)
    }

    fn validate(&self) -> Result<(), InvalidFlockConfig> {
        let mut errors = validate_factors(self.repulsion_factor, self.adhesion_factor, self.cohesion_factor);

        if let Some(creation_error) = validate_distances(self.max_dist_before_boid_is_no_longer_crowded, self.max_dist_of_local_boid) {
            errors.push(creation_error);
        }

        if errors.len() > 0 {
            return Err(InvalidFlockConfig { errors }); // the "into()" will use the From trait to convert the InvalidFlockConfig into an Error
        }

        return Ok(());
    }

    /// Not necessary to split this out for a single fn call
    /// But done to show how initialisation can be done in a separate function
    fn init(&mut self, flock_size: usize) {
        self.boids = Self::generate_boids(flock_size);
    }

    fn generate_boids(flock_size: usize) -> Vec<Boid> {
        let mut boids = Vec::new();
        for _ in 0..flock_size {
            boids.push(Boid::new(0.0, 0.0, 0.0, 0.0));
        }
        return boids;
    }
    fn uncrowd_boid(&mut self, boid_to_update: usize,
                    num_crowding_boids: i32, total_x_dist_of_crowding_boids: f32,
                    total_y_dist_of_crowding_boids: f32) {

        // move away from the average position of the crowding boids
        let dist_to_ave_x_pos_of_crowding_boids: f32 = self.boids[boid_to_update].x_pos - (total_x_dist_of_crowding_boids as f32 / num_crowding_boids as f32);
        let dist_to_ave_y_pos_of_crowding_boids: f32 = self.boids[boid_to_update].y_pos - (total_y_dist_of_crowding_boids as f32 / num_crowding_boids as f32);

        // update velocity to move away from the average boid position within the crowding flock
        self.boids[boid_to_update] = Boid {
            x_vel: self.boids[boid_to_update].x_vel + (dist_to_ave_x_pos_of_crowding_boids * self.repulsion_factor),
            y_vel: self.boids[boid_to_update].y_vel + (dist_to_ave_y_pos_of_crowding_boids * self.repulsion_factor),
            x_pos: self.boids[boid_to_update].x_pos + (self.boids[boid_to_update].x_vel * self.time_per_frame as f32),
            y_pos: self.boids[boid_to_update].y_pos + (self.boids[boid_to_update].y_vel * self.time_per_frame as f32),
        }
    }
    fn align_boid(&mut self, boid_to_update: usize,
                  num_local_boids: i32, total_x_vel_of_local_boids: f32,
                  total_y_vel_of_local_boids: f32) {
        let average_x_vel: f32 = total_x_vel_of_local_boids as f32 / num_local_boids as f32;
        let average_y_vel: f32 = total_y_vel_of_local_boids as f32 / num_local_boids as f32;
        // update the boid's velocity to move towards the average velocity of the local flock, by some adhesion factor
        self.boids[boid_to_update] = Boid {
            x_vel: self.boids[boid_to_update].x_vel + ((average_x_vel - self.boids[boid_to_update].x_vel) * self.adhesion_factor),
            y_vel: self.boids[boid_to_update].y_vel + ((average_y_vel - self.boids[boid_to_update].y_vel) * self.adhesion_factor),
            x_pos: self.boids[boid_to_update].x_pos + (self.boids[boid_to_update].x_vel * self.time_per_frame as f32),
            y_pos: self.boids[boid_to_update].y_pos + (self.boids[boid_to_update].y_vel * self.time_per_frame as f32),
        }
    }
    fn cohere_boid(&mut self, boid_to_update: usize,
                   num_local_boids: i32, total_x_dist_of_local_boids: f32,
                   total_y_dist_of_local_boids: f32) {
        // todo
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
            if self.boids[boid_to_update].is_crowded_by_boid(&other_boid, self.max_dist_before_boid_is_no_longer_crowded) {
                num_crowding_boids += 1;
                total_x_dist_of_crowding_boids += other_boid.x_pos;
                total_y_dist_of_crowding_boids += other_boid.y_pos;
            } else if self.boids[boid_to_update].is_within_sight_of_local_boid(&other_boid, self.max_dist_of_local_boid) {
                num_local_boids += 1;
                total_of_local_boids += *other_boid;
            }
            // else, the other_boid is too far away to affect the boid we're updating
        }

        if num_crowding_boids > 0 {
            Flock::uncrowd_boid(self, boid_to_update, num_crowding_boids, total_x_dist_of_crowding_boids, total_y_dist_of_crowding_boids);
        }
        if num_local_boids > 0 {
            Flock::align_boid(self, boid_to_update, num_local_boids, total_of_local_boids.x_vel, total_of_local_boids.y_vel);
            Flock::cohere_boid(self, boid_to_update, num_local_boids, total_of_local_boids.x_pos, total_of_local_boids.y_pos);
        }
        // todo: test the following
        if num_local_boids == 0 && num_crowding_boids == 0 {
            // boid is unaffected by other boids, continue moving in same direction
            // maybe a random direction would be more realistic idk
            self.boids[boid_to_update] = Boid {
                x_vel: self.boids[boid_to_update].x_vel,
                y_vel: self.boids[boid_to_update].y_vel,
                x_pos: self.boids[boid_to_update].x_pos + (self.boids[boid_to_update].x_vel * self.time_per_frame as f32),
                y_pos: self.boids[boid_to_update].y_pos + (self.boids[boid_to_update].y_vel * self.time_per_frame as f32),
            }
        }

        self.maybe_reflect_off_boundaries(boid_to_update);
    }

    fn maybe_reflect_off_boundaries(&mut self, boid_to_update: usize) {
        if self.boids[boid_to_update].x_pos.abs() >= ((self.frame_width - 1)/2) as f32 {
            self.boids[boid_to_update].x_vel = -self.boids[boid_to_update].x_vel;
        }
        if self.boids[boid_to_update].y_pos.abs() >= ((self.frame_height - 1)/2) as f32 {
            self.boids[boid_to_update].y_vel = -self.boids[boid_to_update].y_vel;
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct Boid {
    pub(crate) x_pos: f32,
    pub(crate) y_pos: f32,
    x_vel: f32,
    y_vel: f32,
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

    fn is_crowded_by_boid(&self, other_boid: &Boid, max_dist_before_boid_is_no_longer_crowded: f32) -> bool {
        return (self.x_pos - other_boid.x_pos).abs() < max_dist_before_boid_is_no_longer_crowded &&
            (self.y_pos - other_boid.y_pos).abs() < max_dist_before_boid_is_no_longer_crowded;
    }

    fn is_within_sight_of_local_boid(&self, other_boid: &Boid, max_dist_of_local_boid: f32) -> bool {
        return (self.x_pos - other_boid.x_pos).abs() < max_dist_of_local_boid &&
            (self.y_pos - other_boid.y_pos).abs() < max_dist_of_local_boid;
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


fn check_float_between_zero_and_one(value: f32, name: String) -> Option<CreationError> {
    match value {
        x if x < 0.0 => Some(CreationError::FactorShouldBeMoreThanZero(name)),
        x if x > 1.0 => Some(CreationError::FactorShouldBeLessThanOne(name)),
        _ => None
    }
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
            CreationError::FactorShouldBeMoreThanZero(factor_name) => factor_name.to_owned() + " factor is negative",
            CreationError::FactorShouldBeLessThanOne(factor_name) => factor_name.to_owned() + " factor is too large and should be below zero",
            CreationError::LocalEnvironmentIsSmallerThanCrowdingEnvironment => "local environment is smaller than (or equal to) crowding environment".to_owned(),
        };
        f.write_str(&description)
    }
}

impl error::Error for CreationError {}



#[cfg(test)]
mod tests {
    use anyhow::Context;
    use super::*;
    #[test]
    fn test_no_crowding_by_boid_outside_of_crowding_zone() {
        let mut flock = Flock::new(0, 4.0, 5.0, 0.0, 0.0, 0.0).unwrap();
        let boid = Boid::new(1.0, 1.0, 1.0, 1.0);
        let other_boid = Boid::new(10.0, 10.0, 2.0, 2.0);
        flock.boids = vec![boid, other_boid];

        assert!(!flock.boids[0].is_crowded_by_boid(&flock.boids[1], flock.max_dist_before_boid_is_no_longer_crowded));
        assert!(!flock.boids[1].is_crowded_by_boid(&flock.boids[0], flock.max_dist_before_boid_is_no_longer_crowded));
    }

    #[test]
    fn test_crowding_by_boid_inside_of_crowding_zone() {
        let mut flock = Flock::new(0, 40.0, 500.0, 0.0, 0.0, 0.0).unwrap();
        let boid = Boid::new(1.0, 1.0, 1.0, 1.0);
        let other_boid = Boid::new(10.0, 10.0, 2.0, 2.0);
        flock.boids = vec![boid, other_boid];

        assert!(flock.boids[0].is_crowded_by_boid(&flock.boids[1], flock.max_dist_before_boid_is_no_longer_crowded));
        assert!(flock.boids[1].is_crowded_by_boid(&flock.boids[0], flock.max_dist_before_boid_is_no_longer_crowded));
    }

    #[test]
    fn test_crowded_boid_has_updated_velocity() {
        let mut flock = Flock::new(0, 40.0, 500.0, 0.0, 0.0, 0.0).unwrap();
        let boid = Boid::new(1.0, 1.0, 1.0, 1.0);
        let other_boid = Boid::new(10.0, 10.0, 1.0, 5.0);
        flock.boids = vec![boid, other_boid];

        flock.uncrowd_boid(0, 1, 10.0, 10.0);
        assert_eq!(flock.boids[0].x_vel, boid.x_vel);
        assert_eq!(flock.boids[0].y_vel, boid.y_vel);
        // v = d/t; t = 1
        assert_eq!(flock.boids[0].x_pos, boid.x_pos + boid.x_vel); // = 2
        assert_eq!(flock.boids[0].y_pos, boid.y_pos + boid.y_vel); // = 2

        flock.repulsion_factor = 1.0;
        flock.uncrowd_boid(1, 1, flock.boids[0].x_pos, flock.boids[0].x_pos);
        // new velocity = original velocity + repulsion*(difference in displacement)*time

        assert_eq!(flock.boids[1].x_vel, other_boid.x_vel + flock.repulsion_factor * (other_boid.x_pos - flock.boids[0].x_pos));
        assert_eq!(flock.boids[1].y_vel, other_boid.y_vel + flock.repulsion_factor * (other_boid.y_pos - flock.boids[0].x_pos));

    }

    #[test]
    fn test_boid_outside_of_local_zone() {
        let mut flock = Flock::new(0, 1.0, 50.0, 0.0, 0.0, 0.0).unwrap();
        let boid = Boid::new(1.0, 1.0, 1.0, 1.0);
        let other_boid = Boid::new(10.0, 10.0, 2.0, 2.0);
        flock.boids = vec![boid, other_boid];

        // not crowded
        assert!(!flock.boids[0].is_crowded_by_boid(&flock.boids[1], flock.max_dist_before_boid_is_no_longer_crowded));
        assert!(!flock.boids[1].is_crowded_by_boid(&flock.boids[0], flock.max_dist_before_boid_is_no_longer_crowded));

        // but still within local zone
        assert!(flock.boids[0].is_within_sight_of_local_boid(&flock.boids[1], flock.max_dist_of_local_boid));
        assert!(flock.boids[1].is_within_sight_of_local_boid(&flock.boids[0], flock.max_dist_of_local_boid));
    }

    #[test]
    fn test_adhesion() {
        let mut flock = Flock::new(0, 1.0, 50.0, 0.0, 1.0, 0.0).unwrap();
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);
        let boid_2 = Boid::new(3.0, 3.0, 10.0, 1000.0);
        let boid_3 = Boid::new(5.0, 5.0, 10.0, -1000.0);
        flock.boids = vec![boid, boid_2, boid_3];

        flock.align_boid(0, 2, 20.0, 0.0);
        assert_eq!(flock.boids[0].x_vel, 10.0);
        assert_eq!(flock.boids[0].y_vel, 0.0);
    }

    #[test]
    fn test_no_adhesion() {
        let mut flock = Flock::new(0, 1.0, 50.0, 0.0, 0.0, 0.0).unwrap();
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);
        let boid_2 = Boid::new(3.0, 3.0, 10.0, 1000.0);
        let boid_3 = Boid::new(5.0, 5.0, 10.0, -1000.0);
        flock.boids = vec![boid, boid_2, boid_3];

        flock.align_boid(0, 2, 20.0, 0.0);
        assert_eq!(flock.boids[0].x_vel, 1.0);
        assert_eq!(flock.boids[0].y_vel, 5.0);
    }

    #[test]
    fn test_half_adhesion() {
        let mut flock = Flock::new(0, 1.0, 50.0, 0.0, 0.5, 0.0).unwrap();
        let boid = Boid::new(1.0, 1.0, 1.0, 5.0);
        let boid_2 = Boid::new(3.0, 3.0, 10.0, 1000.0);
        let boid_3 = Boid::new(5.0, 5.0, 10.0, -1000.0);
        flock.boids = vec![boid, boid_2, boid_3];

        flock.align_boid(0, 2, 20.0, 0.0);
        assert_eq!(flock.boids[0].x_vel, 5.5);
        assert_eq!(flock.boids[0].y_vel, 2.5);
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

        let result = validate_distances( 20.0, 2.0);
        assert_eq!(result, Some(CreationError::LocalEnvironmentIsSmallerThanCrowdingEnvironment));
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
        assert_eq!(CreationError::FactorShouldBeLessThanOne("adhesion".to_string()).to_string(), "adhesion factor is too large and should be below zero".to_string());
        assert_eq!(CreationError::FactorShouldBeMoreThanZero("repulsion".to_string()).to_string(), "repulsion factor is negative".to_string());
        assert_eq!(CreationError::LocalEnvironmentIsSmallerThanCrowdingEnvironment.to_string(), "local environment is smaller than (or equal to) crowding environment".to_string());
    }
}
