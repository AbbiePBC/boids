use anyhow::{anyhow, Error};
use std::error;
use std::fmt;

#[derive(PartialEq, Debug)]
pub(crate) enum CreationError {
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

pub(crate) fn validate_factors(
    repulsion_factor: f32,
    adhesion_factor: f32,
    cohesion_factor: f32,
) -> Vec<CreationError> {
    let repulsion = check_float_between_zero_and_one(repulsion_factor, "repulsion".to_string());
    let adhesion = check_float_between_zero_and_one(adhesion_factor, "adhesion".to_string());
    let cohesion = check_float_between_zero_and_one(cohesion_factor, "cohesion".to_string());

    let errors: Vec<_> = [repulsion, adhesion, cohesion]
        .into_iter()
        .filter_map(|option| option)
        .collect();
    errors
}

pub(crate) fn validate_distances(
    max_dist_before_boid_is_crowded: &f32,
    max_dist_of_local_boid: &f32,
) -> Option<CreationError> {
    if *max_dist_before_boid_is_crowded >= *max_dist_of_local_boid {
        return Some(CreationError::LocalEnvironmentIsSmallerThanCrowdingEnvironment);
    }
    return None;
}

#[derive(Debug)]
pub(crate) struct InvalidFlockConfig {
    pub(crate) errors: Vec<CreationError>,
}

impl From<InvalidFlockConfig> for Error {
    fn from(invalid_flock_config: InvalidFlockConfig) -> Self {
        anyhow!("Invalid Flock input: {:?}", invalid_flock_config.errors)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_incorrect_factor_inputs() {
        let result = validate_factors(2.0, -4.9, 1.0);
        let expected_errors = vec![
            CreationError::FactorShouldBeLessThanOne("repulsion".to_string()),
            CreationError::FactorShouldBeMoreThanZero("adhesion".to_string()),
        ];
        assert_eq!(result, expected_errors);
    }
    #[test]
    fn test_incorrect_distance_inputs() {
        let short_dist: f32 = 2.0;
        let long_dist: f32 = 20.0;
        let result = validate_distances(&long_dist, &short_dist);
        assert_eq!(
            result,
            Some(CreationError::LocalEnvironmentIsSmallerThanCrowdingEnvironment)
        );
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
