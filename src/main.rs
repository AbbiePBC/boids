
fn main() {
    // initialise flock
    // for each boid:
        // steer to avoid crowding local flockmates
        // steer towards the average heading of local flockmates
        // steer to move toward the average position of local flockmates

}

struct Flock {
    boids: Vec<Boid>,
    max_dist_before_boid_is_crowded: i32, // todo: these config values would be better more easily accessible by the boids, but want to avoid storing them for each boid?
    max_dist_of_local_boid: i32, // i.e. the radius of the local flock; far boids in the flock don't influence a boid's behaviour
}

impl Flock {
    fn new(flock_size: usize, max_dist_before_boid_is_crowded: i32, max_dist_of_local_boid: i32) -> Flock {
        return Flock {
            boids: Self::generate_boids(flock_size),
            max_dist_before_boid_is_crowded,
            max_dist_of_local_boid,
        };
    }
    fn generate_boids<>(flock_size: usize) -> Vec<Boid> {
        let mut boids = Vec::new();
        for _ in 0..flock_size {
            boids.push(Boid::new(0, 0, 0, 0));
        }
        return boids;
    }
    fn uncrowd_boid(&mut self, boid_to_update: usize, repulsion_from_close_boids: i32,
        num_crowding_boids: i32, total_x_dist_of_crowding_boids: i32,
        total_y_dist_of_crowding_boids: i32) {

        // move away from the average position of the crowding boids
        let dist_to_ave_x_pos_of_crowding_boids: i32 = self.boids[boid_to_update].x_pos - (total_x_dist_of_crowding_boids / num_crowding_boids);
        let dist_to_ave_y_pos_of_crowding_boids: i32 = self.boids[boid_to_update].y_pos - (total_y_dist_of_crowding_boids / num_crowding_boids);

        // update velocity to move away from the average boid position within the crowding flock
        let time_per_frame: i32 = 1;
        self.boids[boid_to_update] =
        Boid::new(
        self.boids[boid_to_update].x_pos + (self.boids[boid_to_update].x_vel * time_per_frame),
        self.boids[boid_to_update].y_pos + (self.boids[boid_to_update].y_vel * time_per_frame),
        self.boids[boid_to_update].x_vel + (dist_to_ave_x_pos_of_crowding_boids * repulsion_from_close_boids),
        self.boids[boid_to_update].y_vel + (dist_to_ave_y_pos_of_crowding_boids * repulsion_from_close_boids),
        )
    }
    fn update_boid(&mut self, boid_to_update: usize, repulsion_from_close_boids: i32) {
        // todo: this initial implementation loops through all boids in the flock,
        // which will need to be done for alignment and cohesion too, so don't do this

        // todo: also this doesn't consider where the boundaries of the frame are, so the boid could be steered out of the frame
        let mut total_x_dist_of_crowding_boids: i32 = 0;
        let mut total_y_dist_of_crowding_boids: i32 = 0;
        let mut num_crowding_boids: i32 = 0;


        let mut boid_idx = 0;
        for other_boid in &self.boids {
            if boid_idx == boid_to_update {
                boid_idx += 1;
                continue;
            }
            boid_idx += 1;
            if self.boids[boid_to_update].is_crowded_by_boid(&other_boid, self.max_dist_before_boid_is_crowded) {
                num_crowding_boids += 1;
                total_x_dist_of_crowding_boids += other_boid.x_pos;
                total_y_dist_of_crowding_boids += other_boid.y_pos;
            }
        }

        if num_crowding_boids > 0 {
            Flock::uncrowd_boid(self, boid_to_update, repulsion_from_close_boids, num_crowding_boids, total_x_dist_of_crowding_boids, total_y_dist_of_crowding_boids);
        }
    }
}

#[derive(Copy, Clone)]
struct Boid {
    x_pos: i32,
    y_pos: i32,
    x_vel: i32,
    y_vel: i32,
}

impl Boid {
    fn new(x_pos: i32, y_pos: i32, x_vel: i32, y_vel: i32) -> Boid {
        return Boid {
            x_pos,
            y_pos,
            x_vel,
            y_vel,
        };
    }

    fn is_crowded_by_boid(&self, other_boid: &Boid, max_dist_before_boid_is_crowded: i32) -> bool {
        return (self.x_pos - other_boid.x_pos).abs() < max_dist_before_boid_is_crowded &&
            (self.y_pos - other_boid.y_pos).abs() < max_dist_before_boid_is_crowded;
    }

}



#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_no_crowding_by_boid_outside_of_crowding_zone() {
        let mut flock = Flock::new(0, 4, 5);
        let boid = Boid::new(1, 1, 1, 1);
        let other_boid = Boid::new(10, 10, 2, 2);
        flock.boids = vec![boid, other_boid];

        assert!(!flock.boids[0].is_crowded_by_boid(&flock.boids[1], flock.max_dist_before_boid_is_crowded));
        assert!(!flock.boids[1].is_crowded_by_boid(&flock.boids[0], flock.max_dist_before_boid_is_crowded));
    }
    #[test]
    fn test_crowding_by_boid_inside_of_crowding_zone() {
        let mut flock = Flock::new(0, 40, 5);
        let boid = Boid::new(1, 1, 1, 1);
        let other_boid = Boid::new(10, 10, 2, 2);
        flock.boids = vec![boid, other_boid];

        assert!(flock.boids[0].is_crowded_by_boid(&flock.boids[1], flock.max_dist_before_boid_is_crowded));
        assert!(flock.boids[1].is_crowded_by_boid(&flock.boids[0], flock.max_dist_before_boid_is_crowded));
    }
    #[test]
    fn test_crowded_boid_has_updated_velocity() {
        let mut flock = Flock::new(0, 40, 5);
        let boid = Boid::new(1, 1, 1, 1);
        let other_boid = Boid::new(10, 10, 1, 5);
        flock.boids = vec![boid, other_boid];

        flock.update_boid(0, 0);
        assert_eq!(flock.boids[0].x_vel, boid.x_vel);
        assert_eq!(flock.boids[0].y_vel, boid.y_vel);
        // v = d/t; t = 1
        assert_eq!(flock.boids[0].x_pos, boid.x_pos + boid.x_vel);
        assert_eq!(flock.boids[0].y_pos, boid.y_pos + boid.y_vel);

        flock.update_boid(1, 1);
        // new velocity = original velocity + repulsion*(difference in displacement)
        assert_eq!(flock.boids[1].x_vel, other_boid.x_vel + 1 * (other_boid.x_pos - flock.boids[0].x_pos));
        assert_eq!(flock.boids[1].y_vel, other_boid.y_vel + 1 * (other_boid.y_pos - flock.boids[0].y_pos));


    }
}
