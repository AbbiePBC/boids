
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
    fn uncrowd_boid(&mut self, boid_to_uncrowd: usize) {
        // todo: this initial implementation loops through all boids in the flock,
        // which will need to be done for alignment and cohesion too, so don't do this

        // todo: also this doesn't consider where the boundaries of the frame are, so the boid could be steered out of the frame

        // if there are any boids within the max_dist_before_boid_is_crowded radius, return true
        let mut x_dist_to_crowding_boids: i32 = 0;
        let mut y_dist_to_crowding_boids: i32 = 0;
        let mut num_crowding_boids: i32 = -1; // -1 because the boid itself is included in the count
        for other_boid in &self.boids {
            if self.boids[boid_to_uncrowd].is_crowded_by_boid(&other_boid, self.max_dist_before_boid_is_crowded) {
                num_crowding_boids += 1;
                x_dist_to_crowding_boids += self.boids[boid_to_uncrowd].x_pos - other_boid.x_pos;
                y_dist_to_crowding_boids += self.boids[boid_to_uncrowd].y_pos - other_boid.y_pos;
            }
        }

        if num_crowding_boids > 0 {
            // move away from the average position of the crowding boids
            // todo: don't create a new boid just to update the positions

            // if the boid is directly on top of another boid, move it more than usual
            // todo: should be using positions to inform new velocities rather than just moving the boid
            if x_dist_to_crowding_boids == 0 {
                x_dist_to_crowding_boids = self.max_dist_before_boid_is_crowded;
            }
            if y_dist_to_crowding_boids == 0 {
                y_dist_to_crowding_boids = self.max_dist_before_boid_is_crowded;
            }
            self.boids[boid_to_uncrowd] =
                Boid::new(
                    self.boids[boid_to_uncrowd].x_pos + (x_dist_to_crowding_boids / num_crowding_boids),
                    self.boids[boid_to_uncrowd].y_pos + (y_dist_to_crowding_boids / num_crowding_boids),
                    self.boids[boid_to_uncrowd].x_vel,
                    self.boids[boid_to_uncrowd].y_vel);
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
        return (self.x_pos - other_boid.x_pos).abs() < max_dist_before_boid_is_crowded ||
            (self.y_pos - other_boid.y_pos).abs() < max_dist_before_boid_is_crowded;
    }

}



#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_crowding() {
        let mut flock = Flock::new(0, 4, 5);
        let boid = Boid::new(2, 4, 2, 2);
        let other_boid = Boid::new(4, 4, 2, 2);
        flock.boids = vec![boid, other_boid];

        // first boid is crowded by second boid
        assert!(flock.boids[0].is_crowded_by_boid(&flock.boids[1], flock.max_dist_before_boid_is_crowded));
        flock.uncrowd_boid(0);
        // so the first boid is moved away
        assert_eq!(flock.boids[0].x_pos, 0);
        assert_eq!(flock.boids[0].y_pos, 8);

        //now that the first boid has moved, the second boid is no longer crowded
        assert_eq!(flock.boids[1].is_crowded_by_boid(&flock.boids[0], flock.max_dist_before_boid_is_crowded), false);
        flock.uncrowd_boid(1);
        // and it doesn't move
        assert_eq!(flock.boids[1].x_pos, other_boid.x_pos);
        assert_eq!(flock.boids[1].y_pos, other_boid.y_pos);
    }
}
