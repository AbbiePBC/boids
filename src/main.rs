mod boids;

use crate::boids::{Boid, Flock};
use macroquad::prelude::*;

#[macroquad::main("Boids")]
async fn main() -> Result<(), anyhow::Error> {

    let flock_size = 10;
    let mut flock = Flock::new(flock_size, 100.0, 200.0, 1.0, 1.0, 1.0)?;
    flock.frame_width = macroquad::window::screen_width() as i32;
    flock.frame_height = macroquad::window::screen_height() as i32;

    let mid_screen_height = (flock.frame_height/2) as f32;
    let mid_screen_width = (flock.frame_width/2) as  f32;

    // psuedo-randomly generate boids
    // TODO: actually randomly generated the starting positions
    let boid_one = Boid::new(mid_screen_width + 40.0, mid_screen_height - 10.0, -14.0, 13.0);
    let boid_two = Boid::new(mid_screen_width - 20.0,  mid_screen_height - 11.0, 16.0, -14.0);
    let boid_three = Boid::new(mid_screen_width + 20.0, mid_screen_height - 12.0, -8.0, 15.0);
    let boid_four = Boid::new(mid_screen_width - 40.0, mid_screen_height - 13.0, 15.0, 0.0);
    flock.boids = vec![boid_one, boid_two, boid_three, boid_four];

    loop {
        clear_background(WHITE);
        for i in 0..flock_size {
            draw_boid(i, &mut flock);
        }
        next_frame().await
    }
}

fn draw_boid(i : usize, flock: &mut Flock){
    let colours = [RED, BLUE, GREEN, YELLOW];
    let boid = flock.boids[i];
    flock.update_boid(i);
    draw_circle(boid.x_pos, boid.y_pos, 13.0, colours[(i%colours.len())]);
}

