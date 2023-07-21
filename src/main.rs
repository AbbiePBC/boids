mod boids;

use crate::boids::{Boid, Flock};
use macroquad::prelude::*;

#[macroquad::main("Boids")]
async fn main() -> Result<(), anyhow::Error> {

    let flock_size = 10;
    let mut flock = Flock::new(flock_size, 100.0, 200.0, 1.0, 1.0, 1.0)?;
    flock.frame_width = macroquad::window::screen_width();
    flock.frame_height = macroquad::window::screen_height();

    flock.randomly_generate_boids();

    loop {
        clear_background(WHITE);
        for i in 0..flock.flock_size {
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

