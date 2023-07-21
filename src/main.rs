mod boids;

use crate::boids::{Boid, Flock};
use macroquad::prelude::*;

#[macroquad::main("Boids")]
async fn main() -> Result<(), anyhow::Error> {

    let flock_size = 10;
    let mut flock = Flock::new(flock_size, 100.0, 200.0, 0.2, 0.8, 0.9)?;
    flock.frame_width = macroquad::window::screen_width();
    flock.frame_height = macroquad::window::screen_height();

    flock.randomly_generate_boids();

    loop {
        clear_background(WHITE);
        draw_updated_boids(&mut flock);
        next_frame().await
    }
}

fn draw_updated_boids(flock: &mut Flock){
    let colours = [RED, BLUE, GREEN, YELLOW];
    for i in 0..flock.flock_size {
        flock.update_boid(i);
        draw_circle(flock.boids[i].x_pos, flock.boids[i].y_pos, 13.0, colours[(i % colours.len())]);
    }
}
