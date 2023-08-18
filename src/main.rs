mod boids;
mod flock;
mod validate;
// todo: actually read about modules and file structures in rust
use flock::Flock;

use crate::flock::FrameDimensions;
use macroquad::prelude::*;
const TIME_PER_FRAME: f32 = 1.0;

#[macroquad::main("Boids")]
async fn main() -> Result<(), anyhow::Error> {
    let flock_size = 1;
    let mut flock = Flock::new(flock_size, 300.0, 400.0, 0.3, 0.3, 0.3)?;
    let frame_dimensions = FrameDimensions {
        frame_width: macroquad::window::screen_width(),
        frame_height: macroquad::window::screen_height(),
    };

    flock.randomly_generate_boids(&frame_dimensions);

    loop {
        clear_background(WHITE);
        draw_updated_boids(&mut flock, &frame_dimensions);
        next_frame().await
    }
}

fn draw_updated_boids(flock: &mut Flock, frame_dimensions: &FrameDimensions) {
    let colours = [RED, BLUE, GREEN, YELLOW];
    for i in 0..flock.flock_size {
        // todo: maybe there's a way to use boids with flock.boids.iter_mut()?
        // unsure if this would work to allow exclusion of current boid from totals
        // could work around that by summing then subtracting current boid's values
        flock.update_boid(i, &frame_dimensions);
        draw_circle(
            flock.boids[i.clone()].x_pos.clone(),
            flock.boids[i.clone()].y_pos.clone(),
            2.5,
            colours[(i.clone() % colours.len())],
        );
    }
}
