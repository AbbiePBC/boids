## BOIDS

This is a simple implementation of the boids algorithm in rust. The boids algorithm is a simple algorithm that simulates the flocking behavior of birds.
The algorithm is based on three simple rules as described [here](https://www.red3d.com/cwr/boids/) by the original author.

**Separation:** steer to avoid crowding local flockmates

**Alignment:** steer towards the average heading of local flockmates

**Cohesion:** steer to move toward the average position of local flockmates

[This resource](https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html) contains more information on the algorithm and a possible implementation in pseudocode.
