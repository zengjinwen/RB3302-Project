# Signed AMCL motion model, version 2

Plugin based on the Navigation2 Jazzy motion-model interface (LGPL-2.1-or-later; original notices retained). It runs inside the existing AMCL node.

Signed odometry dx/dy is transformed into the previous robot frame and then into each particle's orientation. There is no small-distance forward default. No odometry change causes no motion noise. Pure rotation changes heading only.

Translation magnitude is multiplied by a positive mean-one lognormal variable: `exp(N(0,s^2)-s^2/2)`, with `s^2=log(1+alpha3+alpha4*turn^2)`. Its expected scale is one, it preserves direction, and its absolute uncertainty vanishes continuously with translation distance. This replaces the biased zero-clipped Gaussian. Heading noise uses alpha1/alpha2. This is a custom uncertainty model, not a physically calibrated noise distribution.

Laser scoring and resampling remain unchanged. Startup prints `signed-vector-v2` to confirm the loaded binary version. A wrong odometry input still needs correction upstream.

Build rb3302_amcl_motion and rb3302_mission and source the overlay.
