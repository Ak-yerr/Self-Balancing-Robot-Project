# Prototype #1
The first prototype of the self-balancing robot worked on a programming level - the IMU data was mostly correct (minus some negligible noise) and the motors functioned individually, as in outside of the self-balancing aspect of the functionality. I used a wooden chassis (1" x 5" x 8") and mounted the electronics, mounts, and LiPo battery onto the front and back faces (the motor mounts on the bottom side) of the plank.

This prototype failed because it could not translate the IMU data into motor function effectively or accurately - oftentimes, the robot would continuously spin backwards, regardless of the IMU readings. This is likely due to insufficient testing of the motors and IMU together, combined with the fact that this prototype used motors that did not have encoders, and could not provide closed-loop PID control. 

The next prototype will be preceded by IMU + motor collective tests and implement the tests with motors with encoders. The prototype itself will have a similar chassis, use the same electronics (other than the motors), and will hopefully fix the noise and translation issues from the previous prototype, which kept vigorously falling forward and backward in the small window where the motors and data somewhat corresponded.

