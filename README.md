This is a library allowing multithreading on hardware (motors, servos, etc.) despite the limit to one active thread at a time in the expansion and control hubs.

In addition, the library comes with a few features that utilize this capability, such as the Sequence class, which allows for a sequence of actions to occur on another thread while the main thread loops.

This library was designed to be as plug-and-play as possible, but will require a few changes. Reference or use the sample configuration and sample opmode files to see those changes. This library was designed to work with RoadRunner, and uncommenting the classes added for RoadRunner (and fixing imports) should allow for motors and odometry pods to be replaced seamlessly by the classes from ThreadedHardware.

To install the library, either download this repository and write programs in the TeamCode module (only recommended at the start of a season), or download the repository and paste the ThreadedHardware module into an existing repository before fixing import statements (recommended).

This is the alpha version of the library, so please let us know if you would like certain features to be added/found some bugs. A video guide on using this library will be posted on the AtticFanatics YouTube channel soon.

(Written entirely by Team 14079, the Attic Fanatics)
