##ThreadedHardware

This is a library allowing multithreading on hardware (motors, servos, etc.) despite the limit to one active thread at a time in the expansion and control hubs.

In addition, the library comes with a few features that utilize this capability, such as the Sequence class, which allows for a sequence of actions to occur on another thread while the main thread loops.

This library was designed to be as plug-and-play as possible, and adding the ThreadedHardware module to the teamcode of an existing project should work after imports are fixed. In addition, this library was designed to work with RoadRunner, and uncommenting the classes added for RoadRunner (and fixing imports) should allow for motors and odometry pods to be replaced seamlessly by the classes from ThreadedHardware.

This is the alpha version of the library, so please let us know if you would like certain features to be added/found some bugs. A video user guide on this library will be posted on the AtticFanatics YouTube channel soon.

(Written entirely by Team 14079, the Attic Fanatics)