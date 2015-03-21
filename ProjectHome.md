This is a C# implementation of [SceneLib 1.0](http://www.doc.ic.ac.uk/~ajd/software.html) originally written by [Andrew Davison](http://www.doc.ic.ac.uk/~ajd/)
![http://sluggish.uni.cc/monoSLAM/quickcamexpress.jpg](http://sluggish.uni.cc/monoSLAM/quickcamexpress.jpg)

The ability to both recognize landmarks and to accurately estimate your own movement through space is traditionally one of the more difficult problems in robotics.  Usually this kind of thing is done using laser scanners or other high precision optics which are much too expensive to become practical consumer products in the immediate future, and also contain moving parts which could potentially fail during the rough-and-tumble of robotic life.

Monocular SLAM offers a way to map the environment based upon detecting and tracking features in an efficient way and in real time.  The system is closed loop, in that observed features help to construct a map, and in turn the map helps to guide the search for features in subsequent frames.

[How to use MonoSLAM](howtousemonoslam.md)