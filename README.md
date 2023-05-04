# Peggy: The Peg Solitaire-playing delta-robot

This is part of a graduation project made at DTU (Technical University of Denmark) in collaboration with the solitaire site [Online Solitaire](https://online-solitaire.com/) and the multiplayer card game website [World of Card Games](http://worldofcardgames.com/).

A thorough description of the thoughts and calculations that led up to the creation of this delta-robot can be found in a series of blog posts, the first of which goes through the [inverse calculations of a delta-robot](https://online-solitaire.com/blog/calculating-the-inverse-kinematics-for-a-peg-solitaire-playing-delta-robot/).

In this repository, you'll find all the code necessary to get Peggy to play Peg Solitaire. You can see the robot in action here: [[World of Card Games](https://www.youtube.com/watch?v=wCAqL-u2s_U)](https://www.youtube.com/watch?v=wCAqL-u2s_U).

The robot was designed, created and programmed from scratch, mainly using laser-cutting as the predominant production technique and an Arduino and three ODrives to control the actuators. 

The robot's programming can roughly be divided into 3 main sections: The first section deals with the robot's kinematics, the second makes the robot run in a straight path from point A to point B with appropriate acceleration and deceleration, and the third makes the robot play a game of Peg Solitaire. In addition, there are several helper functions, but most of the written code revolves around these sections.

It should be noted that degrees have been chosen instead of radians as the input for the various functions. This choice was made because most people find it easier to relate to degrees than radians. In other words, it is easier to look at an arm and see that it has moved 37 degrees than it is to relate to the fact that it has moved 0.645 radians. In the functions, the degrees are converted to radians, which are used in the various calculations.

The robot is controlled by an Arduino Mega (Rev3), which is connected via USB to a computer. To update the robot's program, a new program is simply pushed to the Arduino, and then the program's functions can be called via the Arduino's serial port.