# CarND-Kidnapped-Vehicle-Project

This is the implementation of particle filter used for localization the car on the map.

To give an overview of how this works in steps,

1. Initialize the Filter with GPS data
2. Predict the car's location based on yaw and velocity of the car
3. Determine the locations of nearby landmarks
4. Update weights of particles
5. Resample the particles

The codes for all the above have all been impleneted and taught in the Udacity classroom workspace.  One thing that had to be taken in consideration was actually linking all of it together with actual data coming in every second in the simulator.  This was a very straightforward project in my opinion but made me understood the importance of having a particle filter.

The following is a quick Start Up provided by Udacity.

## Udacity guide
The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

Results
---

After setting up the environment and finishing up the coding, the particle filter created was able to pass the criterias of the project.

The error displayed in the simulator is as follows

Error | x | y | yaw |
Value|0.114|0.108|0.004|