# CarND-Extended-Kalman-Filter

In this project, I am utilizing an extended kalman filter to estimate the state of a moving vehicle with noisy lidar and radar measurements.
The source file will work in connection to this [simulator](https://github.com/udacity/self-driving-car-sim/releases).
You can find the visualization of the estimation performance in this [video](https://youtu.be/hVaVZWJ05Jw).

To successfully run the source code, you need to install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) on your machine. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Install the [simulator](https://github.com/udacity/self-driving-car-sim/releases) on your machine.
3. Install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets).
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
