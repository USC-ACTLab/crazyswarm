2016-07-21 - JAP
----------------
- tested quaternion compression to 32bits, good
- 2 cfs/packet works
- python bindings looking cool
- improved object tracker initialization
- factored out our (not all) crtp packet defs into file shared btw firm & server
- took a video of figure-8 with 4x2 0.5m grid configuration but control is wobbly
- "slow blink" issue is getting worse and needs to be fixed badly
  (it stays in the 2sec red LED period, so its IMU never becomes calibrated)
- can takeoff to different heights

2016-07-20 - JAP
----------------
- ellipse was getting corrupted vicon measurements over the air:
  problem was either single2half or struct packing, dont know which
- tested ellipse traj, it works at slow periods but fails badly on fast ones
- ramping up period does not work for ellipse startup due to math
- noted that Vicon tracker gives plenty of axis flips!!
- online planner to hover works

2016-07-19 - JAP
----------------
- tested integrated binary, eliminates message timing jitter in tracker->server messages
- ellipse has some very strange flaws
- talked with Ying about ideas for tuning controller

2016-07-18 - JAP
----------------
- Found that object tracker was not being compiled with optimization - fixed, is fast now
- #3 has some mechanical problem, even in manual flight
- added LED ring blink to indicate low battery
- profiled, found that EKF takes about 800 uSec on vicon packets
- compiled fw with -O3 instead of -Os, reduces to around 600 uSec
- detailed profiling shows that kalman gain and covariance update are slowest
- probably SGEMM is biggest offender
- #2 misses more packets than everyone else
- starting to have weird firmware issue where it gets stuck at "imu calibration" slow blink

- determined that 7 CFs is "magic number" for one radio - 
  they can fly fine, but add an 8th and suddenly vicon packets rate becomes jittery
- tried removing address discrimination from firmware - 
  so we only send one packet - STILL everything goes bad after adding the 8th!
- so now the only difference between 7 and 8 is one more object for the object tracker to track
- added timing code in several places, see large jitter in delta-t of tracker->server msgs
- experiments seem to eliminate everything besides ROS messaging as source of jitter
- plan to write a monolithic node, vicon sdk -> radio usb driver in one process

2016-07-17 - JAP
----------------
- Tested robust initialization for ICP, it works
- Flew figure 8 with 3 CFs in tight formation, worked
- Tried 9 CFs in tight formation, failed badly
- 5 in looser formation sometimes works, sometimes crashes
- Identified object tracker taking a lot of CPU time
- Profiled object tracker, most of time is in SVD alignment
- Tried Levenberg-Marquardt aligner, but it failed on takeoff
- Simulated delay and low packet rate in transmission, it makes ctrl worse but not crash
- Added red->green indicator of packet rate health on LED ring
- Implemented + tested unified simple config file

2016-07-15 - JAP
----------------
- Tuned controller - much higher attitude gains, higher xy gains
- increased d/p ratio in xy _finally_ solves overshoot in fig-8
- Set all camera apertures to 4.0 and increased threshold
- Added onboard shifting for fig-8 traj
- Tried to fly 3 synchronized figure-8s but were besieged by
  CFs dropping out of the sky due to comm timeouts, dropped vicon packets, etc.
- ROS issues

2016-07-14 - JAP
----------------
Hypothesized that latency in the object tracker might be introducing 
enough delay to cause oscillation.
Instrumented code and found that latency is under 1ms, not a problem.
Found that our object tracker was giving a translational offset of a few cm.
Manually fixed the offset, flight is now nearly as stable as Vicon tracker.
**Putting the object''s center of mass in the right location is really important.**

Matt suggested using Vicon''s tracking, with our role reduced to
detecting and fixing object identity misassignments.
Accuracy of new system is now good enough to track 3 nearly-identical objects robustly,
but if you copy the same object 3 times, it will assign them all to one crazyflie.
Seems like there should be a way to get around this, but we can''t find it.

Raw data from Vicon still contains much noise and spurious markers.
Filtering will be necessary in our tracker.

Fixed some firmware crash bugs:
- name of log variable was too long
- stack overflow after increasing computation in `trajectory_poly.c`
- assert does nothing unless using JTAG debugger

Added better takeoff/landing trajectories with linear yaw interpolation.
Fixed a bug where ctrl expected yaw setpoints in degrees but we were giving radians.
This led us to realize that our yaw gains in ctrl were way too low.
Tuning yaw gains gave us probably best hover yet.
Now XY translation gains need work.

Would like to get an adjustable-speed fan to test controller robustness.

2016-07-13 - JAP
----------------
Set up new Vicon PC.
Attempted to use object tracker but found quick tracking loss.
Corrected too-low Euler angle limits.
Now tracks well but still gives extremely unstable flight.

Flat-black Testors enamel marker is very good fix for reflective USB connector,
but it also scratches off easily

Attempted to run static analyzers:
- Clang complains about GCC-only directives
- Slint found some interesting issues, but only understands C89 syntax; rejects decl in for-loop
- cppcheck finds no issues

2016-07-12 - JAP
----------------
Tuned controller.
TODO: what else?

2016-07-11 - JAP
----------------
Refined camera cable management.
Identified source of strange blobs: aperture settings.
If aperture is too large, camera detects many spurious markers.
If too small, its maximum detection distance is reduced.
Optimal aperture seems to be 4.0 or maybe 5.6.

2016-07-10 - JAP
----------------
Set up new Vicon cameras.
All OK except one showing strange blobs and many spurious markers.

2016-07-08 - JAP
----------------
Set up trusses for new Vicon system.

2016-07-06 - JAP
----------------
Identified matte black polyimide (Kapton) tape as a possible solution for spurious markers on shiny parts.
Requested samples of the following products:
- 3M 7412B
- DuPont Kapton B
- Nitto UTS-30BAF
- Polyonics XT-719

2016-07-05 - JAP
----------------
We had a support call with Vicon regarding the old system:
- Got a temporary license for Tracker 3; it calibrates *much* faster than 2.2, with better results.
- Identified that our calibration wand is not straight
  - "Active" wand from new system gives ~5x reduction in image error
  - Feet on active wand do not have enough travel to successfully level the device on our floor
- Identified that our cameras are out of focus. Markers appear blurry with grey halos.
- Attempted to refocus cameras, but:
  - net blocks us from inside and is difficult to remove
  - wide ladder base + clutter make it impossible to get close to cameras from the outside
  - lenses are focus- and aperture-locked with tiny set screws
  - cables are bound to cage structure, making it difficult to take cameras down
    for easier manipulation while still observing image feed
- Failed to refocus any cameras successfully
- Noticed that shiny USB connector, battery, and possibly motors sometimes appear as spurious markers

2016-06-29 - JAP
----------------
- Polynomial timestretching fixed
- Github sub-repos done
- JTAG tether probably not possible - recommended length is like 8 inches
- Met w/ advisors, moving forward with temporary setup in unfinished new building
- Need to figure out a truss/support for Vicon in new space; tripods not so great

2016-06-25 - JAP
----------------
- Piecewise polynomial trajectories are working
- Need to fix polynomial timestretching (TODO)
- Need to tune controller better for position tracking
- Investigate making a JTAG + power tether??
- Organize code into github sub-repos (goal: one pull + make to set up) (TODO: research git features)
