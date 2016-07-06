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
