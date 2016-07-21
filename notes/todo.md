TODO near
=========
- figure out if single2half is really broken
- quaternion compression - use int16
- ellipse upload - use int16
- planning into a trajectory
  + should it go into a moving target or fixed start point?
- test for post-CRTP packet dropping in firmware queues
- test larger swarm
- think about how we want to do real-time control
- think about how to do large-swarm position initialization
- characterize full system latency
- tune attitude controller w/ sinusoid and/or step inputs
- figure out why ellipse tracking is poor
- measure vicon delay with JTAG debugger
- look into delay compensation in EKF

TODO far
=========
- navigation through obstacles with grouped team
- more automated method to compute ctr of mass from marker config (worth it??)
- increase object tracker robustness to outliers
- compute x-staggered positions to render text from a side view
- create "avoid human" planner/controller
- create "conductor wand" planner/controller
