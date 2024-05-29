# Barrier Function Trajectory Planning
This is a Master Thesis repository for an implementation of barrier functions for trajectory optimization.

## Barrier functions

In the barrierFunctions folder you will find the BarrierClass script, which is the barrier function implementation. The folder also contains a barrier_plotting script, which uses the barrier function implementation to produce plots on the given trajectories.

## Webots
In the weBots folder, you will find the environment made for the Webots simulator. It consists of the world, protos used for by Webots and controllers. The controllers folder contains to different communication protcol folders, a ground based and a peer-to-peer based protocol.

In order to change the communication protocol in use for the drones, in the Webots environment, change the controller used by the drones.

