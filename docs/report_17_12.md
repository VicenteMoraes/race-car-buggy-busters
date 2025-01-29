# Report December 17th

Summary of the tests we've done on December 17th.

## Goals

* evaluate how well the driving via `m2p` node is working
* evaluate how well driving in the gazebo simulation tranlates to driving in the real world

## Setup

* have the car placed in the open space
* setup the car with our most recent version of nodes
* startup the car (position (0, 0))
* send messages 'by hand' (publishing the points via terminal, no separate publisher node)
* measure distances with a folding rule

## Test 1

* odometry: `position (0, 0), rotation (1, 0)` 
* assume: pos x value is moving forward, pos y leftward
* goal: move 2 meters forwards (message contains `position (x: 0, y: 0)`)
* behavior: car moved forward, didn't stop driving, when stopping the `m2p` node odometry echoes `postion (x: -41.253, y: 0)`
* result: didn't stop at desired point, unusual behavior
* question: how is the coordinate system really laid out? where is forward, where is backward along the x-axis?
* question: if the point was lying behind vehicle, why didn't the vehicle make a u-turn?

## Test 2

* odometry: `position (-41.253, 0)`
* car is restarted, odometry: `position (-0.288, 0)`
* assume: pos x is moving backwards
* goal: move 2 meters forwards (` position (x: -2.288, y: 0)`)
* problems with node publishing/ receiving points:
* wrong coordinate gets published `position (-43.253, 0)` -> 2 meters forwards based of odometry point before restart
* behavior: instead of driving straight ahead it takes first a left curve, then a right one, car gets caught
* result: quit test, so car doesn't drive 43 meters, pickup car & stop node, odometry: `position (-7.86, -0.63)`, realistically car drove 3 meters forwards and 2 meters to the right in total
* interpretation: odometry does not quite translate to real world measurement
* question: why did it drive curves, when it just needed to drive straight ahead?

## Findings

* car odometry does not quite translate to real world measurements
* unexpected driving behavior, not seen in simulation
* more research needed in odometry & self-localization