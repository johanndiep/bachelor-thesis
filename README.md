# Path Planning for an Omnidirectional Underwater Robot

This GitLab repository contains the code for the "Path Planning for an Omnidirectional Underwater Robot" project. It was developed in the context of a bachelor thesis at ETH Zurich.

## Context

![Scubo](https://i.imgur.com/RVaz3Mf.png)

Scubo is a submersible ROV (remotely operated vehicle) developed in the context of a focusproject at ETH Zurich. Due to the arrangement of eight non-moving propulsion units and the superposition of its forces, it is able to move in any direction without prior change of its orientation. Besides, it is equipped with six cameras pointing in each direction and the necessary hardware for autonomy. Several bachelor theses are concerned with the software implementation of an autonomous path following system.

This bachelor thesis discusses the implementation of an offline global path planning algorithm for the Scubo robot. The algorithm uses a point cloud of the environment to generate a set of collision-free waypoints between an initial and a goal state for the trajectory controller to follow. Since the robot is capable of omnidirectional movements, the generated path does not have to satisfy any kinematic or dynamic constraints of the system other than avoiding collision with obstacles.

## Method

The report explains the basic idea behind available path planning methods on a conceptual level at first. In a second step, the implementation of the path planning algorithm using a variant of the rapidly-exploring random tree (RRT) method is presented in more detail. Lastly, the algorithm is used in a practical application where the goal is to scan an object from all sides under different angles. In this case, the developed path planner in combination with a traveling salesman problem solver proves to be highly suitable to generate a collision-free and cost-efficient path around an object.

## Installation

* The path planner algorithm was implemented and tested on [Matlab](https://ch.mathworks.com/de/products/matlab.html).
* [Paraview](https://www.paraview.org/) was used for better visualization.


## Example

![Planner in 2D](https://i.imgur.com/WV7Zq71.png)
![Planner in 3D](https://i.imgur.com/D6qn3Gg.png)

## Documentation

* 16. June 2016: Version 1.0, [Documentation](https://gitlab.com/jdiep/bachelor-thesis/blob/master/BA%20Report/report.pdf)

## Credential

The core of this work is based on the publication ["Sampling-based Algorithms for Optimal Motion Planning"](https://arxiv.org/pdf/1105.1186.pdf) by Sertac Karaman and Emilio Frazzoli.

## Authors

* Johann Diep (BSc Student Mechanical Engineering, ETH Zurich, jdiep@student.ethz.ch)
* In the context of the "Scubo" focusproject.
