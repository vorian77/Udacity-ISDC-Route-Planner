# Route-Planner class project
Course work for: Udacity's Intro To Self-Driving Cars Nano Degree Program
My implementation of the route planner project, using the A-Star search algorithm.

**Inputs:**
- map - (dict) {node_id: {'pos': (x_pos, y_pos), 'connections': \[node_id1 ... node_idn]\}}
- node_id_start - (int) node id of starting node in the map
- node_id_goal - (int) node id of terminating node in the map

**Returns:**
- list - containing node_ids that define shortest possible path between node_id_start and node_id_goal

Students were supplied with an extensive code template for this project. 
The student's (my) contributions occur after the "TODO" prompts.
The student's (my) responsibilities included selecting and defining the data structures, and
programming the algorithm's scoring components, including the heuristic cost estimator and 
the node selector.


## Table of Contents

* [Dependencies and Pre-requisites](#dependencies-and-pre-requisites)
* [Installation and Usage](#installation-and-usage)
* [Known Issues](#known-issues)
* [Future Plans](#future-plans)
* [Support](#support)
* [Contributing](#contributing)
* [Authors and Acknowledgement](#authors-and-acknowledgement)
* [License](#license)


## Dependencies and Pre-requisites
* Python 3.x (including math and numbers libraries)


## Installation and Usage

1. Copy files path_planner.py, helpers.py, and test.py to a directory,
2. In a terminal, navigate to the directory, 
3. Execute `python3 path_planner.py`.

## Known Issues

None.

## Future Plans

None.

## Support

Please open an issue to receive help. 


## Contributing

Pull requests are welcome. Please open an issue to discuss any changes you would like to make or see.


## Authors and Acknowledgement

This is coursework for the [Udacity - Introduction to Self-Driving Cars Nano Degree](https://www.udacity.com/course/intro-to-self-driving-cars--nd113)

Unit: Navigating Data Structures
Project: Implement Route Planner

## License

[MIT](https://choosealicense.com/licenses/mit/)
