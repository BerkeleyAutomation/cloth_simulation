# cloth_simulation
This repository contains scripts used to simulate cloth physics in 3D under various conditions and interactions.

## Files, Scripts, and Directories:

### point.py
Contains the Point class, which represents a point mass.

### constraint.py
Contains the Constraint class, which represents a relationship between two points that the points try to maintain.

### cloth.py
Contains the Cloth class, which represents a collection of points structured into a rectangular grid in 3D along with perpendicular constraints for each point. The cloth is pinned along the top and bottom by default. It can also be grabbed and tensioned by a tensioner object.

### circlecloth.py
Contains the CircleCloth class which extends the Cloth class. It is similar, but also has a circular pattern drawn on it with specified dimensions/location, and can be grabbed and tensioned as well. The cloth is pinned along the top and bottom by default as well.

### shapecloth.py
Contains a ShapeCloth class which extends the Cloth class. Similar to the CircleCloth class, it has a pattern drawn on it. It takes in a function that specifies whether or not a point is on the outline.

### tensioner.py
Contains the Tensioner class which can be used to grab a position on the cloth, and tug it in a direction.

### mouse.py
Contains the Mouse class, which can be used as a medium through which a physical or virtual mouse can interact with a cloth.

### util.py
Contains utility functions relating to the scripts and objects in the repository.

### demo.py
Contains a main method that can be run out of the box to view a demo of the code in action.

#### To run:

To run an trial that cuts a predefined trajectory on a Cloth object, run "python demo.py" in the terminal from within the directory containing the scripts.
To run a trial that takes in the physical mouse's location on the canvas as the location of the scissors, run "python demo.py manual" in the terminal from within the directory containing the scripts.

### simulation.py
Contains a simulation object class that can be used for running simulations with different cloth objects. See the main method for example usage.

#### To run:

To run a trial that cuts a predefined trajectory on a CircleCloth object, run "python simulation.py" in the terminal from within the directory containing the scripts.

### environment_rep

A package containing environments defined for various experiments with various frameworks such as RLPy or rllab.

### config_files

A folder containing configuration files that can be used to generate simulation objects. An example is default.json.

### Dependencies

* Python 2.7
* Matplotlib
* Numpy
* Scipy 0.18.0 or newer

### Optional Dependencies

Dependencies that are only required for specific scripts in the repository, but not for core functionality.

* rllab
* RLPy
* OpenAI Gym
* ROS
* dvrk_utils
