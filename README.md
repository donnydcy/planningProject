# planningProject
15-887 Project

Implemented Dijkstra and A* (with Dijkstra heuristic).

![GitHub Logo](https://github.com/donnydcy/planningProject/blob/master/example/planning.png)


# Requirments:
Recommand Anaconda3 for dependencies, pyopengl should be installed separately.

* Python 3.5
* pyopengl
* PyQt4

by using 
```bash
conda install pyopengl
```

The installation of the PyQt4 can be problematic, try this:
```bash
conda install -c anaconda pyqt=4.11.4
```
# Execute the program:
```bash
python visualizer.py
```

# Repository:
* src/prj.py:
	visulization and run main functions

# Maps:
* Load a Map
```python
import numpy as np
mapdata = np.loadtxt('../data/map_random_algo.txt')
```
* Map Files for algortihms:

	map_manhattan.txt
	
	map_random.txt
