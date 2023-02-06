# Torsion Spring Topology Optimization
This repo contains code for our Machine Learning for Mechanical Design final project. It includes an automated way to generate an FEM mesh of a torsional spring.
The spring's topology is then optimized using NSGA-II to minimize peak stress and minimize error from a desired spring constant.

## Details for reviewer

* This work was done with a partner. In particular, `utils.py` was written by my partner.

* `mesh_gen.py` parameterizes a mesh and creates it using gmsh.

* `ga.py` runs NSGA-II on these parameters to optimize the topology.

* `utils.py` runs finite element analysis on mesh to find peak stress and spring constant.
