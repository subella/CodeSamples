# Quadrotor Linear Model Predictive Control
This repo contains code for our Underactuated final project. It uses a linearized model predictive controller to track a moving target with noisy estimates.

## Details for reviewer

* This work was done with a partner, but the code is mostly mine.

* `mpc.py` has everything needed to model, control, and simulate LMPC.

* `vehicle_dynamics.py` contains some transition matrices for the car.

* `vechicle_prediction.py` implements an EKF and polynomial regression to make predictions on car's future state under noise.

* `6832_Final_Report.pdf` details our approach.
