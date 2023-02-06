import sys
import os
import glob
import pymoo
import numpy as np
import matplotlib.pyplot as plt
import pickle
import dill
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.termination.max_gen import MaximumGenerationTermination
from pymoo.core.problem import ElementwiseProblem
from pymoo.optimize import minimize
from mesh_gen import create_mesh, get_max_bounds, MIN_SPOKE_WIDTH, MAX_SPOKE_WIDTH
from pymoo.problems import get_problem
from utils import evaluate_mesh_fea,fix_mesh

class MeshOptProblem(ElementwiseProblem):
    def __init__(self, **kwargs):
        
        # GA will modify r, th of polar coords for each spline control point
        # and the width of the spokes.

        # GA will not modify the number of spokes or number of control points.

        # Define number of variables
        # vars = [r1, ..., rn, th1, ..., thn, spoke_width]
        n_var = 2 * num_control_points + 1

        # Define number of outputs
        # obj = [stress, constant_error]
        n_obj = 2

        # Define number of inequality constraints
        n_ieq_constr = 0

        # Define number of equality constraints
        n_eq_constr = 0

        # Define bounds for variables
        low, high = get_max_bounds(num_spokes)
        xl = [low[0]] * num_control_points + [low[1]] * num_control_points + [MIN_SPOKE_WIDTH]
        xu = [high[0]] * num_control_points + [high[1]] * num_control_points + [MAX_SPOKE_WIDTH]

        super().__init__(n_var=n_var,
                         n_obj=n_obj,
                         n_ieq_constr=n_ieq_constr,
                         n_eq_constr=n_eq_constr,
                         xl=xl,
                         xu=xu,
                         **kwargs)

    def _evaluate(self, x, out, *args, **kwargs):
        control_points, spoke_width = convert_1D_to_vars(x, num_control_points)
        mesh_filename="mesh.mesh"
        feasible = create_mesh(control_points, num_spokes, spoke_width, \
                               show_gui=False, verbose=False, filename=mesh_filename)
        if not feasible:
            out["F"] = [np.inf, np.inf]
        else: 
            linear_spring_const,max_stress = evaluate_mesh_fea(mesh_filename, plot_vms=False)
            out["F"] = [abs(linear_spring_const - 5), max_stress]

    # Placeholder function for now
    def evaluate_mesh(self, control_points, spoke_width):
        # Test function incentivizes max spoke width and control points
        return np.sum(control_points) + spoke_width, -np.sum(control_points) - spoke_width

def convert_1D_to_vars(x, num_control_points):
    r = np.array(x[:num_control_points])
    th = np.array(x[num_control_points:-1])
    control_points = np.array([r, th]).T
    spoke_width = x[-1]
    return control_points, spoke_width

def plot_pareto_front(results):
    if not results.X is None:
        plt.scatter(results.F[:,1],results.F[:,0])
        plt.xlabel(r'Max Stress $\frac{N}{cm^2}$')
        plt.ylabel(r'Spring Constant Error $\frac{N}{cm}$')
        plt.title('Pareto Front')
        plt.show()
    else:
        print('Did Not Find Solutions!!')

def evaluate_candidate(candidate, filename, title):
    control_points, spoke_width = convert_1D_to_vars(candidate, num_control_points)
    create_mesh(control_points, num_spokes, spoke_width, show_gui=True, filename=filename)
    linear_spring_const, max_stress = evaluate_mesh_fea(filename, plot_vms=True)
    print(title)
    print("Spring constant Error", linear_spring_const - 5)
    print("Max Stress", max_stress)
    print("----------")

def generate_candidates(results):
    best_spring_constant = results.X[np.argmin(results.F[:,0])]
    evaluate_candidate(best_spring_constant, "best_spring_constant.mesh", "Best Spring Constant Error")
    
    best_constant = results.X[np.argmin(results.F[:,1])]
    evaluate_candidate(best_constant, "best_stress.mesh", "Best Stress")

    best_both = results.X[np.argmin(results.F[:,0] * results.F[:,1])]
    evaluate_candidate(best_both, "best_both.mesh", "Best Both")

def create_folder(folder):
    if not os.path.isdir(folder):
        os.mkdir(folder)
    else:
        files = glob.glob(folder + "/*")
        for f in files:
            os.remove(f)

def write_all_meshes(results, folder_name="pareto_meshes"):
    create_folder(folder_name)
    for x, f in zip(results.X, results.F):
        filename = folder_name + "/mesh_{}_{}.mesh".format(round(f[0], 4), round(f[1], 4)) 
        control_points, spoke_width = convert_1D_to_vars(x, num_control_points)
        create_mesh(control_points, num_spokes, spoke_width, filename=filename)

def write_final_pop(results, folder_name="final_pop_meshes"):
    create_folder(folder_name)
    for pop in results.pop:
        x = pop.get("X")
        f = pop.get("F")
        filename = folder_name + "/mesh_{}_{}.mesh".format(round(f[0], 4), round(f[1], 4)) 
        control_points, spoke_width = convert_1D_to_vars(x, num_control_points)
        create_mesh(control_points, num_spokes, spoke_width, filename=filename)

# This in incorrect
#def write_intermediate_pop(results):
#    intervals = [int(len(results.history) * (1/3)), int(len(results.history) * 2/3)]
#    for i in intervals:
#        algo = results.history[i]
#        res = minimize(problem,
#                       algorithm,
#                       ('n_gen', 1),
#                       copy_algorithm=False,
#                       verbose=True,
#                       save_history=True,
#                       seed=0,
#                       )
#        folder_name = "gen_{}_meshes".format(i)
#        write_final_pop(res, folder_name)

def save_results(results):
    with open("results.obj", 'wb') as f: 
        pickle.dump(results, f)

def load_results():
    with open("results.obj", 'rb') as f:
        results = pickle.load(f)
        return results

def save_checkpoint(algorithm):
    with open("checkpoint", "wb") as f:
        dill.dump(algorithm, f)

def load_checkpoint():
    with open("checkpoint", 'rb') as f:
        checkpoint = dill.load(f)
        print("Loaded Checkpoint:", checkpoint)
        return checkpoint

if __name__=="__main__":
    num_spokes = 4
    num_control_points = 8
    problem = MeshOptProblem()
    algorithm = NSGA2(pop_size=75)
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "view_results":
            results = load_results()
            plot_pareto_front(results)
            generate_candidates(results)
            #write_all_meshes(results)
            #write_final_pop(results)
            #write_intermediate_pop(results)

        elif sys.argv[1] == "resume":
            print("Resuming from checkpoint...")
            checkpoint = load_checkpoint()
            checkpoint.termination = MaximumGenerationTermination(5)
            results = minimize(problem,
                               checkpoint,
                               copy_algorithm=False,
                               verbose=True,
                               save_history=True,
                               seed=0,
                              )
            #save_results(results)
            save_checkpoint(checkpoint)

    else:
        print("Starting from scratch...")
        results = minimize(problem,
                           algorithm,
                           ('n_gen', 500),
                           copy_algorithm=False,
                           verbose=True,
                           save_history=True,
                           seed=0,
                          )
        save_results(results)
        save_checkpoint(algorithm)
    
