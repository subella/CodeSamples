import numpy as np
import matplotlib.pyplot as plt
from sfepy.discrete.fem import Mesh,FEDomain, Field
from sfepy import data_dir
from sfepy.discrete.fem.utils import refine_mesh
from sfepy.discrete import (FieldVariable,Material,Integral,Function,Equation,Equations,Problem)
from mesh_gen import *
from sfepy.scripts.convert_mesh  import *
from sfepy.scripts.resview import *
import gmsh
import sys
import numpy as np
import meshio
import time
from sfepy.mechanics.matcoefs import stiffness_from_lame,lame_from_youngpoisson,stiffness_from_youngpoisson
from sfepy.terms import Term
from sfepy.discrete.conditions import Conditions,EssentialBC
from sfepy.base.base import IndexedStruct,Struct,output
from sfepy.solvers.ls import ScipyDirect
from sfepy.solvers.nls import Newton
from sfepy.solvers.ts_solvers import SimpleTimeSteppingSolver
from sfepy.mechanics.tensors import get_von_mises_stress
from sfepy.base.base import Struct
import os
import time
factory = gmsh.model.geo
print(data_dir)
# ---------------------------------
# for FEA
output.set_output(quiet=True)
def rotate(ts, coors,bc=None,problem=None,angle_rot = 10):
    from sfepy.linalg import rotation_matrix2d
    vec = coors - np.array([0,0])

    #angle = 27.0 * ts.step
    angle = angle_rot + ts.step
    #print(angle,ts.step)
    mtx = rotation_matrix2d(angle)
    vec_rotated = np.dot(vec, mtx)

    displacement = vec_rotated - vec

    return displacement
    
def stress_strain(out,pb, extend=False):#out, pb, state,
    """
    Calculate and output strain and stress for given displacements.
    """


    ev = pb.evaluate
    strain = ev('ev_cauchy_strain.3.Omega(u)', mode='el_avg')
    stress = ev('ev_cauchy_stress.3.Omega(m.D, u)', mode='el_avg',
                copy_materials=False)
#     out['cauchy_strain'] = Struct(name='output_data', mode='cell',
#                                   data=strain, dofs=None)
#     out['cauchy_stress'] = Struct(name='output_data', mode='cell',
#                                   data=stress, dofs=None)
    return stress,strain

def compute_von_mises(out, pb,stress=None, extend=False, wmag=None, wdir=None,view = True):
    """
    Calculate the von Mises stress.
    """
    #stress = pb.evaluate('ev_cauchy_stress.2.Omega(m.D, u)', mode='el_avg')
#     if stress == None:
#         stress,strain = stress_strain(out,pb)
    stress,strain = stress_strain(out,pb)
    #print(stress)
    vms = get_von_mises_stress(stress.squeeze())
    vms.shape = (vms.shape[0], 1, 1, 1)
    if not view:
        return vms
    out['von_mises_stress'] = Struct(name='output_data', mode='cell',data=vms)

    return out
def get_outer_points_region(coors,domain=None,outer_radius=0.5,eps = .000001):
    x,y = coors[:,0],coors[:,1]
    r = np.sqrt(x**2 + y**2)
    flag = np.where((r>(outer_radius-.05)))[0]

    return flag

def get_fixed_points_region(coors,domain=None,inner_radius=0.5):
    x,y = coors[:,0],coors[:,1]
    r = np.sqrt(x**2 + y**2)
    #plt.plot(range(len(r)),r)
    flag = np.where(r<inner_radius)[0]
    return flag

def set_boundary_cond(angle,center,outer_c):
    # set boundary condition for fixed center AND ROTATED OUTER RING
    fix_u = EssentialBC('fix_u',center,{'u.all':0.0}) #fix the center region
    rotate_fun = Function('apply_outer_torque_y',rotate,extra_args={'angle_rot':angle})
    shift_u = EssentialBC('outer_torque',outer_c,{'u.all':rotate_fun})
    return fix_u,shift_u
    
    

def set_up(filename_mesh_usable,refinement_level=0,inner_radius=.6):
    filename_mesh = filename_mesh_usable
    refinement_level = 0
    filename_mesh = refine_mesh(filename_mesh, refinement_level)
    mesh = Mesh.from_file(filename_mesh)
    #MAKE DOMAIN
    domain = FEDomain('domain',mesh)
    min_x, max_x = domain.get_mesh_bounding_box()[:,0]
    diameter = domain.get_diameter()
    outer_radius = diameter/2

    #DEFINE REGIONS
    eps = 1e-8 * (max_x - min_x)
    outer_c_func = Function('get_outer_points_region',get_outer_points_region,extra_args={'outer_radius':outer_radius,'eps':eps})
    inner_fixed_func = Function('get_fixed_points_region',get_fixed_points_region,extra_args={'inner_radius':inner_radius})
    center =domain.create_region('center','vertices by get_fixed_points_region','facet',functions={'get_fixed_points_region':inner_fixed_func})
    outer_c = domain.create_region('outer_c','vertices by outer_c_func','facet',functions={'outer_c_func':outer_c_func})
    omega = domain.create_region('Omega','all')
    #DEFINE OTHER THINGS:
    field = Field.from_args('fu',np.float64,'vector',omega,approx_order=2)
    # define unknown and test variables
    u = FieldVariable('u','unknown',field)
    v = FieldVariable('v','test',field,primary_var_name = 'u')
    
    #define material properties
    m = Material('m', D=stiffness_from_youngpoisson(mesh.dim, young = 200, poisson = 0.25))
    stiff_matrix = stiffness_from_youngpoisson(mesh.dim, young = 200, poisson = 0.25)
    f = Material('f',val=[[0.0],[0.0]])
    # define quadrature order for integral
    integral = Integral('i',order = 3)
    # Define terms and build equations
    
    t1 = Term.new('dw_lin_elastic(m.D,v,u)',integral,omega,m=m,v=v,u=u)
    t2 = Term.new('dw_volume_lvf(f.val,v)',integral,outer_c,f=f,v=v)
    eq = Equation('balance',t1+t2)
    eqs = Equations([eq])
    
    
    #SET UP SOLVER
    ls = ScipyDirect({})
    nls_status = IndexedStruct()
    nls = Newton({},lin_solver=ls,status=nls_status)
    ts_solver = SimpleTimeSteppingSolver({},nls=nls,ts=.01,post_process = 'post_process_hook',verbose=0)
    return ls,nls_status,nls,ts_solver,eqs,center,outer_c,inner_radius

def run_fea_sim_for_diff_displacements(angles_to_sim,ls,nls_status,nls,ts_solver,eqs,center,outer_c,inner_radius,plot_forces=False,plot_vms=False,filename_stress=None):
    torques_final = []
    stresses = []
    for angle in angles_to_sim:
        #filename_mesh = data_dir + '/meshes/2d/circle_sym.mesh'

        #DEFINE PROBLEM AND SOLVE
        fix_u,shift_u = set_boundary_cond(angle,center,outer_c)
        st_fun = Function(name = 'vms',function=compute_von_mises)
        pb = Problem('elasticity',equations=eqs,functions=st_fun)
        pb.save_regions_as_groups('regions')
        pb.set_bcs(ebcs=Conditions([fix_u,shift_u]))
        pb.set_solver(nls)
        pb.setup_hooks(st_fun)
        variables = pb.solve(verbose=False)
        
        #COMPUTE VMS
        vms = compute_von_mises(variables,pb,view=False)
        if plot_vms:
            if filename_stress != None:
                pb.save_state(f'{filename_stress}.vtk',variables,post_process_hook = compute_von_mises)
                os.system(f'sfepy-view {filename_stress}.vtk -2 --max-plots 2')# see stress

                #os.system('sfepy-view linear_elasticity.vtk -f u:wu:f0.5 1:vw') # plot deformation?
            #!sfepy-view linear_elasticity.vtk -2 --max-plots 2 # see stress
        real_vms = []
        for i, tensor in enumerate(vms):
            real_vms.append(tensor[0][0][0])
        max_stress = max(real_vms)
        stresses.append(max_stress)
        
        #compute forces and resultant moment
        u = variables.get_state_parts()['u']
        K= pb.mtx_a
        pb.remove_bcs()
        f = pb.evaluator.eval_residual(u)
        if plot_forces:
            # CODE TO PLOT FORCES
            fvars = variables.copy() # Shallow copy, .vec is shared!
            fvars.init_state(f) # Initialize new .vec with f.
            out = fvars.create_output()
            pb.save_state('force.vtk', out=out)
            #!sfepy-view force.vtk
        pb.time_update()
        f_og_shape = f.shape
        f_pairs = f.copy()
        f_pairs.shape = (int(len(f)/2),2)
        torques = []
        bad_counter = 0
        for i,pair in enumerate(f_pairs):
            mag = np.sqrt(pair[0]**2 + pair[1]**2)
            if mag >.001:
                torques.append(mag* inner_radius)
        resultant_moment =sum(torques)
        torques_final.append(resultant_moment)
        #print("max_stress=",max_stress)
    return torques_final,stresses

def calc_final_results(torques,angles_to_sim,stresses,plot = False,rad = False,verbose=False):
    #calculate spring constant from simluations
    if rad:
        rotations = np.deg2rad(angles_to_sim)
    else:
        rotations = angles_to_sim
    # Generate data
    x = [0] + rotations
    y = [0] + torques
    # Fit linear regression via least squares with numpy.polyfit
    # It returns an slope (b) and intercept (a)
    # deg=1 means linear fit (i.e. polynomial of degree 1)
    m, b = np.polyfit(x, y, deg=1) #linear
    #a,b,c = np.polyfit(x, y, deg=2)#quadratic
    max_stress = max(stresses)
    #print("spring constant=",m)
    if plot:
        # Plot regression line
        # Initialize layout
        # Create sequence of 100 numbers from 0 to 100 
        xseq = np.linspace(0, 10, num=100)
        fig, ax = plt.subplots(figsize = (9, 9))
        # Add scatterplot
        ax.scatter(x, y, s=60, alpha=0.7, edgecolors="k")
        ax.plot(xseq, (m * xseq) +b, color="k", lw=2.5);
        #ax.plot(xseq, (a*(xseq**2))+(b * xseq) +c, color="k", lw=2.5); #
        plt.show()
    if verbose:
        print('linear spring constant =',m)
        print('peak_stress =',max(stresses))
        #print('quadratic terms: a =',a,'b=',b,'c=',c)
        
    return m,max_stress
def evaluate_mesh_fea(filename_mesh_usable,inner_radius=.6,angles_to_sim = [10],rad=False,plot_torque_disp_curve=False,timer = False,plot_forces=False,plot_vms=False,filename_stress="mesh"):
    if timer:
        start_t = time.time()
        ls,nls_status,nls,ts_solver,eqs,center,outer_c,inner_radius = set_up(filename_mesh_usable,refinement_level=0,inner_radius=inner_radius)
        torques,stresses = run_fea_sim_for_diff_displacements(angles_to_sim,ls,nls_status,nls,ts_solver,
                                                              eqs,center,outer_c,inner_radius,
                                                              plot_forces=plot_forces,plot_vms=plot_vms,filename_stress=filename_stress)
        m,max_stress = calc_final_results(torques,angles_to_sim,stresses,plot = plot_torque_disp_curve,rad = rad,verbose=True)
        end_t = time.time()
        print('Sim time alone:',end_t-start_t)
    else:
        ls,nls_status,nls,ts_solver,eqs,center,outer_c,inner_radius = set_up(filename_mesh_usable,refinement_level=0,inner_radius=.6)
        torques,stresses = run_fea_sim_for_diff_displacements(angles_to_sim,ls,nls_status,nls,ts_solver,
                                                              eqs,center,outer_c,inner_radius,
                                                              plot_forces=plot_forces,plot_vms=plot_vms,filename_stress=filename_stress)
        m,max_stress = calc_final_results(torques,angles_to_sim,stresses,plot = plot_torque_disp_curve,rad = rad,verbose=False)
    return m,max_stress

# ---------------------------------
# for meshing
def make_random_mesh(num_div = 4,show_preview=False):
    start = time.time()
    gmsh.initialize()
    

    # Mesh element size
    lc = 0.1

    # Inner radius
    r1 = 1
    # Inner radius of rim
    r2 = 3
    # Outer radius of rim
    r3 = 4

    create_random_mesh(r1, r2, r3)

    factory.synchronize()
    gmsh.model.mesh.generate()
    gmsh.write("t4.mesh")
    # Launch the GUI to see the results:
    if show_preview:
        if '-nopopup' not in sys.argv:
            gmsh.fltk.run()
    gmsh.finalize()
    end = time.time()
    print("Total time", end - start)

def make_new_rand_mesh_and_fix(num_div = 5,show_preview=False):
    make_random_mesh(num_div = num_div)
    #!python3 /Users/bookerschelhaas/miniconda3/envs/ai-design/lib/python3.11/site-packages/sfepy/scripts/convert_mesh.py -2 t4.mesh new_spring.mesh
    #!python3 convert_mesh.py -2 t4.mesh new_spring.mesh

def fix_mesh(mesh_file_name):
    os.system(f'python3 convert_mesh.py -2 {mesh_file_name} {mesh_file_name}')

def evaluate_random_mesh():
    filename = "mesh.mesh"
    create_random_mesh()
    linear_spring_const,max_stress = evaluate_mesh_fea(filename, plot_vms=False)
    print("Constant", linear_spring_const)
    print("Max Stress", max_stress)

if __name__ == "__main__":
    evaluate_random_mesh()



