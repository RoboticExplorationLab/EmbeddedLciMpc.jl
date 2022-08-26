"""
    pace_forward_test.jl 

    This package is used to precompile the sys_image for running LciMPC.
        It includes all of the necessary function to run the LciMPC with
        a centroidal quadruped model. 
"""

using Pkg; Pkg.activate(".")
using Revise
using EmbeddedLciMpc
using ContactImplicitMPC
import ContactImplicitMPC as LciMPC
using LinearAlgebra


vis = false
CIMPC_path = dirname(pathof(ContactImplicitMPC))

# ## Model Initialization 
s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env

# ## Reference Trajectory Generation 
ref_traj = deepcopy(get_trajectory(s.model, s.env,
    joinpath(CIMPC_path, "../examples/A1-imitation/results/pace_backward/run5/pace_backward_tol0.001.jld2"), 
    load_type = :split_traj_alt));

H = ref_traj.H
h = ref_traj.h

# ## MPC setup
N_sample = 5
H_mpc = 3
h_sim = h / N_sample
H_sim = 320
κ_mpc = 2.0e-4

v0 = 0.0

# standing velocity tracking 
obj = TrackingVelocityObjective(model, env, H_mpc,
    v = [Diagonal(1 * [[1,1,1]; 1e+3*[1,1,1]; fill([1,1,1], 4)...]) for t = 1:H_mpc],
    # q = [LciMPC.relative_state_cost(1*[1e-2,1e-2,10], 1e-1*[1,1,1], 1e-0*[0.2,0.2,1]) for t = 1:H_mpc],
    q = [Diagonal([[0,0,5]; 1e-1*[10,10,1]; fill(1e-0*[0.2,0.2,1], 4)...]) for t = 1:H_mpc],
    # u = [Diagonal(3e-5 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
    u = [Diagonal(3e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
    v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

p_stand = ci_mpc_policy(ref_traj, s, obj,
    H_mpc = H_mpc,
    N_sample = N_sample,
    κ_mpc = κ_mpc,
    mode = :configuration,
    ip_opts = InteriorPointOptions(
                    undercut = 5.0,
                    κ_tol = κ_mpc,
                    r_tol = 1.0e-5, # TODO maybe relax this parameter
                    diff_sol = true,
                    solver = :empty_solver,
                    max_time = 1e5),
    n_opts = NewtonOptions(
        r_tol = 3e-5,
        max_time=1.0e-1,
        solver=:ldl_solver,
        threads=false,
        verbose=false,
        max_iter = 5),
    mpc_opts = CIMPCOptions(
        gains=true
        # live_plotting=true
))

# ## Run a single step 
q1_sim, v1_sim = initial_conditions(ref_traj);
q1_sim0 = deepcopy(q1_sim)
output = EmbeddedLciMpc.exec_policy(p_stand, [q1_sim0; v1_sim; zeros(12)], 0.0)

println("Finish Loading Pace Backward")

# ## Sim and Visualization
if vis 
    vis = ContactImplicitMPC.Visualizer()
    ContactImplicitMPC.open(vis)

    # ## Initial conditions
    q1_sim, v1_sim = initial_conditions(ref_traj);
    q1_sim0 = deepcopy(q1_sim)
    q1_sim0[1:3] = [-0.00 0.00 0.3089]
    q1_sim0[4:6] = [-0.01 -0.05 0.0085]
    q1_sim0[7:9] = [0.2027 0.1399 0.001]
    q1_sim0[10:12] = [0.1279 -0.1246 0.0035]
    q1_sim0[13:15] = [-0.153 0.1216 0.001]
    q1_sim0[16:18] = [-0.178 -0.1196 0.00]
    output = EmbeddedLciMpc.exec_policy(p_stand, [q1_sim0; v1_sim; ones(4)*0], 0.0)

    sim = simulator(s, H_sim, h=h_sim, policy=p_walk);
    LciMPC.RoboDojo.set_state!(sim, q1_sim0, v1_sim, 1)
    LciMPC.RoboDojo.simulate!(sim, q1_sim0, v1_sim) 
    ##
    LciMPC.set_light!(vis)
    LciMPC.set_floor!(vis, grid=true)
    LciMPC.set_background!(vis)
    anim = LciMPC.visualize!(vis, model, sim.traj.q; Δt=h_sim) 
end