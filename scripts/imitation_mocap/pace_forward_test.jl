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
using YAML

vis = false
CIMPC_path = dirname(pathof(ContactImplicitMPC))
config_path = joinpath(@__DIR__, "../config_gazebo/pace_forward.yaml")
config = YAML.load_file(config_path; dicttype= Dict{String, Float64});

# ## Model Initialization 
s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env

# ## Reference Trajectory Generation 
ref_traj = deepcopy(get_trajectory(s.model, s.env,
    joinpath(CIMPC_path, "../examples/A1-imitation/results/pace_forward/run21/pace_forward_tol0.001.jld2"), 
    load_type = :split_traj_alt));

H = ref_traj.H
h = ref_traj.h

# ## MPC setup
N_sample = Int(config["N_sample"])
H_mpc = Int(config["H_mpc"])
h_sim = h / N_sample
H_sim = Int(config["H_sim"])
κ_mpc = config["k_mpc"]

v0 = config["v0"]

# standing velocity tracking 
v_weights = Diagonal([[config["w_v_pos_x"], config["w_v_pos_y"], config["w_v_pos_z"]]; 
                    [config["w_v_ang_z"], config["w_v_ang_z"], config["w_v_ang_z"]]; 
                    fill([config["w_v_ft_x"], config["w_v_ft_y"], config["w_v_ft_z"]], 4)...])
println("here")
q_weights = LciMPC.relative_state_cost([config["w_q_pos_x"], config["w_q_pos_y"], config["w_q_pos_z"]], 
                    [config["w_q_ang_z"], config["w_q_ang_z"], config["w_q_ang_z"]], 
                    [config["w_q_ft_x"], config["w_q_ft_y"], config["w_q_ft_z"]])
println("here")                
u_weights = Diagonal(vcat(fill([config["w_u_1"], config["w_u_2"], config["w_u_3"]], 4)...))
println("here")
obj = TrackingVelocityObjective(model, env, H_mpc,
v = h/H_mpc * [v_weights for t = 1:H_mpc],
q = h/H_mpc * [q_weights for t = 1:H_mpc],
u = h/H_mpc * [u_weights for t = 1:H_mpc],
v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

# obj = TrackingVelocityObjective(model, env, H_mpc,
#     v = h/H_mpc * [Diagonal([[1,1,15]; [6000,6000,800]; 2e-3 * fill([0.7,0.7,1], 4)...]) for t = 1:H_mpc],
#     q = h/H_mpc * [LciMPC.relative_state_cost([5,5,50], [600,600,5], [15,15,30]) for t = 1:H_mpc],
#     # q = h/H_mpc * [Diagonal([[100,100,1000]; [1200,1200,600]; fill([5,5,15], 4)...]) for t = 1:H_mpc],
#     u = h/H_mpc * [Diagonal(9e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
#     v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)


p_pace_forward = ci_mpc_policy(ref_traj, s, obj,
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

println("Finish Loading Pace Forward")

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