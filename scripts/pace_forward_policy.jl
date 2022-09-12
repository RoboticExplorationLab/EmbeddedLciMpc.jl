using ContactImplicitMPC
using LinearAlgebra
import ContactImplicitMPC as LciMPC
using EmbeddedLciMpc
using LinearAlgebra
using YAML

CIMPC_path = dirname(pathof(ContactImplicitMPC))
config_path = joinpath(@__DIR__, "config/pace_forward_gazebo.yaml")
# config_path = joinpath(@__DIR__, "config/trot_gazebo_test.yaml")
config = YAML.load_file(config_path; dicttype= Dict{String, Float64});

# ## Model Initialization 
s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env

# ## Reference Trajectory Generation 
ref_traj = deepcopy(get_trajectory(s.model, s.env,
joinpath(CIMPC_path, "../examples/A1-imitation/results/pace_forward/run1/pace_forward_tol0.001.jld2"), 
                load_type = :split_traj_alt));

H = ref_traj.H
h = ref_traj.h

# load params 

# ## MPC setup
N_sample = Int(config["N_sample"])
H_mpc = Int(config["H_mpc"])
κ_mpc = config["κ_mpc"]
h_sim = h / N_sample
H_sim = Int(config["H_sim"])
v0 = config["v0"]
# IP options 
κ_mpc_ip = config["κ_mpc_ip"]
r_tol_ip = config["r_tol_ip"]
undercut_ip = config["undercut_ip"]
max_time_ip = config["max_time_ip"]
# Newton Options 
r_tol_nt = config["r_tol_nt"]
max_time_nt = config["max_time_nt"]
max_iter_nt = Int(config["max_iter_nt"])

# tracking objective 
# standing velocity tracking 
v_weights = Diagonal([[config["w_v_pos_x"], config["w_v_pos_y"], config["w_v_pos_z"]]; 
                    [config["w_v_ang_z"], config["w_v_ang_z"], config["w_v_ang_z"]]; 
                    fill([config["w_v_ft_x"], config["w_v_ft_y"], config["w_v_ft_z"]], 4)...])

q_weights = LciMPC.relative_state_cost([config["w_q_pos_x"], config["w_q_pos_y"], config["w_q_pos_z"]], 
            [config["w_q_ang_z"], config["w_q_ang_z"], config["w_q_ang_z"]], 
            [config["w_q_ft_x"], config["w_q_ft_y"], config["w_q_ft_z"]])

u_weights = Diagonal(vcat(fill([config["w_u_1"], config["w_u_2"], config["w_u_3"]], 4)...))

obj = TrackingVelocityObjective(model, env, H_mpc,
v = h/H_mpc * [v_weights for t = 1:H_mpc],
q = h/H_mpc * [q_weights for t = 1:H_mpc],
u = h/H_mpc * [u_weights for t = 1:H_mpc],
v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

p_pace_forward = ci_mpc_policy(ref_traj, s, obj,
    H_mpc = H_mpc,
    N_sample = N_sample,
    κ_mpc = κ_mpc,
    mode = :configuration,
    ip_opts = InteriorPointOptions(
                    undercut = undercut_ip,
                    κ_tol = κ_mpc_ip,
                    r_tol = r_tol_ip, # TODO maybe relax this parameter
                    diff_sol = true,
                    solver = :empty_solver,
                    max_time = max_time_ip),
    n_opts = NewtonOptions(
        r_tol = r_tol_nt,
        max_time= max_time_nt,
        solver=:ldl_solver,
        threads=false,
        verbose=false,
        max_iter = max_iter_nt),
#     mpc_opts = CIMPCOptions(
#         gains=true
#         # live_plotting=true
# )
)

# ## Run a single step 
q1_sim, v1_sim = initial_conditions(ref_traj);
q1_sim0 = deepcopy(q1_sim)
output = EmbeddedLciMpc.exec_policy(p_pace_forward, [q1_sim0; v1_sim; zeros(12)], 0.0)

println("Finish Loading Centroidal Trot")

