using ContactImplicitMPC
using LinearAlgebra
import ContactImplicitMPC as LciMPC
using EmbeddedLciMpc
using LinearAlgebra
using YAML

CIMPC_path = dirname(pathof(ContactImplicitMPC))
config_path = joinpath(@__DIR__, "config/hardware/wall_walk_hardware.yaml")
# config_path = joinpath(@__DIR__, "config/gazebo/wall_walk_gazebo.yaml")
config = YAML.load_file(config_path; dicttype= Dict{String, Float64});

# ## Model Initialization 
s = get_simulation("centroidal_quadruped_wall", "flat_3D_lc", "flat")
s.model.μ_world = 0.8
model = s.model
# s.model.mass_body = 14.5
env = s.env

# # Cut (comment out)
# using JLD2
# @load joinpath(@__DIR__, "../../ContactImplicitMPC.jl/examples/centroidal_quadruped_wall/reference/" * 
#     "stand_wall_two_steps_steep_v11.jld2") qm um γm bm ψm ηm μm hm x_sol u_sol x_ref

#     using Plots
# plot(hcat(qm...)', labels="")
# start = 103
# qm = qm[start:end]
# um = um[start:end]
# γm = γm[start:end]
# bm = bm[start:end]
# ψm = ψm[start:end] 
# ηm = ηm[start:end]
# μm = model.μ_world
# x_sol = x_sol[start:end]
# u_sol = u_sol[start:end]
# x_ref = x_ref[start:end]
# plot(hcat(qm...)', labels="")
# plot(hcat([q[1:6] for q in qm]...)')

# @save joinpath(@__DIR__, "../../ContactImplicitMPC.jl/examples/centroidal_quadruped_wall/reference/" * 
#     "stand_wall_two_steps_steep_v11-cut.jld2") qm um γm bm ψm ηm μm hm x_sol u_sol x_ref


# ## Reference Trajectory Generation 
ref_traj = deepcopy(get_trajectory(s.model, s.env,
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped_wall/reference/wall_walk_inplace-4.jld2"),
    joinpath(CIMPC_path, "../examples/centroidal_quadruped_wall/reference/wall_walk_quasi_10_cm_20_Tm.jld2"),
    load_type = :split_traj_alt))


# fieldnames(typeof(ref_traj))
# fieldnames(typeof(ref_traj))
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
                    [config["w_v_ang_z"], config["w_v_ang_y"], config["w_v_ang_x"]]; 
                    fill([config["w_v_ft_x"], config["w_v_ft_y"], config["w_v_ft_z"]], 4)...])

q_weights = LciMPC.relative_state_cost([config["w_q_pos_x"], config["w_q_pos_y"], config["w_q_pos_z"]], 
            [config["w_q_ang_z"], config["w_q_ang_y"], config["w_q_ang_x"]], 
            [config["w_q_ft_x"], config["w_q_ft_y"], config["w_q_ft_z"]])

u_weights = Diagonal(vcat(fill([config["w_u_1"], config["w_u_2"], config["w_u_3"]], 4)...))

obj = TrackingVelocityObjective(model, env, H_mpc,
v = 2/2*h/H_mpc * [v_weights for t = 1:H_mpc],
q = 2/2*h/H_mpc * [q_weights for t = 1:H_mpc],
u = 2/2*h/H_mpc * [u_weights for t = 1:H_mpc],
# γ = [Diagonal(1e-2 * ones(s.model.nc)) for t = 1:H_mpc],
# b = [Diagonal(1e-2 * ones(s.model.nc * friction_dim(s.env))) for t = 1:H_mpc],
v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

timing = @elapsed p_wall_walk = ci_mpc_policy(ref_traj, s, obj,
    H_mpc = H_mpc,
    N_sample = N_sample,
    κ_mpc = κ_mpc,
    # mode = :configurationforce,
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

output = EmbeddedLciMpc.exec_policy(p_wall_walk, [q1_sim0; v1_sim; zeros(12)], 0.0)

println("Finish loading centroidal wall walk. Total time: ", timing, "s")

