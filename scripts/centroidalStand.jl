using Pkg; Pkg.activate(".")
using ContactImplicitMPC
import ContactImplicitMPC as LciMPC
using LinearAlgebra

CIMPC_path = dirname(pathof(ContactImplicitMPC))
# include("continuous_policy.jl")
include("continuous_policy_v2.jl")

s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env

ref_traj = deepcopy(get_trajectory(s.model, s.env,
	# joinpath(module_dir(), "src/dynamics/centroidal_quadruped/gaits/inplace_trot_v4.jld2"),
    joinpath(CIMPC_path, "dynamics/centroidal_quadruped/gaits/stand_v0.jld2"),
    load_type = :split_traj_alt));

H = ref_traj.H
h = ref_traj.h

# ## MPC setup
N_sample = 5
H_mpc = 10
h_sim = h / N_sample
H_sim = 320
κ_mpc = 2.0e-4

v0 = 0.0
obj = TrackingVelocityObjective(model, env, H_mpc,
    v = [Diagonal(1e-3 * [[1,1,1]; 1e+3*[1,1,1]; fill([1,1,1], 4)...]) for t = 1:H_mpc],
	q = [LciMPC.relative_state_cost(1e-0*[1e-2,1e-2,1], 3e-1*[1,1,1], 1e-0*[0.2,0.2,1]) for t = 1:H_mpc],
	u = [Diagonal(3e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
	v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

p = ci_mpc_policy(ref_traj, s, obj,
    H_mpc = H_mpc,
    N_sample = N_sample,
    κ_mpc = κ_mpc,
	mode = :configuration,
	ip_opts = InteriorPointOptions(
					undercut = 5.0,
					κ_tol = κ_mpc,
					r_tol = 1.0e-4, # TODO maybe relax this parameter
					diff_sol = true,
					solver = :empty_solver,
					max_time = 1e5),
    n_opts = NewtonOptions(
        r_tol = 3e-5,
        max_time=1.0e-3,
		solver=:ldl_solver,
        threads=false,
        verbose=false,
        max_iter = 5),
    mpc_opts = CIMPCOptions(
		# live_plotting=true
		));


# ## Disturbances
w = [[0.0,0.0,0.0] for i=1:H_sim/N_sample]
d = open_loop_disturbances(w, N_sample)

# ## Initial conditions
q1_sim, v1_sim = initial_conditions(ref_traj);
sim = simulator(s, H_sim, h=h_sim, policy=p);
q1_sim0 = deepcopy(q1_sim)
# LciMPC.RoboDojo.simulate!(sim, q1_sim0, v1_sim)
simulate!(sim)

# ## Run a single step 
@time exec_policy(p, [zeros(18); zeros(18); zeros(12)], 0.1)


# Visualization
vis = ContactImplicitMPC.Visualizer()
ContactImplicitMPC.open(vis)


## 
LciMPC.set_light!(vis)
LciMPC.set_floor!(vis, grid=true)
LciMPC.set_background!(vis)
anim = LciMPC.visualize!(vis, model, sim.traj.q; Δt=h_sim)