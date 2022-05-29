using ContactImplicitMPC
using LinearAlgebra
import ContactImplicitMPC as LciMPC
using EmbeddedLciMpc


isVis = false
s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env

CIMPC_path = dirname(pathof(ContactImplicitMPC))
ref_traj = deepcopy(get_trajectory(s.model, s.env,
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/one_foot_hold.jld2"),
    # joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/one_foot_up01.jld2"),
	joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/inplace_trot_v6.jld2"),
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/inplace_trot_v5.jld2"), 
	# joinpath(CIMPC_path, "dynamics/centroidal_quadruped/gaits/inplace_trot_v4.jld2"),
    # joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/stand005.jld2"),
    load_type = :split_traj_alt));

println(CIMPC_path)

H = ref_traj.H
h = ref_traj.h

# ## MPC setup
N_sample = 5
H_mpc = 2
h_sim = h / N_sample
H_sim = 1000
κ_mpc = 1.0e-5

# N_sample = 5
# H_mpc = 10
# h_sim = h / N_sample
# H_sim = 320
# κ_mpc = 2.0e-4

v0 = 0.0
# obj = TrackingVelocityObjective(model, env, H_mpc,
#     v = [Diagonal(1e-2 * [[3,3,10]; 1e3*[15,15,15]; 1e-3 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
# 	q = [LciMPC.relative_state_cost(1*[5e-1,5e-1,50], 1e-1*[150,150,10], 1e-0*[0.2,0.2,1]) for t = 1:H_mpc],
# 	u = [Diagonal(3e-4 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
# 	v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

obj = TrackingVelocityObjective(model, env, H_mpc,
v = h/H_mpc * [Diagonal([[0,0,15]; [6000,6000,8000]; 2e-4 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
q = h/H_mpc * [LciMPC.relative_state_cost([100,100,1000], [1200,1200,1200], [4,4,20]) for t = 1:H_mpc],
u = h/H_mpc * [Diagonal(6e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

p_walk = ci_mpc_policy(ref_traj, s, obj,
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
        max_time=5.0e-1,
		solver=:ldl_solver,
        threads=false,
        verbose=false,
        max_iter = 5),
    mpc_opts = CIMPCOptions(
		# live_plotting=true
		));

q1_sim, v1_sim = initial_conditions(ref_traj);
q1_sim0 = deepcopy(q1_sim)
output = EmbeddedLciMpc.exec_policy(p_walk, [q1_sim0; v1_sim; zeros(12)], 0.0)

println("Finish loading centroidal Trot")

if isVis 
	vis = ContactImplicitMPC.Visualizer()
	ContactImplicitMPC.open(vis)

	# ## Initial conditions
	q1_sim, v1_sim = initial_conditions(ref_traj);
	q1_sim0 = deepcopy(q1_sim)
	sim = simulator(s, H_sim, h=h_sim, policy=p_walk);
	q1_sim0[4] = 0.1
	q1_sim0[1] = 0.05
	q1_sim0[2] = -0.1
	LciMPC.RoboDojo.set_state!(sim, q1_sim0, v1_sim, 1)
	LciMPC.RoboDojo.simulate!(sim, q1_sim0, v1_sim)	

	##
	LciMPC.set_light!(vis)
	LciMPC.set_floor!(vis, grid=true)
	LciMPC.set_background!(vis)
	anim = LciMPC.visualize!(vis, model, sim.traj.q; Δt=h_sim) 
end 

