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
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/inplace_trot_v8.jld2"),
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/inplace_trot_v6.jld2"),
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/inplace_trot_10Hz_double.jld2"),
	joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/inplace_trot_20Hz.jld2"),
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/one_foot_up_10Hz.jld2"),
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/inplace_crawl_v10.jld2"),
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/inplace_crawl_v10.jld2"),
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/cra.jld2"),
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/cra1.jld2"),
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/stand_0.1.jld2"), 
	# joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/inplace_trot_v5.jld2"), 
	# joinpath(CIMPC_path, "dynamics/centroidal_quadruped/gaits/inplace_trot_v4.jld2"),
    # joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/stand001.jld2"),
    load_type = :split_traj_alt))

println(CIMPC_path)
fieldnames(typeof(ref_traj))
fieldnames(typeof(ref_traj))


H = ref_traj.H
h = ref_traj.h

# # ## MPC setup
# N_sample = 5
# H_mpc = 2
# h_sim = h / N_sample
# H_sim = 1000
# κ_mpc = 1.0e-5

# # N_sample = 5
# # H_mpc = 10
# # h_sim = h / N_sample
# # H_sim = 320
# # κ_mpc = 2.0e-4

# v0 = 0.0#-0.2

# ## best so far
# obj = TrackingVelocityObjective(model, env, H_mpc,
# v = h/H_mpc * [Diagonal([[0.8,0.8,10]; [6000,6000,8000]; 2e-4 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
# q = h/H_mpc * [LciMPC.relative_state_cost([0,0,600], [1800,1500,600], [15,15,30]) for t = 1:H_mpc],
# u = h/H_mpc * [Diagonal(6e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
# v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

# p_walk = ci_mpc_policy(ref_traj, s, obj,
#     H_mpc = H_mpc,
#     N_sample = N_sample,
#     κ_mpc = κ_mpc,
# 	mode = :configuration,
# 	ip_opts = InteriorPointOptions(
# 					undercut = 5.0,
# 					κ_tol = κ_mpc,
# 					r_tol = 1.0e-5, # TODO maybe relax this parameter
# 					diff_sol = true,
# 					solver = :empty_solver,
# 					max_time = 0.1),
#     n_opts = NewtonOptions(
#         r_tol = 3e-5,
#         max_time=1.0e-1,
# 		solver=:ldl_solver,
#         threads=false,
#         verbose=false,
#         max_iter = 5),
#     mpc_opts = CIMPCOptions(
# 		# live_plotting=true
# 		));

# # ## MPC setup
# N_sample = 7
# H_mpc = 15
# h_sim = h / N_sample
# H_sim = 1000
# κ_mpc = 2.0e-4

# v0 = 0.0
# obj = TrackingVelocityObjective(model, env, H_mpc,
# v = h/H_mpc * [Diagonal([[0.8,0.8,10]; [6000,6000,8000]; 2e-4 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
# q = h/H_mpc * [Diagonal([[10,10,100]; [1800,1500,600]; fill([15,15,400], 4)...]) for t = 1:H_mpc],
# u = h/H_mpc * [Diagonal(1e-2 * vcat(fill([1,1,1.0e-1], 4)...)) for t = 1:H_mpc],
# v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

# # v0 = 0.0
# # obj = TrackingVelocityObjective(model, env, H_mpc,
# # 	v = h/H_mpc * [Diagonal([[1,1,1]; [10,10,10]; 1e-1 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
# # 	q = h/H_mpc * [Diagonal([[0.3,0.3,3]; [30,30,30]; fill([3,3,30.0], 4)...]) for t = 1:H_mpc],
# # 	u = h/H_mpc * [Diagonal(3e-3 * vcat(fill([1,1,3e-1], 4)...)) for t = 1:H_mpc],
# # 	v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

# p_walk = ci_mpc_policy(ref_traj, s, obj,
#     H_mpc = H_mpc,
#     N_sample = N_sample,
#     κ_mpc = κ_mpc,
# 	mode = :configuration,
# 	ip_opts = InteriorPointOptions(
# 					undercut = 5.0,
# 					κ_tol = κ_mpc,
# 					r_tol = 1.0e-5, # TODO maybe relax this parameter
# 					diff_sol = true,
# 					solver = :empty_solver,
# 					max_time = 0.1),
#     n_opts = NewtonOptions(
#         r_tol = 3e-5,
#         max_time=1.0e-2,
# 		solver=:ldl_solver,
#         threads=false,
#         verbose=false,
#         # max_iter = 2),################################################
#         max_iter = 1),
#     mpc_opts = CIMPCOptions(
# 		# live_plotting=true
# 		));


# ############################################################################################
# # # one leg up
# # ############################################################################################

# # ## MPC setup
# N_sample = 5
# H_mpc = 8
# h_sim = h / N_sample
# H_sim = 320
# κ_mpc = 2.0e-4

# v0 = 0.0
# obj = TrackingVelocityObjective(model, env, H_mpc,
#     v = [Diagonal(1e-2 * [[1,1,1]; 1e+3*[1,1,1]; fill([1,1,1], 4)...]) for t = 1:H_mpc],
# 	q = [LciMPC.relative_state_cost(1*[1e-2,1e-2,10], 1e-1*[1,1,1], 1e-0*[0.2,0.2,1]) for t = 1:H_mpc],
# 	u = [Diagonal(3e-5 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
# 	v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

# p_walk = ci_mpc_policy(ref_traj, s, obj,
#     H_mpc = H_mpc,
#     N_sample = N_sample,
#     κ_mpc = κ_mpc,
# 	mode = :configuration,
# 	ip_opts = InteriorPointOptions(
# 					undercut = 5.0,
# 					κ_tol = κ_mpc,
# 					r_tol = 1.0e-5, # TODO maybe relax this parameter
# 					diff_sol = true,
# 					solver = :empty_solver,
# 					max_time = 1e5),
#     n_opts = NewtonOptions(
#         r_tol = 3e-5,
#         max_time=1.0e-1,
# 		solver=:ldl_solver,
#         threads=false,
#         verbose=false,
#         max_iter = 5),
#     mpc_opts = CIMPCOptions(
# 		# live_plotting=true
# ));



# ###########################################################################################
# # trotting gait BEST SO FAR
# ###########################################################################################

# # ## MPC setup
# N_sample = 5
# H_mpc = 2
# h_sim = h / N_sample
# H_sim = 320
# κ_mpc = 2.0e-4

# v0 = 0.2
# obj = TrackingVelocityObjective(model, env, H_mpc,
# v = h/H_mpc * [Diagonal([[1,1,15]; [6000,6000,8000]; 2e-3 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
# q = h/H_mpc * [LciMPC.relative_state_cost([0,0,1000], [1200,1200,1200], [4,4,20]) for t = 1:H_mpc],
# u = h/H_mpc * [Diagonal(9e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
# v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)
# # v_target = [1/ref_traj.h * [0;v0;0; 0;0;0; 0;v0;0; 0;v0;0; 0;v0;0; 0;v0;0] for t = 1:H_mpc],)


# # velocity tracking 
# # v0 = 0.2
# # obj = TrackingVelocityObjective(model, env, H_mpc,
# # v = h/H_mpc * [Diagonal([[1,1,15]; [6000,6000,8000]; 2e-4 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
# # q = h/H_mpc * [LciMPC.relative_state_cost([0,0,1000], [1200,1200,1200], [4,4,20]) for t = 1:H_mpc],
# # u = h/H_mpc * [Diagonal(6e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
# # v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

# # obj = TrackingVelocityObjective(model, env, H_mpc,
# #     # v = [Diagonal(1e-3 * [[1,1,1]; 1e+3*[1,1,1]; fill([1,1,1], 4)...]) for t = 1:H_mpc],
# # 	v = [Diagonal([2e+1*[1,1,1]; 1e+3*[1,1,1]; fill(1e0*[1,1,0.3], 4)...]) for t = 1:H_mpc],
# # 	q = [Diagonal([1e-0*[1e-1,1e-1,1]; 3e-0*[1,1,1]; fill(1e+0*[0.2,0.2,20], 4)...]) for t = 1:H_mpc],
# # 	u = [Diagonal(1e-4 * vcat(fill([1,1,0.1], 4)...)) for t = 1:H_mpc],
# # 	v_target = [ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],
# # 	)

# p_walk = ci_mpc_policy(ref_traj, s, obj,
#     H_mpc = H_mpc,
#     N_sample = N_sample,
#     κ_mpc = κ_mpc,
# 	mode = :configuration,
# 	ip_opts = InteriorPointOptions(
# 					undercut = 5.0,
# 					κ_tol = κ_mpc,
# 					r_tol = 1.0e-5, # TODO maybe relax this parameter
# 					diff_sol = true,
# 					solver = :empty_solver,
# 					max_time = 1e5),
#     n_opts = NewtonOptions(
#         r_tol = 3e-5,
#         max_time=0.01,
# 		solver=:ldl_solver,
#         threads=false,
#         verbose=false,
#         max_iter = 5),
#     # mpc_opts = CIMPCOptions(
# 	# 	gains=true,
# 	# 	# live_plotting=true,
# 	# )
# )




###########################################################################################
# trotting gait EXPERIMENTING with 20Hz and double length
###########################################################################################

# ## MPC setup
N_sample = 5
H_mpc = 2
h_sim = h / N_sample
H_sim = 320
κ_mpc = 1.0e-5

# v0 = -0.1
# obj = TrackingVelocityObjective(model, env, H_mpc,
# v = h/H_mpc * [Diagonal([[1,1,15]; [6000,6000,800]; 2e-3 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
# q = h/H_mpc * [LciMPC.relative_state_cost([30,30,1000], [1200,1200,1200], [2,2,20]) for t = 1:H_mpc],
# u = h/H_mpc * [Diagonal(9e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
# v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)
# v_target = [1/ref_traj.h * [0;v0;0; 0;0;0; 0;v0;0; 0;v0;0; 0;v0;0; 0;v0;0] for t = 1:H_mpc],)


# velocity tracking 
# v0 = 0.2
# obj = TrackingVelocityObjective(model, env, H_mpc,
# v = h/H_mpc * [Diagonal([[1,0.5,15]; [6000,6000,8000]; 2e-3 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
# q = h/H_mpc * [LciMPC.relative_state_cost([0,0,1000], [1200,1200,1200], [4,4,20]) for t = 1:H_mpc],
# u = h/H_mpc * [Diagonal(9e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
# v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

v0 = 0.0
obj = TrackingVelocityObjective(model, env, H_mpc,
v = h/H_mpc * [Diagonal([[10,20,15]; [6000,6000,800]; 2e-3 * fill([0.7,0.7,1], 4)...]) for t = 1:H_mpc],
q = h/H_mpc * [LciMPC.relative_state_cost([0,0,1000], [1200,1200,600], [5,5,15]) for t = 1:H_mpc],
u = h/H_mpc * [Diagonal(9e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)
# v_target = [1/ref_traj.h * [0;v0;0; 0;0;0; 0;v0;0; 0;v0;0; 0;v0;0; 0;v0;0] for t = 1:H_mpc],)


# velocity tracking 
# v0 = 0.2
# obj = TrackingVelocityObjective(model, env, H_mpc,
# v = h/H_mpc * [Diagonal([[1,1,15]; [6000,6000,8000]; 2e-4 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
# q = h/H_mpc * [LciMPC.relative_state_cost([0,0,1000], [1200,1200,1200], [4,4,20]) for t = 1:H_mpc],
# u = h/H_mpc * [Diagonal(6e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
# v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

# obj = TrackingVelocityObjective(model, env, H_mpc,
#     # v = [Diagonal(1e-3 * [[1,1,1]; 1e+3*[1,1,1]; fill([1,1,1], 4)...]) for t = 1:H_mpc],
# 	v = [Diagonal([2e+1*[1,1,1]; 1e+3*[1,1,1]; fill(1e0*[1,1,0.3], 4)...]) for t = 1:H_mpc],
# 	q = [Diagonal([1e-0*[1e-1,1e-1,1]; 3e-0*[1,1,1]; fill(1e+0*[0.2,0.2,20], 4)...]) for t = 1:H_mpc],
# 	u = [Diagonal(1e-4 * vcat(fill([1,1,0.1], 4)...)) for t = 1:H_mpc],
# 	v_target = [ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],
# 	)

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
        max_time=0.01,
		solver=:ldl_solver,
        threads=false,
        verbose=false,
        max_iter = 2),
    # mpc_opts = CIMPCOptions(
	# 	gains=true,
	# 	# live_plotting=true,
	# )
)



# ###########################################################################################
# # crawling gait
# ###########################################################################################

# # ## MPC setup
# N_sample = 5
# H_mpc = 2
# h_sim = h / N_sample
# H_sim = 320
# κ_mpc = 2.0e-4

# v0 = 0.0
# obj = TrackingVelocityObjective(model, env, H_mpc,
# v = h/H_mpc * [Diagonal([[1,1,15]; [6000,6000,8000]; 2e-3 * fill([1,1,1], 4)...]) for t = 1:H_mpc],
# q = h/H_mpc * [Diagonal([[1,1,100]; [1200,1200,1200]; fill([4,4,100], 4)...]) for t = 1:H_mpc],
# u = h/H_mpc * [Diagonal(9e-3 * vcat(fill([1,1,1], 4)...)) for t = 1:H_mpc],
# v_target = [1/ref_traj.h * [v0;0;0; 0;0;0; v0;0;0; v0;0;0; v0;0;0; v0;0;0] for t = 1:H_mpc],)

# p_walk = ci_mpc_policy(ref_traj, s, obj,
#     H_mpc = H_mpc,
#     N_sample = N_sample,
#     κ_mpc = κ_mpc,
# 	mode = :configuration,
# 	ip_opts = InteriorPointOptions(
# 					undercut = 5.0,
# 					κ_tol = κ_mpc,
# 					r_tol = 1.0e-5, # TODO maybe relax this parameter
# 					diff_sol = true,
# 					solver = :empty_solver,
# 					max_time = 1e5),
#     n_opts = NewtonOptions(
#         r_tol = 3e-5,
#         max_time=0.01,
# 		solver=:ldl_solver,
#         threads=false,
#         verbose=false,
#         max_iter = 5),
#     mpc_opts = CIMPCOptions(
# 		gains=true
# 		# live_plotting=true
# 	)
# )


############################################################################################
############################################################################################

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

