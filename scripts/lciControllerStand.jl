using Revise
using Pkg; Pkg.activate(".")
using EmbeddedLciMpc

## Instantiate Models and Environments 
EmbeddedLciMpc.exec_policy

ref_traj = deepcopy(get_trajectory(s.model, s.env,
	# joinpath(module_dir(), "src/dynamics/centroidal_quadruped/gaits/inplace_trot_v4.jld2"),
    joinpath(CIMPC_path, "dynamics/centroidal_quadruped/gaits/stand_v0.jld2"),
    load_type = :split_traj_alt));

EmbeddedLciMpc.exec_policy(p, [zeros(18); zeros(18); zeros(4)], 0.0)