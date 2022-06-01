"""
	tvlqr_test.jl 

	This package is used to precompile the sys_image for running tvlqr on A1.
"""

using Pkg; Pkg.activate(".")
# using Revise
using EmbeddedLciMpc
using ContactImplicitMPC
using LinearAlgebra
using JLD2 

CIMPC_path = dirname(pathof(ContactImplicitMPC))
file = JLD2.jldopen(joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/standing_tvlqr_policy_v2.jld2"))

# policy = file["policy"]
K = [0*k for k in file["K"]]
x = file["x"]
u = file["u"]
timestep = file["timestep"]
H = file["H"]
JLD2.close(file)

tvlqr_policy = EmbeddedLciMpc.TVLQRPolicy(K, x, u, timestep, H)

output = EmbeddedLciMpc.exec_policy(tvlqr_policy, zeros(37), 0.0)

println("Finish Loading tvlqr stand")
