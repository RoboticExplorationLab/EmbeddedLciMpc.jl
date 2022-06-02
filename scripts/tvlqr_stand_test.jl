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
file = JLD2.jldopen(joinpath(CIMPC_path, "../examples/centroidal_quadruped/reference/trotting_tvlqr_policy_v2.jld2"))

# policy = file["policy"]
K = 0.00 * file["K"]
x = file["x"]
u = 0.01 * file["u"]
timestep = file["timestep"]
H = file["H"]
JLD2.close(file)


# standing gains
Kp1 = 70
Kd1 = 20
Kp2 = 180
Kd2 = 20
Kp3 = 300
Kd3 = 20

# trotting gains
Kp1 = 70
Kd1 = 20
Kp2 = 180
Kd2 = 20
Kp3 = 300
Kd3 = 20

for k in K
	k[7,13] = Kp1 # Kp
	k[7,14] = Kd1 # Kd
	k[8,15] = Kp2 # Kp
	k[8,16] = Kd2 # Kd
	k[9,17] = Kp3 # Kp
	k[9,18] = Kd3 # Kd
	
	k[10,19] = Kp1 # Kp
	k[10,20] = Kd1 # Kd
	k[11,21] = Kp2 # Kp
	k[11,22] = Kd2 # Kd
	k[12,23] = Kp3 # Kp
	k[12,24] = Kd3 # Kd
	
	k[13,25] = Kp1 # Kp
	k[13,26] = Kd1 # Kd
	k[14,27] = Kp2 # Kp
	k[14,28] = Kd2 # Kd
	k[15,29] = Kp3 # Kp
	k[15,30] = Kd3 # Kd

	k[16,31] = Kp1 # Kp
	k[16,32] = Kd1 # Kd
	k[17,33] = Kp2 # Kp
	k[17,34] = Kd2 # Kd
	k[18,35] = Kp3 # Kp
	k[18,36] = Kd3 # Kd
end



tvlqr_policy = EmbeddedLciMpc.TVLQRPolicy(K, x, u, timestep, H)

output = EmbeddedLciMpc.exec_policy(tvlqr_policy, zeros(37), 0.0)

println("Finish Loading tvlqr stand")
