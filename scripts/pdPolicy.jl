using Pkg; Pkg.activate(".")
using Revise
using LinearAlgebra
import EmbeddedLciMpc as EM
using BenchmarkTools

# (mass::Float64, inertia::Matrix,
#                 Q::Diagonal, R::Diagonal,
#                 α::Vector, ω::Vector, 
#                 Kpp::Float64, Kdp::Float64, Kdω::Float64)
mass = 12.454
inertia = Diagonal([0.08, 0.3, 0.39])
Q = Diagonal([100, 100, 300, 80, 80, 100])
R = Diagonal([1e-2, 1e-2, 1e-3, 
              1e-2, 1e-2, 1e-3,
              1e-2, 1e-2, 1e-3,
              1e-2, 1e-2, 1e-3])
α = zeros(3)
ω = zeros(3)
Kpp = 100.
Kdp = 100. 
Kdω = 100. 
qpPolicy = EM.initQP(mass, inertia, Q, R, α, ω, Kpp, Kdp, Kdω)

x = zeros(19); x[4] = 1.0 
x[8:10] = [0.1, 0.1, -0.2]
x[11:13] = [0.1, -0.1, -0.2]
x[14:16] = [-0.1, 0.1, -0.2]
x[17:19] = [-0.1, -0.1, -0.2]
@btime EM.controller($qpPolicy, $x, $0.0)