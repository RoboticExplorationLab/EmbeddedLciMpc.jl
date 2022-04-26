module EmbeddedLciMpc

using Rotations
using LinearAlgebra
using OSQP 
using StaticArrays
using SparseArrays
# using ContactImplicitMPC

include("utils.jl")
include("controller.jl")
include("qpController.jl")

end # module
