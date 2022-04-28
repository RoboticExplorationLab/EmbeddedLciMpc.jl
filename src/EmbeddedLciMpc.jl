module EmbeddedLciMpc

using Rotations
using LinearAlgebra
# using OSQP 
using StaticArrays
using SparseArrays
# using ContactImplicitMPC

abstract type AbstractPolicy end 

include("utils.jl")
include("controller.jl")
include("pdController.jl")
# include("qpController.jl")

end # module
