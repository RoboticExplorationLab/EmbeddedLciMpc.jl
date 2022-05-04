module EmbeddedLciMpc

using Rotations
using LinearAlgebra
# using OSQP 
using StaticArrays
using SparseArrays
import ContactImplicitMPC as LciMPC

abstract type AbstractPolicy end 

include("utils.jl")
include("controller.jl")
include("pdController.jl")
include("lciPolicy.jl")

end # module
