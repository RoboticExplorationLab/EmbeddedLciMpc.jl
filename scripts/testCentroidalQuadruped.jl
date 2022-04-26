using ContactImplicitMPC
using LinearAlgebra

s = get_simulation("centroidal_quadruped", "flat_3D_lc", "flat")
model = s.model
env = s.env