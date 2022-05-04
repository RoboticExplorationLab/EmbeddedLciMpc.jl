using Pkg; Pkg.activate(".");
using PackageCompiler

precompile_file_path = joinpath(@__DIR__, "centroidalStand_test.jl")
PackageCompiler.create_sysimage(["EmbeddedLciMpc", "ContactImplicitMPC", "LinearAlgebra"]; 
                                sysimage_path="LciMPCSysImageDocker.so",
                                precompile_execution_file=precompile_file_path)