mutable struct TVLQRPolicy{T}
    K::Vector{Matrix{T}}
    x::Vector{Vector{T}}
    u::Vector{Vector{T}}
    timestep::T
    H::Int
end

function exec_policy(p::TVLQRPolicy{T}, x::Vector{T}, t::T) where {T}
    timestep = p.timestep
    i = convert(Int64, floor(t / timestep) % H + 1)
    u = p.u[i] + p.K[i] * (p.x[i] - x)

	# q_ref_now = p.traj.q[2]
	# q_ref_next = p.traj.q[3]
	# v_ref = (q_ref_next - q_ref_now) / p.traj.h 

	# q_now = p.newton.traj.q[2]
	# q_next = p.newton.traj.q[3]
	# v_now = (q_next - q_now) / p.traj.h 

    return [
            u / timestep; # force
            q_now; 
            v_now; 
            q_ref_now; 
            v_ref;
    ]
end