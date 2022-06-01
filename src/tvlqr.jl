mutable struct TVLQRPolicy{T}
    K::Vector{Matrix{T}}
    x::Vector{Vector{T}}
    u::Vector{Vector{T}}
    timestep::T
    H::Int
end

function exec_policy(p::TVLQRPolicy{T}, x::Vector{T}, t::T) where {T}

    pos = x[1:3]
    qua = x[4:7]
    posFL = x[8:10]
    posFR = x[11:13]
    posRL = x[14:16]
    posRR = x[17:19]

    vel = x[19 .+ (1:3)]
    avel = x[19 .+ (4:6)]
    velFL = x[19 .+ (7:9)]
    velFR = x[19 .+ (10:12)]
    velRL = x[19 .+ (13:15)]
    velRR = x[19 .+ (16:18)]

    mrp = qua[2:4] ./ (qua[1] + 1.0)

    xb = [
        pos; mrp; vel; avel; 

        posRL[1]; velRL[1];
        posRL[2]; velRL[2];
        posRL[3]; velRL[3];

        posRR[1]; velRR[1];
        posRR[2]; velRR[2];
        posRR[3]; velRR[3];

        posFL[1]; velFL[1];
        posFL[2]; velFL[2];
        posFL[3]; velFL[3];
                
        posFR[1]; velFR[1];
        posFR[2]; velFR[2];
        posFR[3]; velFR[3];
    ]

    timestep = p.timestep
    i = convert(Int64, floor(t / timestep) % p.H + 1)
    u = p.u[i] + p.K[i] * (p.x[i] - xb)

	q_ref_now = zeros(19)
	# q_ref_next = p.traj.q[3]
	v_ref = zeros(18)#(q_ref_next - q_ref_now) / p.traj.h 

	q_now = zeros(19) #p.newton.traj.q[2]
	# q_next = #p.newton.traj.q[3]
	v_now = zeros(18)#(q_next - q_now) / p.traj.h 

    return [
            u / timestep; # force
            q_now; 
            v_now; 
            q_ref_now; 
            v_ref;
    ]
end