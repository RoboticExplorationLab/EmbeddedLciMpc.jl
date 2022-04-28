mutable struct PDPolicy <: AbstractPolicy 
    Kp::Float64
    Kd::Float64
    q_des::Matrix{Float64}
end 

function policy(policy::PDPolicy, x, t) 
    q = x[1:12]
    v = x[13:end]
    return 0.2 * policy.Kp * (policy.q_des - q) - 1.0 * policy.Kd * (v)
end