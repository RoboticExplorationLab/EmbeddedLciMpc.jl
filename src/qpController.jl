abstract type AbstractPolicy end 

const g = 9.81 
mutable struct PolicyQP 
    mass::Float64    # robot mass 
    inertia::Matrix  # robot inertia 
    Q::SMatrix       # State cost hessian 
    R::SMatrix       # Control cost hessian 
    A::MMatrix       # linearized dynamics 
    α::MVector      # desired acceleration 
    ω::MVector      # desired rotation velocity 
    osqp_model::OSQP.Model  # OSQP solver 
    Kpp::Float64 
    Kdp::Float64
    Kdω::Float64
end 

function initQP(mass::Float64, inertia::AbstractMatrix,
                Q::Diagonal, R::Diagonal,
                α::Vector, ω::Vector, 
                Kpp::Float64, Kdp::Float64, Kdω::Float64)

    Q = SMatrix{6, 6}(Q)
    R = SMatrix{12, 12}(R)
    α = MVector{3}(α)
    ω = MVector{3}(ω)
    A = MMatrix{6, 12}(zeros(6, 12))
    A[1:3, :] = kron(ones(1,4), I(3))
    osqp_model = OSQP.Model()
    OSQP.setup!(osqp_model; P=sparse(A' * Q * A), q=zeros(12), verbose=0)
    return PolicyQP(mass, inertia, Q, R, A,  α, ω, osqp_model, Kpp, Kdp, Kdω)
end 


"""
    controller(policy::PolicyQP, x::AbstractArray, t::Float64)

given a quadruped state x, compute ground reaction forces that keep the 
robot balanceds. Assume no desired acceleration or rotation
position : x[1:3]
quaternion : x[4:7]
FL_foot : x[8:10]
FR_foot : x[11:13]
RL_foot : x[14:16]
RR_foot : x[17:19]
"""
function controller(policy::PolicyQP, x::AbstractArray, t::Float64)
    m = policy.mass
    # g = 9.81
    g_vec = SVector{3, Float64}([0, 0, 9.81])
    # temp = @SVector zeros(3)
    b = SVector{6, Float64}([m*(policy.α  + g_vec )... , m * policy.ω...])
    
    policy.A[4:6, 1:3] = hat(x[8:10])
    policy.A[4:6, 4:6] = hat(x[11:13])
    policy.A[4:6, 7:9] = hat(x[14:16])
    policy.A[4:6, 10:12] = hat(x[17:19])
    A = SMatrix{6, 12}(policy.A)

    # calcualte qp matrix 
    hessian = A'*policy.Q*A + policy.R
    gradient = - A' * policy.Q * b 

    display(sparse(hessian))
    display(sparse(hessian).nzval)

    OSQP.update!(policy.osqp_model, Px=sparse(hessian).nzval, q=Vector(gradient))
    OSQP.setup!(policy.osqp_model; P=sparse(hessian), q=Vector(gradient), verbose=0)
    result = OSQP.solve!(policy.osqp_model)    
    return result.x 
end 
