# function exec_policy(p::LciMPC.CIMPC{T,NQ,NU,NW,NC}, x::Vector{T}, t::T) where {T,NQ,NU,NW,NC}
# 	# reset
# 	if t ≈ 0.0
# 		p.next_time_update = 0.0
# 		p.q0 .= p.ref_traj.q[1]
# 		p.altitude .= 0.0
# 		p.buffer_time = 0.0
# 		LciMPC.set_trajectory!(p.traj, p.ref_traj)
# 		LciMPC.set_implicit_trajectory!(p.im_traj, p.im_traj_cache)
# 		LciMPC.reset_window!(p.window)
# 	end

# 	K1 = p.K_traj[p.window[1]]

#     if t >= p.next_time_update
# 		(p.opts.altitude_update && t > 0.0) && (LciMPC.update_altitude!(p.altitude, p.ϕ, p.s,
# 									x, NQ, NC, p.N_sample,
# 									threshold = p.opts.altitude_impact_threshold,
# 									verbose = p.opts.altitude_verbose))

# 		LciMPC.set_altitude!(p.im_traj, p.altitude)
# 		LciMPC.update!(p.im_traj, p.traj, p.s, p.altitude, p.κ[1], p.traj.H)

# 		# visualize
# 		# p.opts.live_plotting && live_plotting(p.s.model, p.traj, traj, p.newton, p.q0, traj.q[t+1], t)

# 		# shift trajectory
# 		LciMPC.rot_n_stride!(p.traj, p.traj_cache, p.stride, p.window)

# 		# update
# 		LciMPC.update_window!(p.window, p.ref_traj.H)

# 		# reset update time
# 		p.next_time_update = (t - t % p.traj.h) + p.traj.h
#     end

# 	policy_time = @elapsed begin
# 		if p.buffer_time <= 0.0
# 			# @show t
# 			# optimize
# 			q1 = x[1:NQ]
# 			q0 = x[1:NQ] - x[NQ .+ (1:NQ)] .* p.traj.h
# 			LciMPC.newton_solve!(p.newton, p.s, q0, q1,
# 				p.window, p.im_traj, p.traj, warm_start = t > 0.0)

# 			LciMPC.update!(p.im_traj, p.traj, p.s, p.altitude, p.κ[1], p.traj.H)
# 		end
# 	end

# 	# update buffer time
# 	(p.buffer_time < 0.0) && (p.buffer_time = policy_time)
# 	p.buffer_time -= p.traj.h

# 	# scale control
# 	p.u .= p.newton.traj.u[1]

# 	# add gains
# 	q1 = x[1:NQ]
# 	q0 = x[1:NQ] - x[NQ .+ (1:NQ)] .* p.traj.h
# 	p.u .+= K1 * ([p.traj.q[1]; p.traj.q[2]] - [q0; q1])
# 	# @show norm(p.newton.traj.u[1])
# 	# @show norm(K1 * ([p.traj.q[1]; p.traj.q[2]] - [q0; q1]))
# 	p.u ./= p.traj.h 

# 	# extract the current reference trajectory from the controller 
# 	q_ref_now = p.traj.q[2]
# 	q_ref_next = p.traj.q[3]
# 	v_ref = (q_ref_next - q_ref_now) / p.traj.h 

# 	# extract the solved trajectory from the controller 
# 	q_now = p.newton.traj.q[2]
# 	q_next = p.newton.traj.q[3]
# 	v_now = (q_next - q_now) / p.traj.h 
# 	return [p.u; q_now; v_now; q_ref_now; v_ref]
# end

function exec_policy(p::LciMPC.CIMPC{T,NQ,NU,NW,NC}, x::Vector{T}, t::T) where {T,NQ,NU,NW,NC}
	# reset
	if t ≈ 0.0
		# p.next_time_update = 0.0
		p.q0 .= p.ref_traj.q[1]
		# p.altitude .= 0.0
		p.buffer_time = 0.0
		# set_trajectory!(p.traj, p.ref_traj)
		LciMPC.set_implicit_trajectory!(p.im_traj, p.im_traj_cache)
		LciMPC.reset_window!(p.window)
	end

	policy_time = @elapsed begin
		if p.buffer_time <= 0.0

			# window 
			idx_nearest = findnearest(p.times_reference, t % (p.ref_traj.h * (p.ref_traj.H - 1)))[1]
			for i = 1:(p.H + 2)
				p.window[i] = i + idx_nearest - 1 > p.ref_traj.H ? i - p.ref_traj.H + idx_nearest - 1 : i + idx_nearest - 1
				# print(p.window[i])
			end
			# println()
			LciMPC.set_window!(p.traj, p.ref_traj, p.window)

			# optimize
			q1 = x[1:NQ]
			q0 = x[1:NQ] - x[NQ .+ (1:NQ)] .* p.traj.h
			LciMPC.newton_solve!(p.newton, p.s, q0, q1,
				p.window, p.im_traj, p.traj, warm_start = t > 0.0)
		end
	end

	# update buffer time
	(p.buffer_time <= 0.0) && (p.buffer_time = policy_time)
	p.buffer_time -= p.traj.h
	p.buffer_time = -0.01

	# control
	p.u .= p.newton.traj.u[1]

	# add gains
	if p.opts.gains
		K1 = p.K_traj[1]

		q1 = x[1:NQ]
		q0 = x[1:NQ] - x[NQ .+ (1:NQ)] .* p.traj.h
		p.u .+= K1 * ([p.traj.q[1]; p.traj.q[2]] - [q0; q1])
	end
	
	p.u ./= p.traj.h

	# extract the current reference trajectory from the controller 
	q_ref_now = p.traj.q[2]
	q_ref_next = p.traj.q[3]
	v_ref = (q_ref_next - q_ref_now) / p.traj.h 

	# extract the solved trajectory from the controller 
	q_now = p.newton.traj.q[3]
	q_next = p.newton.traj.q[4]
	v_now = (q_next - q_now) / p.traj.h 
	return [p.u; q_now; v_now; q_ref_now; v_ref]
end

function findnearest(a,x)
	length(a) > 0 || return 0:-1
	r = searchsorted(a,x)
	length(r) > 0 && return r
	last(r) < 1 && return searchsorted(a,a[first(r)])
	first(r) > length(a) && return searchsorted(a,a[last(r)])
	x-a[last(r)] < a[first(r)]-x && return searchsorted(a,a[last(r)])
	x-a[last(r)] > a[first(r)]-x && return searchsorted(a,a[first(r)])
	return first(searchsorted(a,a[last(r)])):last(searchsorted(a,a[first(r)]))
end