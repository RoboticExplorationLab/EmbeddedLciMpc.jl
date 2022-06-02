function update_velocity(p::EmbeddedLciMpc.LciMPC.CIMPC{T,NQ,NU,NW,NC}, v) where {T,NQ,NU,NW,NC}
    for i in 1:length(p.newton.obj.v_target)
        # linear velocity 
        p.newton.obj.v_target[i][1:3] .= v[1:3]
        p.newton.obj.v_target[i][7:9] .= v[1:3]
        p.newton.obj.v_target[i][10:12] .= v[1:3]
        p.newton.obj.v_target[i][13:15] .= v[1:3]
        p.newton.obj.v_target[i][16:18] .= v[1:3]

        # angular velocity 
        p.newton.obj.v_target[i][4:6] .= v[4:6]
    end 
end

println("Finish Loading Utils")