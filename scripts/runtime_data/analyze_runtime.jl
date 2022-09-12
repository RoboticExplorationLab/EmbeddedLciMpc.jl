using Statistics
using Distributions
using HypothesisTests
using PlotlyJS

function load_csv(file_path)
    # input file path 
    # output a Float64 array for solve times 
    
    f = open(file_path, "r")

    solve_time = Float64[]
    for line in readlines(f)
        push!(solve_time, parse(Float64, line))
    end 
    return solve_time
end

function analyze(solve_time)
    ci_lower, ci_upper = ci(OneSampleTTest(solve_time))
    max = maximum(solve_time)
    min = minimum(solve_time)
    average = mean(solve_time)
    err = ci_upper - average
    return err, max, min, average 
end

function plot_solve_times(h_mpc_list, n_sample_list, name)
    bar_list = GenericTrace{Dict{Symbol, Any}}[]
    for n_sample in n_sample_list
        average_list = []
        err_list = []
        max_list = []
        min_list = []
        x_list = []
        for h_mpc in h_mpc_list
            file_path = joinpath(@__DIR__, "stand/Nsample$(n_sample)_Hmpc$(h_mpc).csv")
            solve_time = load_csv(file_path);
            err, max_, min_, ave = analyze(solve_time)
            push!(average_list, ave)
            push!(err_list, err)
            push!(max_list, max_)
            push!(min_list, min_)
            push!(x_list, "MPC horizon=$(h_mpc) ")
        end
        # plot mean and CI for the data set 
        bar_ = PlotlyJS.bar(
                name = "N Sample = $(n_sample)",
                x = x_list, y = average_list,
                error_y = attr(type="data", array=err_list)
            )
        push!(bar_list, bar_)
        # TODO: max of solve time of the data set 
    end

    PlotlyJS.plot(bar_list)
    # TODO: save plots 
end

h_mpc_list = [2, 5, 10]
n_sample_list = [5, 10]

plot_solve_times(h_mpc_list, n_sample_list, "stand")



# plot(
# [
#     bar(
#         name = "solve time",
#         x = ["HMPC=5"], y = [ave],
#         error_y = attr(type="data", array=[err])
#     )
# ])


# [
#     bar(
#         name="Control",
#         x=["Trial 1", "Trial 2", "Trial 3"], y=[3, 6, 4],
#         error_y=attr(type="data", array=[1, 0.5, 1.5])
#     ),
#     bar(
#         name="Experimental",
#         x=["Trial 1", "Trial 2", "Trial 3"], y=[4, 7, 3],
#         error_y=attr(type="data", array=[0.5, 1, 2])
#     )
# ]