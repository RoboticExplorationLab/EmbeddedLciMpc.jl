using Statistics
using Distributions
using HypothesisTests
using PlotlyJS
using DataFrames
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

function plot_solve_times(h_mpc_list, n_sample_list, gait)
    bar_list = GenericTrace{Dict{Symbol, Any}}[]
    for n_sample in n_sample_list
        average_list = []
        err_list = []
        max_list = []
        min_list = []
        x_list = []
        for h_mpc in h_mpc_list
            file_path = joinpath(@__DIR__, "$(gait)/Nsample$(n_sample)_Hmpc$(h_mpc).csv")
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
    layout = Layout(
                xaxis_title="",
                yaxis_title="solve time (ms)",
                yaxis_range=[0, 10],
                title=attr(
                        text= "$(gait) solve times",
                        y=0.95,
                        x=0.5,
                        xanchor= "center",
                        yanchor= "top"
                    )
                # legend_title_text="Legend"
            )
    plot = PlotlyJS.plot(bar_list, layout)
    # TODO: save plots 
    savefig(plot, joinpath(@__DIR__, "$(gait)/solve_times.png"))
end

function boxplot_solve_times(h_mpc_list, gait)
    trace_list = GenericTrace{Dict{Symbol, Any}}[]
    bar_list = GenericTrace{Dict{Symbol, Any}}[]
    n_sample = 5
    for h_mpc in h_mpc_list
        file_path = joinpath(@__DIR__, "$(gait)/Hmpc$(h_mpc).csv")
        solve_time = load_csv(file_path);
        trace = box(y=solve_time, boxpoints="all", name="MPC horizon = $(h_mpc)")
        push!(trace_list, trace)
        bar_ = PlotlyJS.bar(
                # name = "N Sample = $(n_sample)",
                x = ["MPC horizon = $(h_mpc)"], y = [maximum(solve_time)],
                # error_y = attr(type="data", array=err_list)
            )
        push!(bar_list, bar_)
    end 
    layout_box = Layout(
                xaxis_title="",
                yaxis_title="solve time (ms)",
                yaxis_range=[0, 40],
                title=attr(
                        text= "$(gait) solve times",
                        y=0.95,
                        x=0.5,
                        xanchor= "center",
                        yanchor= "top"
                    )
                # legend_title_text="Legend"
            )
    layout_bar = Layout(
        xaxis_title="",
        yaxis_title="solve time (ms)",
        yaxis_range=[0, 200],
        title=attr(
                text= "$(gait) max solve times",
                y=0.95,
                x=0.5,
                xanchor= "center",
                yanchor= "top"
            )
        # legend_title_text="Legend"
    )
    box_plt = plot(trace_list, layout_box)
    bar_plt = plot(bar_list, layout_bar)
    savefig(box_plt, joinpath(@__DIR__, "$(gait)/solve_times.png"))
    savefig(bar_plt, joinpath(@__DIR__, "$(gait)/max_solve_times.png"))
end

h_mpc_list = [2, 5, 10]
n_sample_list = [5, 10]
boxplot_solve_times(h_mpc_list, "stand")
boxplot_solve_times(h_mpc_list, "trot")
# plot_solve_times(h_mpc_list, n_sample_list, "stand")
# plot_solve_times(h_mpc_list, n_sample_list, "trot")

file_path = joinpath(@__DIR__, "reach/test.csv")
solve_time = load_csv(file_path);
mean(solve_time)

maximum(solve_time)