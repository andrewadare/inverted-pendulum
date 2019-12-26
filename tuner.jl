#!/usr/bin/env julia

println("Loading dependencies...")
using Plots
using LibSerialPort
using JSON

# Plot backend
gr(size=(900, 500))

function readmcu(mcu::SerialPort)
    datadict = Dict()
    msg = readstring(mcu)
    if startswith(msg, "{") && endswith(msg, "}")
        try
            datadict = JSON.parse(msg)
        catch
            println("skipping $msg")
        end
    elseif length(msg) > 0
        println(msg) # Pass through non-data messages
    end
    return datadict
end

# Manage arrays as FIFO queues
function bump_queues(datadict, times, setpoints, positions)

    for key in ["time", "setpoint", "x"]
        haskey(datadict, key) || return false
    end

    push!(times, datadict["time"]/1000) # ms to seconds
    push!(setpoints, datadict["setpoint"])
    push!(positions, datadict["x"])
    for v in (times, setpoints, positions)
        shift!(v)
    end
    return true
end

function rtplot(times, setpoints, positions)
    plot(times, [setpoints, positions],
         xaxis="time [s]",
         yaxis=("position [% max]", (-110, 110)), # title, limits
         labels=(["setpoint","position"]), # legend
         line=([1 2], [:steppre :path], ["gray" "crimson"], [1.0 0.5]), # lineweights, linetypes, colors, alphas
         fill=(0, [0.1 0], "dodgerblue")) # y-origin, alpha, colors

    # Update the plot window
    gui()
end

let
    counter = 0
    global process_mcu_stream

    function process_mcu_stream(mcu, times, setpoints, positions)
        datadict = readmcu(mcu)
        counter += 1
        if counter > 100
            bump_queues(datadict, times, setpoints, positions) || return
            rtplot(times, setpoints, positions)
        end
    end
end

function main()

    # Number of points to display
    const N = 500
    const address = length(ARGS) == 1 ? ARGS[1] : "/dev/cu.usbmodem2370891"
    times, setpoints, positions = collect(linspace(0,10,N)), zeros(N), zeros(N)

    println("Opening plot window...")
    rtplot(times, setpoints, positions)
    sleep(3)

    println("Connecting to MCU...")
    mcu = open(address, 115200)

    println("Starting I/O loop.")
    while true
        @async begin
            a = readline(STDIN)
            a == "q\n" && quit()
            write(mcu, "$a")
        end
        @async begin
            process_mcu_stream(mcu, times, setpoints, positions)
        end
        # Give the queued tasks a chance to run
        sleep(0.0001)
    end
end

main()