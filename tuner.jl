#!/usr/bin/env julia

println("Loading dependencies...")

using Plots
using LibSerialPort
using JSON

# Plot backend
gr(size=(900, 500))

address = length(ARGS) == 1 ? ARGS[1] : "/dev/cu.usbmodem2370891"

# Number of points to display
n = 500


# Values for horizontal (time) axis and data
t, y1, y2 = zeros(n), zeros(n), zeros(n)

# Use update_counter like a static variable with function scope in C
let update_counter = 0
    global readmcu
function readmcu(sp::SerialPort)
    msg = chomp(readuntil(mcu, '\n', 100))

    data = Dict()
    if startswith(msg, "{") && endswith(msg, "}")
        try
            data = JSON.parse(msg)
        catch
            println("skipping $msg")
            return
        end
        # println(data)
    end

    for key in ["time", "setpoint", "x"]
        if !haskey(data, key)
            # println("missing $key")
            return
        end
    end

    tms = data["time"]
    setpoint =data["setpoint"]
    x = data["x"]
    println("$tms, $setpoint, $x")

    # Manage arrays as FIFO queues
    push!(t, data["time"]/1000) # ms to seconds
    push!(y1, data["setpoint"])
    push!(y2, data["x"])
    if length(t) > n
        for v in (t, y1, y2)
            shift!(v)
        end
    end

    # if mod(tms, 100) == 0
    if update_counter > 10
        rtplot(t, y1, y2)
        update_counter = 0
    end

    update_counter += 1
end
end

function rtplot(t, y1, y2)
    plot(t, [y1, y2],
         title="Data stream", # Not displaying with GR backend (?)
         xaxis="time [s]",
         yaxis=("position [% max]", (-110, 110)), # title, limits
         labels=(["setpoint","position"]), # legend
         line=([1 2], [:steppre :path], ["gray" "crimson"], [1.0 0.5]), # lineweights, linetypes, colors, alphas
         fill=(0, [0.1 0], "dodgerblue"), # y-origin, alpha, colors
         )
    # Update the plot window
    gui()
end

println("Creating plot...")
rtplot(t,y1,y2)

println("Connecting to MCU...")
mcu = open(address, 115200)
println("Ready")

while true

    @async begin
        a = readline(STDIN)
        gui()
        a == "q\n" && quit()
        write(mcu, "$a")
    end

    @async begin
        readmcu(mcu)
        # @sync begin
            # @async readmcu(mcu)
            # @async rtplot(t,y1,y2)
        # end
    end

    # Give the queued tasks a chance to run
    sleep(0.0001)
end
