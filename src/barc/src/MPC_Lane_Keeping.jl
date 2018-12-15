#!/usr/bin/env julia
# Author: Tony Zheng
# This node uses MPC algorithms to calculate the optimal inputs that minimizes 
# the distance between the BARC and the reference trajectory. For this demonstration
# a dummy reference trajectory is fed into this node which then publishes the optimal
# state trajectory.

println("Initializing Packages...")
tic()
using RobotOS
using JuMP
using Ipopt

@rosimport std_msgs.msg: Float64MultiArray, Float64
@rosimport barc.msg: Input, barc_state
@rosimport geometry_msgs.msg: Vector3, Twist

rostypegen()
using barc.msg
using geometry_msgs.msg
using std_msgs.msg


println("DONE")

# -------------------------------------------------------------------
#                           MPC PARAMETERS
# -------------------------------------------------------------------

println("Defining Parameters...")

# Number of states / Number of inputs
nz = get_param("/numStates")
nu = get_param("/numInputs")


#### MAKE MPC TERMS PARAMETERS
# Horizons
n = get_param("/numStepsToLookAhead")           # MPC Horizon (CFTOC Horizon)

# Cost matrices
Q = get_param("/Q")
R = get_param("/R")

#Box constraints on states
x_min = get_param("/x_min")
x_max = get_param("/x_max")
y_min = get_param("/y_min")
y_max = get_param("/y_max")

# Box constraints on input, A*u <= B
v_max = get_param("/v_max")
v_min = get_param("/v_min")
steer_min = get_param("/steer_min")
steer_max = get_param("/steer_max")

vref = get_param("/reference_velocity")
rate = get_param("/loop_rate")

numMovingAveragePoints = get_param("/numMovingAveragePoints")
velSum = []
deltaSum = []
count = 0

println("DONE")

# -------------------------------------------------------------------
#                         SET UP CFTOC MODEL
# -------------------------------------------------------------------

println("Creating CFTOC Model...")

m = Model(solver = IpoptSolver(print_level=0))

# Variables
@variable(m,z[1:nz,1:n+1])              # State vectors (nz x n+1)
@variable(m,u[1:nu,1:n])                # Input vectors (nu x n)

# Reference Trajectory
@NLparameter(m, xref[i=1:n] == 0)
@NLparameter(m, yref[i=1:n] == 0)
@NLparameter(m, pref[i=1:n] == 0)

# Objective Function

@NLobjective(m, :Min, sum((z[1,i+1]-z[1,i])^2+(z[2,i+1]-z[2,i])^2 for i in 1:n))


# State Constraints

for i in 1:n+1
    @constraint(m, x_min <= z[1,i] <= x_max)
    @constraint(m, y_min <= z[2,i] <= y_max)
    @constraint(m, -3.1415926/2 <= z[3,i] <= 3.1415926/2)
end

for i in 1:n
    @constraint(m, v_min <= u[1,i] <= v_max)
    @constraint(m, steer_min <= u[2,i] <= steer_max)
end

# Initial Condition and Constraints
@NLparameter(m, x0 == 0); @NLconstraint(m, z[1,1] == x0);
@NLparameter(m, y0 == 0); @NLconstraint(m, z[2,1] == y0);
@NLparameter(m, p0 == 0); @NLconstraint(m, z[3,1] == p0);
@NLparameter(m, dt == 0.05);
@NLparameter(m, xf == 0); @NLconstraint(m, z[1,n+1] == xf);
@NLparameter(m, yf == 0); @NLconstraint(m, z[2,n+1] == yf);

# Dynamics Constraints
for i in 1:n-1                            
    @NLconstraint(m, z[1,i+1] == z[1,i] + dt*u[1,i]*cos(z[3,i]))
    @NLconstraint(m, z[2,i+1] == z[2,i] + dt*u[1,i]*sin(z[3,i]))
    @NLconstraint(m, z[3,i+1] == z[3,i] + dt*u[2,i])
end



println("DONE")

# -------------------------------------------------------------------
#                             INITIAL SOLVE
# -------------------------------------------------------------------

println("Initial solve...")
solve(m)
println("DONE")

# -------------------------------------------------------------------
#                             ROS FUNCTIONS
# -------------------------------------------------------------------

println("Running MPC...")

function dt_callback(msg::Float64)
    setvalue(dt, msg.data)
end

function callback2(msg::Vector3)
    # Assign new reference trajectory
    setvalue(xf, msg.x)
    setvalue(yf, msg.y)
end

function loop(pub_one, pub_two)
    loop_rate = Rate(rate)
    count = 0
    velSum = []
    deltaSum = []
    while ! is_shutdown()
	count = count+1
        # -----------------------------------------------------------
        #                      RUNNING MPC
        # -----------------------------------------------------------
       
        solve(m)                                # Solve CFTOC
        zOpt = getvalue(z)
        uOpt = getvalue(u)
        JOpt = getobjectivevalue(m)


	if count<numMovingAveragePoints
		push!(velSum,uOpt[1,1])
		push!(deltaSum,uOpt[2,1])
	else
        deleteat!(velSum,1)
        deleteat!(deltaSum,1)
        push!(velSum,uOpt[1,1])
        push!(deltaSum,uOpt[2,1])
	end
    # Publish optimal inputs and optimal state trajectory
    # Only need to publish state traj for this demonstration

    cmdInt = Twist(Vector3(mean(velSum),0,0),Vector3(0,0,mean(deltaSum)))    # Command to publish
    optTraj = barc_state(zOpt[1,1:end],zOpt[2,1:end],zOpt[3,1:end])
    publish(pub_one,cmdInt)                 
    publish(pub_two,optTraj)
    rossleep(loop_rate)

    end
end

function main()
    init_node("MPC_node")

    pub = Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=10)
    pub2 = Publisher("optimal_state_trajectory", barc_state, queue_size=10)
    sub1 = Subscriber("dt", Float64Msg, dt_callback, queue_size=10)
    sub2 = Subscriber("terminal_state", Vector3, callback2, queue_size=10)
    loop(pub, pub2)
end

if ! isinteractive()
    main()
end
