from controller import Robot

# Create the Robot instance
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Initialize the motors
motor1 = robot.getDevice('motor1')
motor2 = robot.getDevice('motor2')

# Set target positions for the motors
motor1.setPosition(float('inf'))  # 'inf' for velocity control mode
motor2.setPosition(float('inf'))

# Set initial velocity
motor1.setVelocity(100.0)
motor2.setVelocity(0.0)

# Main loop:
# - Perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Set desired velocities
    motor1.setVelocity(10.0)  # Example velocity value
    motor2.setVelocity(10.0)  # Example velocity value
    
    # You can add additional logic here to change velocities or stop the motors

# Note: Exiting the loop is handled by Webots when the simulation stops

# If you have any cleanup code, you can put it here
