import math
import time

# Constants for Lennard-Jones potential
epsilon = 100  # Depth of the potential well
sigma = 0.25   # Distance at which potential is zero (2 * bot length)
cutoff_distance = 5 * sigma #cutoff distance, beyond this distance the potential is zero

# Initial position (in meters), base speed (in meters per time step), and orientation (in degrees)
position = [0.0, 0.0]  # Position in meters
base_speed = 0.05  # Base speed in meters per time step
orientation = 0.0  # Angle in degrees

# Speed limits (in meters per time step)
min_speed = 0.001
max_speed = 0.1

def calculate_force(position_a, position_b):
    """Calculate the force vector between two bots based on Lennard-Jones potential."""
    dx = position_b[0] - position_a[0]
    dy = position_b[1] - position_a[1]
    r = math.sqrt(dx ** 2 + dy ** 2)
    """No force returns after cutoff distance"""
    if r > cutoff_distance:
        return 0.0, 0.0
    
    if r != 0:
        force_magnitude = 24 * epsilon * (2 * (sigma ** 12 / r ** 13) - (sigma ** 6 / r ** 7))
        force_x = force_magnitude * (dx / r)
        force_y = force_magnitude * (dy / r)
    else:
        force_x, force_y = 0, 0
    
    print(f"Force from ({position_a}) to ({position_b}) -> Force: ({force_x}, {force_y})")  # Debugging force values
    return force_x, force_y

def update_position_and_orientation(position, orientation, forces):
    """Update the robot's position and orientation based on calculated forces."""
    total_force_x, total_force_y = 0, 0
    for fx, fy in forces:
        total_force_x += fx
        total_force_y += fy
    
    # Calculate the resultant force direction and magnitude
    resultant_angle = math.atan2(total_force_y, total_force_x) * 180 / math.pi
    resultant_magnitude = math.sqrt(total_force_x ** 2 + total_force_y ** 2)
    
    # Update orientation to align with the resultant force direction
    orientation = resultant_angle
    
    # Adjust speed based on the resultant force magnitude
    # Map the resultant_magnitude to a speed between min_speed and max_speed
    speed = min(max_speed, max(min_speed, base_speed * resultant_magnitude))
    
    print(f"Total Force: ({total_force_x}, {total_force_y}) -> Magnitude: {resultant_magnitude}, Speed: {speed}")  # Debugging speed adjustment
    
    # Update position based on adjusted speed and resultant force direction
    position[0] += speed * math.cos(math.radians(orientation))
    position[1] += speed * math.sin(math.radians(orientation))
    
    return position, orientation, speed

def main_loop():
    """Main loop for the robot."""
    global position, orientation
    while True:
        # Receive positions of other bots from the server (for simplicity, we use a placeholder)
        other_bot_positions = receive_positions_from_server()
        
        # Calculate forces from other bots
        forces = []
        for other_position in other_bot_positions:
            fx, fy = calculate_force(position, other_position)
            forces.append((fx, fy))
        
        # Update position and orientation based on forces
        position, orientation, speed = update_position_and_orientation(position, orientation, forces)
        
        # Send updated position to the server
        send_position_to_server(position, orientation, speed)
        
        # Pause for a short time (e.g., 100 ms) to simulate time steps
        time.sleep(0.5)

def receive_positions_from_server():
    """Placeholder function to receive positions from the server."""
    #   TODO
    return [(0.75, .75), (1.0, -1.0), (-0.50, 0.50)]

def send_position_to_server(position, orientation, speed):
    """Placeholder function to send the current position to the server might not be necessary in final code, used for simulation and debugging now"""
    # TODO
    print("Sending position to server:", position, "Orientation:", orientation, "Speed:", speed)

# Start the main loop
main_loop()
