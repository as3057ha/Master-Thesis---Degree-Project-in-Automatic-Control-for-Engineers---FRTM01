import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import simps
import os
import datetime

SAMPLE_TIME = 0.1

def generate_linear_acceleration(start_speed, end_speed, acceleration):
    t1 = (end_speed - start_speed) / acceleration

    # Time array
    time = np.linspace(0, t1, int(t1 / SAMPLE_TIME) + 1)

    velocity = start_speed + acceleration * time

    # Ensure velocity doesn't exceed end_speed
    velocity[velocity > end_speed] = end_speed
    
    return velocity

def generate_linear_deceleration(start_speed, end_speed, deceleration):
    t1 = (start_speed - end_speed) / deceleration

    # Time array
    time = np.linspace(0, t1, int(t1 / SAMPLE_TIME) + 1)

    velocity = start_speed - deceleration * time

    # Ensure velocity doesn't become negative
    velocity[velocity < 0] = 0

    return velocity


def generate_constant_speed(speed, time_at_speed):
    t1 = time_at_speed

    # Time array
    time = np.linspace(0, t1, int(t1 / SAMPLE_TIME))

    velocity = np.zeros(time.shape[0])

    for i in range(time.shape[0]):
        velocity[i] = speed
    
    return velocity

def generate_quadratic_acceleration(start_speed, top_speed, acc_time):
    acceleration = (top_speed - start_speed) / acc_time
    t1 = min(acc_time, top_speed / acceleration)  # Ensure we reach top_speed in acc_time

    # Time array
    time = np.linspace(0, t1, int(t1 / SAMPLE_TIME) + 1)

    velocity = start_speed + 0.5 * acceleration * time**2

    return velocity

def generate_quadratic_deceleration(start_speed, top_speed, dec_time):
    deceleration = start_speed / dec_time
    t1 = min(dec_time, start_speed / deceleration)  # Ensure we reach zero speed in dec_time

    # Time array
    time = np.linspace(0, t1, int(t1 / SAMPLE_TIME) + 1)

    velocity = start_speed - 0.5 * deceleration * time**2
    velocity[velocity < 0] = 0  # Ensure velocity doesn't become negative

    return velocity

def concatenate_velocity_vectors(velocity_vectors):

    # Concatinate array
    concatinated_velocity = np.concatenate(velocity_vectors)
    # Determine total length of concatenated velocity vectors
    total_length = np.shape(concatinated_velocity)[0]
    print("Total length" +  str(total_length))
    
    time = np.linspace(0, int(total_length*SAMPLE_TIME), total_length )

    return time, concatinated_velocity

# Function to add noise to velocity data
def add_noise(velocity, noise_level=0.005):
    """
    Adds Gaussian noise to the velocity data.

    Parameters:
        velocity (array_like): Array containing velocity data.
        noise_level (float): Standard deviation of the Gaussian noise.

    Returns:
        array_like: Array containing velocity data with added noise.
    """
    noise = np.random.normal(loc=0, scale=noise_level, size=len(velocity))
    velocity_with_noise = velocity + noise
    return velocity_with_noise

def main():
    addNoise = True
    # Generate velocity data
    time, velocity = concatenate_velocity_vectors([
        generate_quadratic_acceleration(0,0.5,2),
        generate_constant_speed(0.5, 1),
        generate_quadratic_deceleration(0.5,0,3)
    ])
    
    # Integrate velocity to get distance using Simpson's rule
    distance = simps(velocity, time)
    distance_meters = distance 

    noise_str = "withNoise" if addNoise else "withoutNoise"

    if addNoise:
        velocity = add_noise(velocity)
    
    # Generate a unique filename based on today's date and distance
    today_date = datetime.datetime.now().strftime("%Y-%m-%d")
    filename = f"velocity_data_{distance_meters:.2f}m_{noise_str}_{today_date}.txt"
    graph_filename = f"velocity_graph_{distance_meters:.2f}m_{noise_str}_{today_date}.png"

    # Convert velocity array to string
    velocity_str = ','.join(map(str, velocity))

    # Save velocity data to a text file
    with open(filename, 'w') as file:
        file.write(velocity_str)

    # Plot and save the graph
    plt.figure(figsize=(10, 6))
    plt.plot(time, velocity, label='Velocity')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.title('Velocity vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(graph_filename)
    plt.show()

    print(f"Velocity data saved to {filename}")
    print(f"Graph saved as {graph_filename}")

if __name__ == "__main__":
    main()