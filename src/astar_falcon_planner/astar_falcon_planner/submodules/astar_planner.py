import math
import pandas as pd

def plan_path(start, end, robot_radius, clearance, delta_time, goal_threshold, 
             wheel_radius, wheel_distance, rpm1, rpm2):
    """Returns path as List[List[dx, dy, dtheta]] in cm and radians"""

    file_path = 'waypoints.txt'

    # Pipeline to account for the falcon offsets

    # Read the data from the file
    df = pd.read_csv(file_path, header=None)

    # Calculate the differences of the second elements
    diffs = df[1].diff().dropna()

    # Initialize the list of lists
    absolute_path_cm_deg = []

    # Set the second element of the first list to 700
    second_element_previous = 700

    # Loop through each row to build the list
    for index, row in df.iterrows():
        # For the first element, calculate as 1630 + 100 * first element
        first_element = 1630 + 100 * row[0]
        
        # For the second element, use the previous value and subtract the difference
        second_element = second_element_previous if index == 0 else second_element_previous - 100 * (diffs.iloc[index-1])
        
        # Keep the third element the same
        third_element = row[2]
        
        # Append the list to the result
        absolute_path_cm_deg.append([first_element, second_element, third_element])
        
        # Update the previous second element for the next iteration
        second_element_previous = second_element

    # Print the result
    for item in absolute_path_cm_deg:
        print(item)

    # Convert to relative path to return increments
    path = []
    for i in range(1, len(absolute_path_cm_deg)):
        prev = absolute_path_cm_deg[i-1]
        curr = absolute_path_cm_deg[i]
        
        dx = curr[0] - prev[0]  # delta x (cm)
        dy = curr[1] - prev[1]  # delta y (cm)
        dtheta_deg = curr[2] - prev[2]  # delta theta (degrees)
        
        # Normalize angle difference to [-180, 180]
        dtheta_deg = (dtheta_deg + 180) % 360 - 180
        dtheta_rad = math.radians(dtheta_deg)  # convert to radians
        
        path.append([dx, dy, dtheta_rad])

    # path=[]
    
    return path