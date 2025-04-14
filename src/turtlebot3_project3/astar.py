import heapq
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

# Define the shapes using semi-algebraic equations
def vertical_lines(x, y):
    vertical_1 = (1000 <= x) & (x <= 1100) & (10 <= y) & (y <= 2010)
    vertical_2 = (2100 <= x) & (x <= 2200) & (990 <= y) & (y <= 2990)
    vertical_3 = (3200 <= x) & (x <= 3300) & (1990 <= y) & (y <= 2990)
    vertical_4 = (3200 <= x) & (x <= 3300) & (10 <= y) & (y <= 1010)
    vertical_5 = (4300 <= x) & (x <= 4400) & (10 <= y) & (y <= 2010)
    vertical_border_right = (0 <= x) & (x <= 10) & (0 <= y) & (y <= 3000)
    vertical_border_left = (5390 <= x) & (x <= 5400) & (0 <= y) & (y <= 3000)
    horizontal_border_top = (0 <= x) & (x <= 5400) & (2990 <= y) & (y <= 3000)
    horizontal_border_bottom = (0 <= x) & (x <= 5400) & (0 <= y) & (y <= 10)
    return vertical_1 | vertical_2 | vertical_3 | vertical_4 | vertical_5 | vertical_border_right | vertical_border_left  | horizontal_border_bottom | horizontal_border_top

def all_shapes(x, y):   # combinign all shapes
    return vertical_lines(x, y)

def generate_map(clearance = 25, map_height = 3000, map_width = 5400):      # defining map size in mm
   
    mm_to_pixels = 0.5      # defining scale factor of mm to pixels 
    width_pixels = int(map_width * mm_to_pixels)
    height_pixels = int(map_height * mm_to_pixels)  # converting mm to pixels

    x_mm = np.arange(width_pixels) / mm_to_pixels
    y_mm = (height_pixels - np.arange(height_pixels)) / mm_to_pixels
    X_mm, Y_mm = np.meshgrid(x_mm, y_mm)

    # Create obstacle mask
    obstacle_mask = all_shapes(X_mm, Y_mm)  # geting obstale map

    
    edge_clearance = int(1)    # Create edge clearance mask
    edge_mask = (
        (X_mm <= edge_clearance) | 
        (X_mm >= map_width - edge_clearance) | 
        (Y_mm <= edge_clearance) | 
        (Y_mm >= map_height - edge_clearance)
    )


    # create image with 3 channels - color image
    image = np.ones((height_pixels, width_pixels, 3), dtype=np.uint8) * 255
    image[obstacle_mask] = [0, 0, 0]  # black for obstacles
    image[edge_mask] = [0, 0, 255]  #red for edge clearance

    # Create clearance area around obstacles
    clearance_pixels = int(clearance/mm_to_pixels)  
    kernel = np.ones((3*clearance_pixels+1, 2*clearance_pixels+1), np.uint8)

    dilated = cv2.dilate((obstacle_mask*255).astype(np.uint8), kernel, iterations=1)    # dialating 
    obstacle_clearance_area = (dilated > 0) & ~obstacle_mask
    image[obstacle_clearance_area] = [0, 0, 255]  # red for obstacle clearance
    # Combining all obstacle areas for algorithm
    total_obstacle_area = obstacle_mask | obstacle_clearance_area | edge_mask

    # converting it into binary
    planning_image = np.ones((height_pixels, width_pixels, 3), dtype=np.uint8) * 255
    planning_image[total_obstacle_area] = 0


    # plt.imshow(image)
    # plt.show()

    return image

def generate_action(start_x, start_y, Thetai, action, map_rgb, grid, scale_factor=500, time_step=1):
    """
    Simulate the motion from the current state using given wheel speeds (action)
    and draw the trajectory on the provided image. Given by professor in class Howtoplotcurve.py/
    
    Parameters:
      start_x, start_y:   current coordinates.
      Thetai:             current orientation (degrees).
      action:             list/tuple [UL, UR] representing wheel speeds.
      map_rgb:            the image on which to draw the line (in RGB).
      scale_factor:       multiplier for step length.
      time_step:          how many iterations the simulation runs.
      
    Returns:
      (Xn, Yn, Thetan):   final coordinates and orientation (in degrees).
    """
    
    t = 0
    r = 0.033  # parameter (wheel radius or similar)
    L = 0.354  # parameter (distance between wheels)
    dt = 0.1     # time increment
    Xn = start_x
    Yn = start_y
    UL, UR = action[0], action[1]
    Thetan = 3.14 * Thetai / 180  # convert initial angle to radians
    # cv2.circle(map_rgb, (start_x,np.shape(map_rgb)[0] - start_y), 10, (255,0,255), -1)
    while t < time_step:
        t += dt
        # store previous position
        Xs = Xn
        Ys = Yn
        # update positions using the motion model
        Xn += (0.5 * r * (UL + UR) * math.cos(Thetan) * dt) * scale_factor
        Yn += (0.5 * r * (UL + UR) * math.sin(Thetan) * dt) * scale_factor
        # update orientation
        Thetan += (r / L) * (UR - UL) * dt
        # print(np.shape(map_rgb)[0] - int(Yn))
        if (np.shape(map_rgb)[0] - int(Yn)) >= np.shape(map_rgb)[0] or int(Xn) >= np.shape(map_rgb)[1] or grid[np.shape(map_rgb)[0] - int(Yn)][int(Xn)] < 255 :
            return Xn, Yn, Thetan, False
        # Draw the segment: note that we flip the y-coordinate so that (0,0) is bottom-left.
        pt1 = (int(Xs), np.shape(map_rgb)[0] - int(Ys))
        pt2 = (int(Xn), np.shape(map_rgb)[0] - int(Yn))
        cv2.line(map_rgb, pt1, pt2, (255, 0, 0), thickness=2)  # blue line (in BGR)


    # Convert final angle to degrees
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, True

def heuristic(a, b):
    """Calculate Euclidean distance between points a and b."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)




def a_star_search(grid, map_rgb, start, goal, out, rpm1,rpm2):
    XY_THRESHOLD = 50  # mm     # duplicate removal method as given in project 3 phase 1
    THETA_THRESHOLD = 30  # degrees

    X_DIM = int(np.shape(grid)[1] / XY_THRESHOLD)    # 500
    Y_DIM = int(np.shape(grid)[0] / XY_THRESHOLD)  
    THETA_DIM = int(360 / THETA_THRESHOLD)  # 12

    V = np.zeros((X_DIM, Y_DIM, THETA_DIM), dtype=np.uint8)
    print("Generating Path....")

    def get_region_indices(x, y, theta):        # getting regions for 3x3 matrix 
        x_idx = min(int(x / XY_THRESHOLD), X_DIM - 1)
        y_idx = min(int(y / XY_THRESHOLD), Y_DIM - 1)
        theta_idx = min(int(theta % 360 / THETA_THRESHOLD), THETA_DIM - 1)
        return x_idx, y_idx, theta_idx

    def is_visited(x, y, theta):
        x_idx, y_idx, theta_idx = get_region_indices(x, y, theta)       # functionn to check if node is already visted
        return V[x_idx, y_idx, theta_idx] == 1

    def mark_visited(x, y, theta):
        x_idx, y_idx, theta_idx = get_region_indices(x, y, theta)       # if node is already visted then it is marked as 1 in the V matrix
        V[x_idx, y_idx, theta_idx] = 1

    rows, cols = np.shape(grid)[0], np.shape(grid)[1]


    # plt.imshow(grid)
    # plt.show()
    cv2.circle(map_rgb, (goal[0],np.shape(map_rgb)[0] - goal[1]), 10, (0,0,255), -1)        # plotting start and goal point
    cv2.circle(map_rgb, (start[0],np.shape(map_rgb)[0] - start[1]), 10, (0,255,0), -1)
    

    open_set = []   
    heapq.heappush(open_set, (0, start))    # defing a heap queue
    came_from = {}      # dict to maintin parent nodes 
    cost_to_come = {start: 0}   # dict to maintain costs


    while open_set:
        current = heapq.heappop(open_set)[1]

        # Skip if already visited (done at node expansion )
        if is_visited(current[0], current[1], current[2]):
           
            continue

        # Mark as visited
        mark_visited(current[0], current[1], current[2])

        if heuristic(current, goal) < 50:   # checking if near goal point
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]    # applying backtracking to get the optimal path
            path.append(start)
            path.reverse()

            for i in range(1, len(path)):
                pt1 = (path[i - 1][0], np.shape(map_rgb)[0] - path[i - 1][1])
                pt2 = (path[i][0], np.shape(map_rgb)[0] - path[i][1])
                cv2.line(map_rgb, pt1, pt2, (0, 255, 0), thickness=10)      # plotting final path
                frame = cv2.resize(map_rgb, (640, 480))
                out.write(frame)
                cv2.waitKey(5)

            return path

        action_set = [[rpm1, rpm2], [rpm1, rpm1], [rpm2, rpm2], [rpm1, 0], [0, rpm1], [rpm2, rpm1]]     # action set

        for action in action_set:
            new_x, new_y, new_theta, ret = generate_action(current[0], current[1], current[2], action, map_rgb, grid)       # generating action from the howtoplotcurve.py code to genereate the search space

            if not ret:
                continue

            new_x = int(round(new_x))
            new_y = int(round(new_y))
            new_theta = round(new_theta % 360, 1)       # getting new nodes
            neighbor = (new_x, new_y, new_theta, tuple(action))

            if 0 <= neighbor[1] < rows and 0 <= neighbor[0] < cols:     # checking if nodes in bound
                if grid[neighbor[1]][neighbor[0]] < 255:
                    continue  # obstacle

                tentative_g = cost_to_come[current] + heuristic(current, neighbor)  # getting total tentative cost

                if neighbor not in cost_to_come or tentative_g < cost_to_come[neighbor]:
                    cost_to_come[neighbor] = tentative_g
                    cost_to_go = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (cost_to_go, neighbor))    # pushing in openlist
                    came_from[neighbor] = current   # updating

            frame = cv2.resize(map_rgb, (640, 480))
            out.write(frame)


    out.release()
    return None  # No path found

if __name__ == "__main__":      # run main file

    while True:
        try:
            clearance = int(input("Enter Robot Clearance (in millimeters 10 - 40) : "))      # getting clearnce input
            break
        except ValueError:
            print("Invalid input. Please enter integer value!")

    map_img = generate_map(clearance)   # generating map

    map_gray = cv2.cvtColor(map_img, cv2.COLOR_BGR2GRAY)
    

    # plt.imshow(map_gray, cmap='gray')
    # plt.title("Map (Grayscale)")
    # plt.show()

    print("Map dimensions:", np.shape(map_gray))
    


    rows, cols = map_gray.shape
    while True:
        try:
            sx = int(input("Enter start x (in millimeters): "))
            sy = int(input("Enter start y (in millimeters): "))
            sx = int(sx*0.5)
            sy = int(sy*0.5)
            print(sx,sy)
            stheta = int(input("Enter start theta (in degrees): ")) # getting input from user for start node
            
            if 0 <= sy < rows and 0 <= sx < cols:
                if map_gray[sy][sx] == 255:
                    start_state = (sx, sy, stheta)
                    break
                else:
                    print("The start lies in an obstacle. Please enter a valid point.")
            else:
                print("Coordinates out of bounds. Please enter values within map dimensions.")
        except ValueError:
            print("Invalid input. Please enter integers for x, y, and theta.")

    while True:
        try:
            gx = int(input("Enter goal x (in millimeters): "))
            gy = int(input("Enter goal y (in millimeters): "))
            gx = int(gx*0.5)
            gy = int(gy*0.5)
            
            if 0 <= gy < rows and 0 <= gx < cols:
                if map_gray[gy][gx] == 255:
                    goal_state = (gx, gy, 30)
                    break                       # getting input from user for goal point
                else:
                    print("The goal lies in an obstacle. Please enter a valid point.")
            else:
                print("Coordinates out of bounds. Please enter values within map dimensions.")
        except ValueError:
            print("Invalid input. Please enter integers for x, y.")
    

    while True:
        try:
            rpm1 = int(input("Enter RPM1 (1 - 10): "))
            rpm2 = int(input("Enter RPM2 (10 - 20): "))
            break
        except ValueError:                  # getting input from user for RPMs
            print("Invalid input. Please enter integers for RPM1 and RPM2")

    frame_width = 640
    frame_height = 480
    fps = 360       # opencv video writer parameters
    output_file = 'output_video.mp4'

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'mp4v' for .mp4 output
    out = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

    # Run A* search
    map_rgb = cv2.cvtColor(map_gray, cv2.COLOR_GRAY2RGB)
    path = a_star_search(map_gray,map_rgb, start_state, goal_state, out, rpm1, rpm2)        # main function to run the search
    print(' Video Saved! ')
    out.release()
    plt.imshow(map_rgb)
    plt.show()
    print("Found path:")
    print(path)

    path_original_scale = []

    for i in path:          # getting the path that is used as waypoints for gazebo 
        path_original_scale.append(((i[0]/0.5) / 1000 , (i[1]/0.5 - start_state[1]/0.5) / 1000, i[2]))

    print(' Planner Way points ')

    print(path_original_scale)

    # Save planner waypoints to a file which is read by gazebo

    with open("config/planner_waypoints.txt", "w") as file:
        for coord in path_original_scale:
            file.write(f"{coord[0]:.3f}, {coord[1]:.3f}, {coord[2]:.1f}\n")

    print("Planner waypoints saved to planner_waypoints.txt")


    
    # # Draw the complete path on the map image.
    # if path is not None:
    #     # Create a color version of the map
    #     map_path = cv2.cvtColor(map_gray, cv2.COLOR_GRAY2BGR)
    #     for i in range(1, len(path)):
    #         pt1 = (path[i-1][0], np.shape(map_path)[0] - path[i-1][1])
    #         pt2 = (path[i][0], np.shape(map_path)[0] - path[i][1])
    #         cv2.line(map_path, pt1, pt2, (0, 0, 255), thickness=2)

    #     #plt.imshow(map_path)
    #     #plt.show()
