import matplotlib.pyplot as plt
import numpy as np
import modules.ogmap as ogmap
from modules import prm
import modules.func as func
import time
import modules.cubic_spline_path as csp
import modules.b_spline_path as b


show_animation = True
prm = prm.PRM() # probabilistic road map class





def b_spline_course(x, y, sampling_number=10):
    # way points
    way_point_x = x
    way_point_y = y
    n_course_point = sampling_number  # sampling number

    rax, ray = b.approximate_b_spline_path(way_point_x, way_point_y,
                                         n_course_point)
    rix, riy = b.interpolate_b_spline_path(way_point_x, way_point_y,
                                         n_course_point)

    return rax, ray, rix, riy

###################################################################################
def cubic_spline_course(x, y, ds=0.1):
    sp = csp.Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk



def actual_to_grid_pos(actual_pos, map_resolution):
    return round(actual_pos/map_resolution)


def plan_path(map_image_file, robot_radius_cm, map_resolution, current_loc, target_loc, obs_pos=()):
    x, y = 0, 1

    robot_radius_in_grid = robot_radius_cm/map_resolution
    new_map = ogmap.get_obs_map(map_image_file, robot_radius_cm, map_resolution)

    min_x = new_map['min_x']
    min_y = new_map['min_y']
    max_x = new_map['max_x']
    max_y = new_map['max_y']
    x_width = new_map['x_width']
    y_width = new_map['y_width']
    ox = new_map['ox']
    oy = new_map['oy']
    obstacle_map = new_map['map']


    ###############################################################
    ##### adding obstacle to the created map ######################
    ###############################################################
    if len(obs_pos) != 0:
    
        for loc in range(0, len(obs_pos)):
            polygon_vertices_x = obs_pos[loc][x] # x_cordinates of the polygon
            polygon_vertices_y = obs_pos[loc][y] # y_cordinates of the polygon
            new_ox, new_oy = func.draw_polygon(polygon_vertices_x, polygon_vertices_y)
            obstacle_map = func.calc_obstacle_map(new_ox, new_oy, robot_radius_in_grid, obstacle_map)
            for i in range(0,len(new_ox)):
                ox.append(new_ox[i])
                oy.append(new_oy[i])
            pass
        

    # print map info
    print("MAP INFORMATION")
    print("x_width:", x_width, "grid cells")
    print("y_width:", y_width, "grid cells")
    print("map resolution:", map_resolution, "cm/grid")
    print("actual_x_width:", x_width*map_resolution, "cm")
    print("actual_y_width:", y_width*map_resolution, "cm")
    print()


    plt.plot(ox, oy, ".k")

    plt.plot(current_loc[x], current_loc[y], marker = '.', ms = 20, mfc = 'k')
    plt.plot(target_loc[x], target_loc[y], marker = '.', ms = 20, mfc = 'b')
    
    plt.grid(True)
    plt.axis("equal")


    rx, ry = prm.prm_planning(current_loc[x], current_loc[y], target_loc[x], target_loc[y], ox, oy, robot_radius_in_grid)

    assert rx, 'Cannot found path'
    
    px, py = func.remove_redundant_path(rx,ry)
    prm_path_x, prm_path_y = func.reverse_list(px), func.reverse_list(py)
    

    plt.plot(ox, oy, ".k")

    plt.plot(current_loc[x], current_loc[y], "ok")
    plt.plot(target_loc[x], target_loc[y], "ob")
    
    plt.grid(True)
    plt.axis("equal")

    plt.plot(prm_path_x, prm_path_y, "-b")


    # Rx,Ry,ryaw,rk = cubic_spline_course(prm_path_x, prm_path_y)
    # plt.plot(Rx, Ry, "-r")


    rax, ray, rix, riy = b_spline_course(prm_path_x, prm_path_y)
    plt.plot(rax, ray, '-r', label="Approximated B-Spline path")

    # plt.plot(rix, riy, '-r', label="Interpolated B-Spline path")


    plt.pause(0.01)

    
    # return prm_path_x, prm_path_y
    return rax, ray

###################################################################################














if __name__ == '__main__':

    start_time = time.time()

    map_image_file = "maps/test/floor_map_reduced.png"
    robot_radius_cm = 15 # in cm
    map_resolution = 10 # cm per grid based on the map

    # start and goal grid position and not the actual pos in cm
    sx, sy = 42, 75
    gx, gy = 94, 13

    # # start and goal actual position in cm
    # sx, sy = 120, 3080
    # gx, gy = 1650, 160
    # sx, sy, gx, gy = actual_to_grid_pos(sx, sy, gx, gy, map_resolution)


    start = [sx, sy]
    goal = [gx, gy]



    # plans and gets the generated path
    
    px, py = plan_path(map_image_file, robot_radius_cm, map_resolution, start, goal)
    
    stop_time = time.time()

    run_time = stop_time-start_time
    print("run_time :",run_time)



    path_coord, _, point_index = func.get_path_coord(px, py, map_resolution)
    path_dist, path_total_dist, _, _ = func.get_total_path_dist(px, py, map_resolution) 
    path_angles = func.get_path_angles(px, py)

    # print(len(path_coord))
    # print(len(path_dist))
    # print(len(path_coord))

    print("total_dist :", path_total_dist*map_resolution, "cm")

    
    func.print_data(point_index, path_coord, path_dist, path_angles)
    
    # show planned map
    plt.show()