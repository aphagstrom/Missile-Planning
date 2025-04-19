

from charts.data_writer import DataWriter
from planners.dubins_extender import DubinsExtender

def sync_dubins_paths(ally_vehicles, targets):
    
    # get default dubins paths
    for i in range(len(targets)):
        hsv = [vehicle for vehicle in ally_vehicles if vehicle.target == targets[i]] # list of vehicles assigned to target
        # print("hsv target state 0 is:",hsv[0].target.state.pos)
        # print("hsv target state 1 is:",hsv[1].target.state.pos)
        max_path = get_max_path(hsv) 
        
        # data_writer = DataWriter()
    
        for i in range(len(hsv)):
            if hsv[i].id != max_path[0]:
                # print("Extending path of Missile", hsv[i].id)
                dubins_extender = DubinsExtender(hsv[i], max_path)
                hsv[i].path_manager.waypoints = dubins_extender.extend_dubins() # extend the path of every missile to match length of default
                # data_writer.write_dubins_circles(i, path_manager[i].dubins_path)
            # else:
                # data_writer.write_dubins_circles(i, path_manager[i].dubins_path)
            
def get_max_path(hsv):
    max_path = [None, 0] # missile ID, then max path length
    for i in range(len(hsv)):
        # print("length of hsv is:",len(hsv))
        hsv[i].path_manager.update(hsv[i])
        if hsv[i].path_manager.dubins_path.length > max_path[1]:
            max_path[0] = hsv[i].id
            max_path[1] =  hsv[i].path_manager.dubins_path.length 
    # print("path lengths 0 are:",hsv[0].path_manager.dubins_path.length) 
    # print("path lengths 1 are:",hsv[1].path_manager.dubins_path.length) 
            # print("path lengths 1 are:",hsv[1].path_manager.dubins_path.length) 
    # print("Max path is length", max_path[1], "for Missile", max_path[0], "\n")    
    return max_path

# from planners.dubins_extender import DubinsExtender

# def sync_dubins_paths(ally_vehicles, targets):
#     # Get default Dubins paths
#     for target in targets:
#         hsv = [vehicle for vehicle in ally_vehicles if vehicle.target == target]
#         print("hsv is:", hsv)
#         max_path = get_max_path(hsv) 

#         for vehicle in hsv:
#             if vehicle.id != max_path[0]:
#                 print("Extending path of Missile", vehicle.id)
#                 dubins_extender = DubinsExtender(vehicle, max_path)
#                 vehicle.path_manager.waypoints = dubins_extender.extend_dubins()

# def get_max_path(hsv):
#     max_path = [None, 0]  # [missile ID, max path length]
#     for vehicle in hsv:
#         vehicle.path_manager.update(vehicle)
#         if vehicle.path_manager.dubins_path.length > max_path[1]:
#             max_path[0] = vehicle.id
#             max_path[1] = vehicle.path_manager.dubins_path.length   
#     print("Max path is length", max_path[1], "for Missile", max_path[0], "\n")    
#     return max_path
