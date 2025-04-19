import numpy as np
import parameters.simulation_parameters as SIM
from tools.get_dist import get_dist
from tools.angles import bound_angle

def calculate_impact(sim_time, prev_state, hsv):
    k = (hsv.target.state.pos[2, 0] - prev_state.pos[2, 0])/(hsv.state.pos[2, 0] - prev_state.pos[2, 0])

    impact_position = np.zeros(3)
    for j in range(len(impact_position)):
        impact_position[j] = prev_state.pos[j]+k*(hsv.state.pos[j] - prev_state.pos[j])
    impact_time = (sim_time - SIM.ts_simulation) + k*(SIM.ts_simulation)
    
    elev_angle = np.rad2deg(bound_angle(np.arctan(abs(prev_state.pos[2, 0] - impact_position[2])/get_dist(prev_state.pos[:2], impact_position[:2, np.newaxis]))))
    az_angle = np.rad2deg(bound_angle(np.arctan2(impact_position[1] - prev_state.pos[1, 0], impact_position[0] - prev_state.pos[0, 0])))
    
    # data_writer.write_impact(hsv.missile_id, impact_time, impact_position, elev_angle, az_angle, np.linalg.norm(hsv.state.vel))
    
    print("Missile ", hsv.id, ":", sep='')
    print("Impact time: ", impact_time, sep='')
    print("Impact position: ", impact_position, sep='')
    print("Error: ", np.sqrt((impact_position[0] - hsv.target.state.pos[0, 0])**2 + (impact_position[1] - hsv.target.state.pos[1, 0])**2), sep='')
    print("Elevation Angle: ", elev_angle, " deg", sep='')      
    print("Azimuth Angle: ", az_angle, " deg", sep='')   
    print("Missile Speed (m/s): ", np.linalg.norm(hsv.state.vel), "\n", sep='')
