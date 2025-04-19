import parameters.simulation_parameters as SIM
from models.agent import Agent
from models.models import HSV_6DOF, StationaryTarget

def init_agents(ally_vehicles_input = [], enemy_vehicles_input = [], targets_input = []):
    ally_vehicles = [None]*len(ally_vehicles_input)
    enemy_vehicles = [None]*len(enemy_vehicles_input)
    targets = [None]*len(targets_input)
    
    fill_agent_list(ally_vehicles, ally_vehicles_input)
    fill_agent_list(enemy_vehicles, enemy_vehicles_input)
    fill_agent_list(targets, targets_input)
    
    return ally_vehicles, enemy_vehicles, targets
      
def fill_agent_list(agent_list, agent_input):
    for i in range(len(agent_list)):
        agent_list[i] = Agent(i)
        model_type = agent_input[i][0]
        init_state = agent_input[i][1]
        if model_type == "6DOF":
            model_type_obj = HSV_6DOF()
        elif model_type == "stationary":
            model_type_obj = StationaryTarget()
        else:
            raise ValueError("\"" + model_type + "\" is not a valid model type")
        
        agent_list[i].set_dynamics(model_type_obj, init_state)
            