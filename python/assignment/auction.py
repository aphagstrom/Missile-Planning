# auction
#   - Target assignment based on the auction algorithm by Bertsekas
#     - Update history:  
#         12/07/2023 - JTC

import numpy as np
import matplotlib.pyplot as plt

class Assignment(object):
    def __init__(self, agents, targets):
        # Set arrays of agents and targets to instance variables
        self.agents = agents
        self.targets = targets
        # Set number of agents and targets to instance variables
        self.num_agents = len(agents)
        self.num_targets = len(targets)
        # Set priority of targets to instance variables
        self.target_priority = priority
        # Set constant for relaxation of complementary slackness condition
        self.epsilon = 1 / (100 * self.num_targets)

        # Initialize a dictionary to hold which target agents are assigned to
        self.assignments = {x: {} for x in agents}

        self.generate_assignments()

    def generate_assignments(self):
        all_assigned = False
        while not all_assigned:
            break
        return None

    def bidding_phase(self):
        return None
    
    def assignment_phase(self):
        return None

class Auction(Assignment):
