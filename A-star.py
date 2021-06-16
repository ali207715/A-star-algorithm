import env
import math
import time
import os


class Agent(kuimaze.BaseAgent):
    """
    Agent class created for maze solving.
    """

    def __init__(self, environment):  # Initializing class
        self.environment = environment

    def find_path(self):
        """
        Method that solves for the shortest path towards goal.
        Expects to return a path_section as a list of positions [(x1, y1), (x2, y2), ... ].

        Each state will be in the form of a list, and have indexes referring to certain characteristics of the state.
        The format will be - [(position), g_value, f_value, (parent's position)]
        """
        observation = self.environment.reset()  # must be called first, it is necessary for maze initialization.
        open_list = []  # a list of possible positions yet to be chosen.
        closed_list = []  # a list of positions chosen for the shortest possible route to the goal.

        def f_value(post, goal, g_val):
            """
            Determining the f_value of the state.

            :param post: The position of the current state.
            :param goal: The position(x,y) of the goal.
            :param g_val: Describes the distance from the start position.
            :return: the f_value, which is a combination of expected distance to goal + distance from start position.
            """

            h_value = math.sqrt(((post[0] - goal[0]) ** 2) + ((post[1] - goal[1]) ** 2))
            # Determining the heuristic value, the pythagorean theorem is utilized in this case.
            return g_val + h_value

        end_pos = observation[1][0:2]  # The position (x,y) of the goal.
        start_pos = observation[0][0:2]  # start position (x,y).
        open_list.append([start_pos])
        open_list[0].append(0)  # Attaching g_value to the start position.
        open_list[0].append(f_value(open_list[0][0], end_pos, 0))  # Attaching f_value.
        open_list[0].append(0)  # Attaching non-existent parent as 0.
        while True:  # Infinite loop
            # self.environment.render()
            # time.sleep(0.1)
            # Every loop, we confirm the open_list is not empty, which hints at no path being found.
            if len(open_list) == 0:
                return None

            else:
                open_list.sort(key=lambda x: x[2])  # Sorting list according its f-cost.
                current = open_list.pop(0)  # Smallest value is the first value.
                closed_list.append(current)  # Smallest value now being considered as a potential position.

                if current[0] == end_pos:  # Checking if the algorithm has reached its goal.
                    # If yes, then going to backtrack using the parent characteristic present for all states.
                    final_path = []  # Initializing final path list for output.
                    while current[3] != 0:  # As long as the parent isn't the start value, keep looping.
                        final_path.append(current[0])
                        for pos in closed_list:  # Searching for the parent.
                            if pos[0] == current[3]:
                                current = pos  # Parent found, setting the parent as the new "child".
                                break
                    final_path.append(start_pos)  # Appending the start position.

                    # self.environment.render()
                    # time.sleep(0.1)
                    return final_path[::-1]
                    # Goal has not reached, thus we expand current state and generate children.
                children_d = self.environment.expand(current[0])
                children = []  # Initializing a list for the "good" children.
                for child in children_d:  # Isolating non-diagonal values.
                    if child[1] == 1.0:
                        children.append(child)
                for child in children:
                    mem_ship = any(child[0] in pos for pos in closed_list)
                    # Checking if child already present in the closed_list. Returns True or False.
                    if not mem_ship:
                        child[1] += current[1]  # Assigning its true distance from the start position.
                        f_v = f_value(child[0], end_pos, child[1])
                        child.append(f_v)  # Stores its f_value.
                        child.append(current[0])  # Store its parent information.
                        open_list_presence = False  # Confirms if child is in open_list.
                        if len(open_list) != 0:
                            for open_node in open_list:
                                if child == open_node:
                                    # Checking if state already present in open_list.
                                    open_list_presence = True
                                    break
                                elif child[0] == open_node[0] and child[1] < open_node[1]:
                                    # If child's position present and has a shorter path, update the node.
                                    open_node[1] = child[1]
                                    open_node[3] = child[3]
                                    open_node[2] = child[2]
                                    open_list_presence = True
                                    break
                                elif child[0] == open_node[0]:
                                    #  If child has worse g_value than node.
                                    open_list_presence = True
                                    break
                        else:
                            open_list.append(child)  # Added to the list of possible candidates.

                        if not open_list_presence:
                            open_list.append(child)
                            # Completely new child, added to the list of possible candidates.


if __name__ == '__main__':

    MAP = 'maps/normal/normal11.bmp'
    MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), MAP)
    GRAD = (0, 0)
    SAVE_PATH = False
    SAVE_EPS = False

    env = kuimaze.InfEasyMaze(map_image=MAP, grad=GRAD)  # For using random map set: map_image=None
    agent = Agent(env)

    path = agent.find_path()
    env.set_path(path)  # set path it should go from the init state to the goal state
    if SAVE_PATH:
        env.save_path()  # save path of agent to current directory
    if SAVE_EPS:
        env.save_eps()  # save rendered image to eps
    env.render(mode='human')
    time.sleep(100)

"""
References:

https://www.youtube.com/watch?v=PzEWHH2v3TE; Help with logic.
https://www.youtube.com/watch?v=-L-WgKMFuhE; Help with the logic.
https://cw.fel.cvut.cz/wiki/courses/be5b33kui/labs/search/start; General code source and guide.
http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html;  Theory.
https://cw.felk.cvut.cz/cmp/courses/be5b33kui/kuimaze_doc/index.html; Information concerning the kuimaze library.
"""