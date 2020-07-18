from helpers import load_map_10, load_map_40
import math

# Do not change this cell
# When you write your methods correctly this cell will execute
# without problems
class PathPlanner():
    """Construct a PathPlanner Object"""

    def __init__(self, M, start=None, goal=None):
        """ """
        self.map = M
        self.start = start
        self.goal = goal
        self.closedSet = self.create_closedSet() if goal != None and start != None else None
        self.openSet = self.create_openSet() if goal != None and start != None else None
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
        self.gScore = self.create_gScore() if goal != None and start != None else None
        self.fScore = self.create_fScore() if goal != None and start != None else None
        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None

    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        total_path = [current]
        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            total_path.append(current)
        return total_path

    def _reset(self):
        """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        """ """
        if self.map == None:
            raise (ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
        if self.goal == None:
            raise (
            ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
        if self.start == None:
            raise (
            ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.closedSet = self.closedSet if self.closedSet != None else self.create_closedSet()
        self.openSet = self.openSet if self.openSet != None else self.create_openSet()
        self.cameFrom = self.cameFrom if self.cameFrom != None else self.create_cameFrom()
        self.gScore = self.gScore if self.gScore != None else self.create_gScore()
        self.fScore = self.fScore if self.fScore != None else self.create_fScore()

        while not self.is_open_empty():
            current = self.get_current_node()

            if current == self.goal:
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                self.openSet.remove(current)
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    continue  # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:  # Discover a new node
                    self.openSet.add(neighbor)

                # The distance from start to a neighbor
                # the "dist_between" function may vary as per the solution requirements.
                # if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                #     continue  # This is not a better path.
                if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    continue  # This is not a better path.

                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)

        print("No Path Found")
        self.path = None
        return False

    # INITIALIZE DATA STRUCTURES
    def create_closedSet(self):
        """ Creates and returns a data structure suitable to hold the set of nodes already evaluated"""
        # EXAMPLE: return a data structure suitable to hold the set of nodes already evaluated

        # data structure type: set
        return set()

    def create_openSet(self):
        """ Creates and returns a data structure suitable to hold the set of currently discovered nodes
        that are not evaluated yet. Initially, only the start node is known."""
        if self.start != None:
            # TODO: return a data structure suitable to hold the set of currently discovered nodes
            # that are not evaluated yet. Make sure to include the start node.

            # data structure type: set
            # initialize with the start node
            return set({self.start})

        raise (
        ValueError, "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")

    def create_cameFrom(self):
        """Creates and returns a data structure that shows which node can most efficiently be reached from another,
        for each node."""
        # TODO: return a data structure that shows which node can most efficiently be reached from another,
        # for each node.

        # data structure type: dictionary
        # key: node
        # value: preceding node
        return {}

    def init_score(self, start_score):
        # initialize scoring containers with value set to infinity
        # for each location in the map

        # uses dictionary comprehension learned from
        # https://www.geeksforgeeks.org/python-dictionary-comprehension/
        score = {key: math.inf for key in self.map.intersections}

        # add start node fScore
        score[self.start] = start_score

        return score

    def create_gScore(self):
        """Creates and returns a data structure that holds the cost of getting from the start node to that node,
        for each node. The cost of going from start to start is zero."""
        # TODO:  return a data structure that holds the cost of getting from the start node to that node, for each node.
        # The cost of going from start to start is zero. The rest of the node's values should
        # be set to infinity.

        # data structure type: dictionary
        # key: node
        # value: gScore
        return self.init_score(0)

    def create_fScore(self):
        """Creates and returns a data structure that holds the total cost of getting from the start node to the goal
        by passing by that node, for each node. That value is partly known, partly heuristic.
        For the first node, that value is completely heuristic."""
        # TODO: return a data structure that holds the total cost of getting from the start node to the goal
        # by passing by that node, for each node. That value is partly known, partly heuristic.
        # For the first node, that value is completely heuristic. The rest of the node's value should be
        # set to infinity.

        # data structure type: dictionary
        # key: node
        # value: fScore

        return self.init_score(self.heuristic_cost_estimate(self.start))

    # SET VARIABLES
    def set_map(self, M):
        """Method used to set map attribute """
        self._reset()
        #self._reset(self)
        self.start = None
        self.goal = None
        # TODO: Set map to new value.
        self.map = M

    def set_start(self, start):
        """Method used to set start attribute """
        self._reset()
        #self._reset(self)
        # TODO: Set start value. Remember to remove goal, closedSet, openSet, cameFrom, gScore, fScore,
        # and path attributes' values.
        self.start = start

    def set_goal(self, goal):
        """Method used to set goal attribute """
        self._reset()
        #self._reset(self)
        # TODO: Set goal value.
        self.goal = goal

    # GET NODE INFORMATION
    def is_open_empty(self):
        """returns True if the open set is empty. False otherwise. """
        # TODO: Return True if the open set is empty. False otherwise.
        return len(self.openSet) == 0

    def get_current_node(self):
        """ Returns the node in the open set with the lowest value of f(node)."""
        # TODO: Return the node in the open set with the lowest value of f(node).

        ###
        ###
        # The following commented out code was my original solution
        # The code following was suggested by my reviwer.
        ###
        ###

        # search thru openSet, cross referencing with fScore to determine the node
        # with the lowest fScore
        # current_fscore_value = 0
        # min_fscore_value = math.inf   # learned how to define infinity from geeksforgeeks.com
        # min_fscore_node = math.inf
        #
        # for node in self.openSet:
        #     current_fscore_value = self.fScore[node]
        #     if current_fscore_value < min_fscore_value:
        #         min_fscore_value = current_fscore_value
        #         min_fscore_node = node
        # return min_fscore_node

        # really cool suggestion made by my project reviewer
        # that uses both data structures to achieve O(n) time
        return min(self.openSet, key=self.fScore.get)

    def get_neighbors(self, node):
        """Returns the neighbors of a node"""
        # TODO: Return the neighbors of a node
        # get neighbors from map
        return self.map.roads[node]

    # SCORES AND COSTS
    def get_gScore(self, node):
        """Returns the g Score of a node"""
        # TODO: Return the g Score of a node
        return self.gScore[node]

    def distance(self, node_1, node_2):
        """ Computes the Euclidean L2 Distance"""
        # TODO: Compute and return the Euclidean L2 Distance

        # formula sourced from
        # https://www.w3resource.com/python-exercises/math/python-math-exercise-79.php

        node_1_pos_x = self.map.intersections[node_1][0]
        node_1_pos_y = self.map.intersections[node_1][1]
        node_2_pos_x = self.map.intersections[node_2][0]
        node_2_pos_y = self.map.intersections[node_2][1]
        return math.sqrt((node_1_pos_x - node_2_pos_x) ** 2 + (node_1_pos_y - node_2_pos_y) ** 2)

    def get_tentative_gScore(self, current, neighbor):
        """Returns the tentative g Score of a node"""
        # TODO: Return the g Score of the current node
        # plus distance from the current node to it's neighbors
        return self.gScore[current] + self.distance(current, neighbor)

    def heuristic_cost_estimate(self, node):
        """ Returns the heuristic cost estimate of a node """
        # TODO: Return the heuristic cost estimate of a node

        # "as the crow flies" distance between node and goal
        # is guaranteed to be admissible
        return self.distance(node, self.goal)

    def calculate_fscore(self, node):
        """Calculate the f score of a node. """
        # TODO: Calculate and returns the f score of a node.
        # REMEMBER F = G + H
        return self.gScore[node] + self.heuristic_cost_estimate(node)

    # RECORD THE BEST PATH
    def record_best_path_to(self, current, neighbor):
        """Record the best path to a node """
        # TODO: Record the best path to a node, by updating cameFrom, gScore, and fScore
        self.cameFrom[neighbor] = current
        self.gScore[neighbor] = self.get_tentative_gScore(current, neighbor)
        self.fScore[neighbor] = self.calculate_fscore(neighbor)
        if neighbor not in self.openSet:
            self.openSet.add(neighbor)

def main():
    # interactive test
    map_10 = load_map_10()
    map_40 = load_map_40()

    print(f"Testing path planner...")
    start_node = 5
    goal_node = 34
    planner = PathPlanner(map_40, start_node, goal_node)
    path = planner.path

    print(f"Start Node: {start_node}")
    print(f"Goal Node: {goal_node}")
    print(f"Resulting path: {path}")
    if path == [5, 16, 37, 12, 34]:
        print(f"Great! Your code works for these inputs!\n")
    else:
        print("Something is off, your code produced the following:")
        print(path, "\n")

    # course test for validation of project
    from test import test
    print(f"Running project validation tests from Udacity...")
    test(PathPlanner)

if __name__ == "__main__":
    main()