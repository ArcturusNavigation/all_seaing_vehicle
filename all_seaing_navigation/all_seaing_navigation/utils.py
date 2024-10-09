import heapq

''' These data structures can be used in the search function
First run
ros2 launch path_planning sim_plan.launch.xml
then
ros2 launch racecar_simulator simulate.launch.xml
ssh -L 6081:localhost:6081 racecar@192.168.1.85
'''

class PriorityQueue:
    """
    Priority Queue implementation using heapq
    """
    def __init__(self):
        self.elements = []
        self.element_set = set()

    def empty(self):
        return len(self.elements) == 0

    def put(self, item):
        heapq.heappush(self.elements, item)
        self.element_set.add(item)

    def get(self):
        return heapq.heappop(self.elements)

    def __contains__(self,item):
        return item in self.element_set
