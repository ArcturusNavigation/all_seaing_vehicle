import heapq

class PriorityQueue:
    """
    Priority Queue implementation using heapq
    """
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item):
        heapq.heappush(self.elements, item)

    def get(self):
        return heapq.heappop(self.elements)

if __name__ == "__main__":
	main()
