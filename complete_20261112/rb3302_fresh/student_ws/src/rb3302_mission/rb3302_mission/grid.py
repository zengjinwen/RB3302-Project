"""58cm字母格A*；细地图只判断中心连线是否碰墙。"""
import heapq
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))



class MapPlanner:
    def __init__(self, msg, points, clearance=.22):
        self.points = {name: tuple(point) for name, point in points.items()}
        resolution = msg.info.resolution
        width, height = msg.info.width, msg.info.height
        origin = msg.info.origin.position
        blocked = set()
        n = math.ceil(clearance/resolution)
        offsets = [(x,y) for x in range(-n,n+1) for y in range(-n,n+1)
                   if math.hypot(x,y)*resolution <= clearance+1e-6]
        for i, value in enumerate(msg.data):
            if value < 0 or value >= 50:
                blocked.update((i%width+x, i//width+y) for x,y in offsets)

        def clear_line(a, b):
            steps = max(1,math.ceil(math.dist(a,b)/(resolution/2)))
            for i in range(steps+1):
                x = math.floor((a[0]+(b[0]-a[0])*i/steps-origin.x)/resolution)
                y = math.floor((a[1]+(b[1]-a[1])*i/steps-origin.y)/resolution)
                if not (0 <= x < width and 0 <= y < height) or (x,y) in blocked:
                    return False
            return True

        self.clear_line = clear_line
        self.neighbors = {name: [] for name in self.points}
        for name, a in self.points.items():
            for other, b in self.points.items():
                if (math.isclose(math.dist(a,b), .58)
                        and (math.isclose(a[0],b[0]) or math.isclose(a[1],b[1]))
                        and clear_line(a,b)):
                    self.neighbors[name].append(other)

    def plan(self, start, goal, blocked_cell=None):
        if start == blocked_cell or goal == blocked_cell:
            return []
        def distance(name):
            a,b = self.points[name],self.points[goal]
            return (abs(a[0]-b[0])+abs(a[1]-b[1]))/.58
        queue = [(distance(start), 0, start)]
        costs, parent = {start: 0}, {}
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost != costs[current]:
                continue
            if current == goal:
                route = [current]
                while route[-1] != start:
                    route.append(parent[route[-1]])
                return route[::-1]
            for nxt in self.neighbors[current]:
                if nxt == blocked_cell:
                    continue
                if cost+1 < costs.get(nxt, math.inf):
                    costs[nxt], parent[nxt] = cost+1, current
                    heapq.heappush(queue, (cost+1+distance(nxt),cost+1,nxt))
        return []

    def turning_points(self, route):
        result = route[:1]
        for before, current, after in zip(route, route[1:], route[2:]):
            a,b,c = (self.points[name] for name in (before,current,after))
            dx,dy = b[0]-a[0],b[1]-a[1]
            ex,ey = c[0]-b[0],c[1]-b[1]
            if abs(dx*ey-dy*ex)>1e-9 or dx*ex+dy*ey<=0:
                result.append(current)
        if len(route)>1:
            result.append(route[-1])
        return result
