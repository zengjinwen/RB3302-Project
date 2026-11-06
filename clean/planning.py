"""60 cm cardinal A*, static footprint clearance, one occupied letter cell."""
import heapq
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Planner:
    def __init__(self, grid, points, clearance):
        self.grid, self.points = grid, {k: tuple(v) for k,v in points.items()}
        self.res = grid.info.resolution
        self.width, self.height = grid.info.width, grid.info.height
        self.origin = grid.info.origin.position
        q = grid.info.origin.orientation
        self.rotation = 2*math.atan2(q.z,q.w)
        self.occupied = set()
        self.wall_band = set()
        n = math.ceil(clearance/self.res)
        offsets = [(x,y) for x in range(-n,n+1) for y in range(-n,n+1)
                   if math.hypot(x,y)*self.res <= clearance]
        for i,v in enumerate(grid.data):
            if v < 0 or v >= 50:
                self.occupied.update((i%self.width+x,i//self.width+y) for x,y in offsets)
                self.wall_band.update((i%self.width+x,i//self.width+y) for x in range(-3,4) for y in range(-3,4) if math.hypot(x,y)*self.res<=.12)
        self.edges = {k: [] for k in points}
        for a,pa in self.points.items():
            for b,pb in self.points.items():
                if math.isclose(math.dist(pa,pb),.6) and (math.isclose(pa[0],pb[0]) or math.isclose(pa[1],pb[1])) and self.clear(pa,pb):
                    self.edges[a].append(b)
        self.blocked = None

    def pixel(self,p):
        x,y=p[0]-self.origin.x,p[1]-self.origin.y
        c,s=math.cos(self.rotation),math.sin(self.rotation)
        return math.floor((c*x+s*y)/self.res), math.floor((-s*x+c*y)/self.res)

    def static_hit(self,p):
        x,y=self.pixel(p)
        return not (0<=x<self.width and 0<=y<self.height) or self.grid.data[y*self.width+x] != 0

    def clear(self,a,b):
        n=max(1,math.ceil(math.dist(a,b)/(self.res/2)))
        for i in range(n+1):
            x,y=self.pixel((a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n))
            if not (0<=x<self.width and 0<=y<self.height) or (x,y) in self.occupied:return False
        return True

    def path(self,position,goal):
        # Connect actual position only to visible centres; never connect across a wall.
        starts=sorted((math.dist(position,p),k) for k,p in self.points.items()
                      if k!=self.blocked and self.clear(position,p))
        if not starts or goal==self.blocked:return []
        # Closest reachable centre anchors the route; no arbitrary long shortcuts.
        start=starts[0][1]
        def h(k):return math.dist(self.points[k],self.points[goal])
        queue=[(h(start),0.,start)];costs={start:0.};parent={}
        while queue:
            _,cost,k=heapq.heappop(queue)
            if cost!=costs[k]:continue
            if k==goal:
                result=[k]
                while k in parent:k=parent[k];result.append(k)
                return result[::-1]
            for nxt in self.edges[k]:
                if nxt==self.blocked:continue
                value=cost+.6
                if value<costs.get(nxt,math.inf):
                    costs[nxt]=value;parent[nxt]=k;heapq.heappush(queue,(value+h(nxt),value,nxt))
        return []

    def corners(self,names):
        out=names[:1]
        for a,b,c in zip(names,names[1:],names[2:]):
            p,q,r=(self.points[k] for k in (a,b,c))
            if abs((q[0]-p[0])*(r[1]-q[1])-(q[1]-p[1])*(r[0]-q[0]))>1e-6:out.append(b)
        if len(names)>1:out.append(names[-1])
        return out
