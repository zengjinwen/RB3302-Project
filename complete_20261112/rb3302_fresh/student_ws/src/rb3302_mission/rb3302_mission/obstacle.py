"""Detect a stable broad obstruction in a route cell, without a box-shape fit."""
import math


class CellObstacle:
    def __init__(self, points, map_msg, confirm_time=.3, min_width=.20, boundary_tolerance=.20):
        self.points = points
        self.boundary_tolerance = boundary_tolerance
        self.confirm_time,self.min_width = confirm_time,min_width
        self.blocked = self.candidate = None
        self.since = self.last_stamp = None
        self.count = 0
        info = map_msg.info
        self.resolution = info.resolution
        self.origin = (info.origin.position.x,info.origin.position.y)
        self.width,self.height,self.data = info.width,info.height,map_msg.data

    def static_free(self, x, y):
        ix = math.floor((x-self.origin[0])/self.resolution)
        iy = math.floor((y-self.origin[1])/self.resolution)
        # Ignore hits within two map pixels (about 10cm) of known walls/unknown space.
        for dx in range(-2,3):
            for dy in range(-2,3):
                j,k = ix+dx,iy+dy
                if not (0 <= j < self.width and 0 <= k < self.height):
                    return False
                value = self.data[k*self.width+j]
                if value < 0 or value >= 50:
                    return False
        return True

    def visible_in_map(self, laser, hit):
        # A return behind an existing mapped wall cannot be a new obstacle.
        length = math.dist(laser,hit)
        steps = max(1,math.ceil(length/(self.resolution*.5)))
        for i in range(1,steps):
            x = laser[0]+(hit[0]-laser[0])*i/steps
            y = laser[1]+(hit[1]-laser[1])*i/steps
            ix = math.floor((x-self.origin[0])/self.resolution)
            iy = math.floor((y-self.origin[1])/self.resolution)
            if not (0<=ix<self.width and 0<=iy<self.height):
                return False
            value = self.data[iy*self.width+ix]
            if value<0 or value>=50:
                return False
        return True

    def detect_cells(self, hits, laser, cells):
        free_hits = [(x,y) for x,y in hits if self.visible_in_map(laser,(x,y))]
        matches = []
        for name in cells:
            cx,cy = self.points[name]
            dx,dy = laser[0]-cx,laser[1]-cy
            axis = 0 if abs(dx)>=abs(dy) else 1
            centre = (cx,cy)
            direction = math.copysign(1.,dx if axis==0 else dy)
            near_face = centre[axis]+direction*.29
            # Only the neighbour's near face; never its far or side faces.
            local = [(hit[axis],hit[1-axis]-centre[1-axis])
                     for hit in free_hits
                     if abs(hit[axis]-near_face)<=self.boundary_tolerance
                     and abs(hit[1-axis]-centre[1-axis])<=.29]
            # A broad surface must exist at one depth; scattered returns do not count.
            for depth,_ in local:
                surface = [(d,lateral) for d,lateral in local if depth<=d<=depth+.12]
                across = [lateral for _,lateral in surface]
                if (len(across)<5 or max(across)-min(across)<self.min_width
                        or not min(across)<0<max(across)):
                    continue
                # Fit depth against transverse position: a blocking face is axis-aligned.
                support = 0
                for d,lateral in surface:
                    point = [cx,cy]
                    point[axis],point[1-axis] = d,centre[1-axis]+lateral
                    support += self.static_free(*point)
                if support<5:
                    continue
                mean_d = sum(d for d,_ in surface)/len(surface)
                mean_l = sum(across)/len(across)
                variance = sum((l-mean_l)**2 for l in across)
                slope = sum((l-mean_l)*(d-mean_d) for d,l in surface)/variance
                residual = math.sqrt(sum((d-mean_d-slope*(l-mean_l))**2
                                         for d,l in surface)/len(surface))
                if abs(slope)>math.tan(math.radians(10)) or residual>.025:
                    continue
                matches.append((math.hypot(dx,dy),name))
                break
        return [name for _,name in sorted(matches)]

    def detect(self, hits, laser, cells):
        matches = self.detect_cells(hits,laser,cells)
        return matches[0] if matches else None

    def observe(self, candidate, stamp):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        gap = self.last_stamp is None or stamp-self.last_stamp>1.
        self.last_stamp = stamp
        if gap or candidate != self.candidate:
            self.candidate,self.since,self.count = candidate,stamp,0
        self.count += 1
        if (candidate is not None and candidate != self.blocked and self.count>=3
                and stamp-self.since>=self.confirm_time):
            self.blocked = candidate  # Replace, never accumulate old obstacle cells.
            return True
        return False
