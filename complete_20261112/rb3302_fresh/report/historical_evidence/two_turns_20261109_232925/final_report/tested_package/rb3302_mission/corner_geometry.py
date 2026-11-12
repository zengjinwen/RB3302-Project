"""Pure geometry for the local RPP experiment; no ROS dependencies."""
import math


def turn_windows(route, points):
    windows = []
    for i in range(1,len(route)-1):
        a,b,c = (points[n] for n in route[i-1:i+2])
        u,v = (b[0]-a[0],b[1]-a[1]),(c[0]-b[0],c[1]-b[1])
        if abs(u[0]*v[1]-u[1]*v[0]) > 1e-9:
            first,last = i-1,i+1
            if windows and first <= windows[-1][1]:
                windows[-1] = (windows[-1][0],last)
            else:
                windows.append((first,last))
    actions, cursor = [],0
    for first,last in windows:
        if first > cursor:
            actions.append(('line',cursor,first))
        actions.append(('rpp',first,last))
        cursor = last
    if cursor < len(route)-1:
        actions.append(('line',cursor,len(route)-1))
    return actions


def lengths(points):
    result = [0.]
    for a,b in zip(points,points[1:]):
        result.append(result[-1]+math.dist(a,b))
    return result


def project(position, points, distances):
    choices = []
    for i,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy = b[0]-a[0],b[1]-a[1]
        t = max(0.,min(1.,((position[0]-a[0])*dx+(position[1]-a[1])*dy)/(dx*dx+dy*dy)))
        q = (a[0]+t*dx,a[1]+t*dy)
        choices.append((math.dist(position,q),i,distances[i]+t*(distances[i+1]-distances[i])))
    _,index,progress = min(choices)
    return index,progress


def point_at(points, distances, progress):
    for i in range(len(points)-1):
        if progress <= distances[i+1]:
            t = max(0.,(progress-distances[i])/(distances[i+1]-distances[i]))
            a,b = points[i:i+2]
            return a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])
    # Keep the geometric target ahead on the final centreline, even near
    # the endpoint. Arrival still uses the ORIGINAL endpoint plane.
    a,b = points[-2:]
    scale = (progress-distances[-1])/math.dist(a,b)
    return b[0]+scale*(b[0]-a[0]),b[1]+scale*(b[1]-a[1])


def pursuit(position, heading, target):
    dx,dy = target[0]-position[0],target[1]-position[1]
    distance2 = dx*dx+dy*dy
    return 0. if distance2 < 1e-12 else 2.*(-math.sin(heading)*dx+math.cos(heading)*dy)/distance2
