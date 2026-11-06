"""Tangent quarter-circle geometry and differential-drive tracking."""
import math
from .grid import wrap, clamp


def corner_arc(a, b, c, radius):
    incoming = math.atan2(b[1]-a[1], b[0]-a[0])
    outgoing = math.atan2(c[1]-b[1], c[0]-b[0])
    turn = wrap(outgoing-incoming)
    if not math.isclose(abs(turn), math.pi/2):
        return None
    direction = math.copysign(1., turn)
    entry = (b[0]-radius*math.cos(incoming), b[1]-radius*math.sin(incoming))
    exit_point = (b[0]+radius*math.cos(outgoing), b[1]+radius*math.sin(outgoing))
    centre = (entry[0]+radius*math.cos(outgoing), entry[1]+radius*math.sin(outgoing))
    return entry, exit_point, centre, direction, incoming, outgoing


def arc_points(arc, radius):
    entry, _, centre, direction, _, _ = arc
    begin = math.atan2(entry[1]-centre[1], entry[0]-centre[0])
    return [(centre[0]+radius*math.cos(begin+direction*i*math.pi/80),
             centre[1]+radius*math.sin(begin+direction*i*math.pi/80)) for i in range(41)]


def arc_control(arc, radius, speed, pose, heading_gain, cross_gain, limit, heading_tolerance, min_turn):
    entry, _, centre, direction, _, _ = arc
    x,y,heading = pose
    begin = math.atan2(entry[1]-centre[1], entry[0]-centre[0])
    angle = math.atan2(y-centre[1], x-centre[0])
    progress = direction*wrap(angle-begin)
    radial_error = math.hypot(x-centre[0], y-centre[1])-radius
    tangent = angle+direction*math.pi/2
    omega = direction*speed/radius+heading_gain*wrap(tangent-heading)+direction*cross_gain*radial_error
    if progress >= math.pi/2:
        # Position alone must not hand off an unfinished turn to the next line.
        heading_error = wrap(arc[5]-heading)
        if abs(heading_error) <= heading_tolerance:
            return 0., 0., True, radial_error
        omega = math.copysign(max(min_turn, abs(heading_gain*heading_error)), heading_error)
        return 0., clamp(omega, limit), False, radial_error
    return speed, clamp(omega, limit), False, radial_error
