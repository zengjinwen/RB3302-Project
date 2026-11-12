"""Turn-only measured-rate PI trim; the successful straight controller is untouched."""
import math
from .grid import clamp, wrap
from .experiment_obstacles import ExperimentalObstacleMission
from .node import main as mission_main


class RateTrim:
    """Unity feedforward plus PI; no borrowed motor calibration.

    Design starting point: a full rate error adds 25% proportional command;
    the integral adds the same amount in one second. Total correction is
    bounded at 50% of the requested command. These are tunable design choices,
    not measured plant gains. Stop/reversal/stale feedback reset the integral.
    """
    def __init__(self):
        self.integral = 0.
        self.direction = 0.
        self.command_limit = 2.  # Same SI angular-command cap as the active base driver.

    def update(self, requested, measured, dt):
        if requested == 0. or measured is None:
            self.integral = self.direction = 0.
            return requested
        requested = clamp(requested,self.command_limit)
        direction = math.copysign(1.,requested)
        if direction != self.direction:
            self.integral = 0.
        self.direction = direction
        error = requested-measured
        gain, integral_time = .25,1.
        limit = .5*abs(requested)
        self.integral = clamp(self.integral,limit)
        candidate = clamp(self.integral+gain/integral_time*error*dt,limit)
        correction = gain*error+candidate
        # Conditional integration: do not accumulate further into saturation.
        if ((abs(correction) <= limit and abs(requested+correction) <= self.command_limit)
                or correction*error < 0.):
            self.integral = candidate
        return clamp(requested+clamp(gain*error+self.integral,limit),self.command_limit)


class FeedbackMission(ExperimentalObstacleMission):
    transition_delay = 0.
    goal_position_tolerance = .05
    goal_turn_speed = .5
    goal_min_speed = 0.

    def __init__(self):
        self.rate_trim = RateTrim()
        self.rate_active = False
        self.rate_time = None
        self.rate_log = 0.
        super().__init__()


    def command(self, linear=0., angular=0.):
        if self.rate_active:
            now = self.now()
            dt = 0. if self.rate_time is None else min(.2,max(0.,now-self.rate_time))
            self.rate_time = now
            measured = None
            if (self.odom is not None and self.odom.child_frame_id == 'base_link'
                    and -.1 <= now-self.stamp(self.odom) < .5):
                rate = self.odom.twist.twist.angular.z
                if math.isfinite(rate):
                    measured = rate
            requested = angular
            angular = self.rate_trim.update(requested,measured,dt)
            if requested and now >= self.rate_log:
                actual = 'stale' if measured is None else f'{measured:+.3f}'
                self.get_logger().info(f'TURN RATE requested={requested:+.3f} measured={actual} command={angular:+.3f}')
                self.rate_log = now+1.
        super().command(linear,angular)

    def align_goal(self, goal):
        """Rotate toward the goal if needed, then regulated pure pursuit to the goal tolerance."""
        target = self.points[goal]
        capture_distance = getattr(self,'goal_capture_distance',self.goal_position_tolerance)
        turning = False
        next_log = 0.
        self.rate_active = True
        self.rate_trim = RateTrim()
        self.rate_time = None
        try:
            while True:
                self.pause()
                sample = self.straight_sample()
                if sample is None:
                    self.command()
                    continue
                x,y,heading,_ = sample
                dx,dy = target[0]-x,target[1]-y
                distance = math.hypot(dx,dy)
                if distance <= capture_distance:
                    self.command()
                    self.get_logger().info(f'GOAL ALIGN {goal}: dx={dx:+.3f} dy={dy:+.3f}m; within {capture_distance*100:.0f}cm')
                    return
                bearing = wrap(math.atan2(dy,dx)-heading)
                # Separate enter/leave angles prevent rotate/drive chatter.
                if abs(bearing) > math.pi/4.:
                    turning = True
                elif abs(bearing) < math.radians(10.):
                    turning = False
                self.goal_changed_heading = True
                if turning:
                    linear,angular = 0.,clamp(1.5*bearing,self.goal_turn_speed)
                else:
                    curvature = 2.*math.sin(bearing)/distance
                    linear = min(self.p['approach_speed'],max(self.goal_min_speed,.8*distance))
                    if abs(curvature) > 1e-9:
                        linear = min(linear,self.goal_turn_speed/abs(curvature))
                    angular = linear*curvature
                self.command(linear,angular)
                if self.now() >= next_log:
                    self.get_logger().info(f'GOAL RPP {goal}: remaining={distance:.3f}m rotate={turning} v={linear:.3f} w={angular:+.3f}')
                    next_log = self.now()+1.
        finally:
            self.command()
            self.rate_active = False
            self.rate_trim = RateTrim()

    def face_route(self, heading):
        if not getattr(self,'goal_changed_heading',False):
            return super().face_route(heading)
        self.face_heading(heading)

    def face_heading(self, heading):
        """Close the heading loop on the absolute route direction."""
        self.rate_active = True
        self.rate_trim = RateTrim()
        self.rate_time = None
        try:
            while True:
                self.pause()
                sample = self.straight_sample()
                if sample is None:
                    self.command()
                    continue
                error = wrap(heading-sample[2])
                if abs(error) <= math.radians(3.):
                    break
                self.command(0.,clamp(1.5*error,self.goal_turn_speed))
        finally:
            self.command()
            self.rate_active = False
            self.rate_trim = RateTrim()
        self.goal_changed_heading = False
        self.route_heading = heading

    def turn_relative(self, angle):
        self.rate_trim = RateTrim()
        self.rate_time = None
        self.rate_active = True
        try:
            super().turn_relative(angle)
        finally:
            self.rate_active = False
            self.rate_trim = RateTrim()


def main(args=None):
    mission_main(args,mission_class=FeedbackMission)
