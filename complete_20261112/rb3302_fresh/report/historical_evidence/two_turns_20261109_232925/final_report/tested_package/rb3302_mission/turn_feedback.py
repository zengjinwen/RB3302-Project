"""Turn-only measured-rate PI trim; the successful straight controller is untouched."""
import math
from .grid import clamp
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

    def update(self, requested, measured, dt):
        if requested == 0. or measured is None:
            self.integral = self.direction = 0.
            return requested
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
        if abs(correction) <= limit or correction*error < 0.:
            self.integral = candidate
        return requested+clamp(gain*error+self.integral,limit)


class FeedbackMission(ExperimentalObstacleMission):
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
