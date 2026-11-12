"""Keep the supplied serial/feedback interface; use documented SI velocity command."""
import math
import rclpy
from waverover_base.waverover_driver import WaveRoverDriver

class Driver(WaveRoverDriver):

    def _send_velocity(self, linear, angular):
        if not math.isfinite(linear) or not math.isfinite(angular):
            linear, angular = (0.0, 0.0)
        self.transport.write_json({'T': 13, 'X': round(linear, 4), 'Z': round(angular, 4)})
if __name__ == '__main__':
    rclpy.init()
    node = None
    try:
        node = Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
