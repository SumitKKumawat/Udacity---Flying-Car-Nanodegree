import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


TARGET_HEIGHT = 3.0     # [meter]
BOX_SIZE = 10.0         # [meter]
TOLERANCE_XY = 0.5      # [meter]
TOLERANCE_Z = 0.1       # [meter]
TOLERANCE_VEL = 0.25    # [meter/second]


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = [0.0, 0.0, 0.0]
        self.all_waypoints = self.calculate_box()
        self.in_mission = True
        self.check_state = {}
        self.is_stable = False

        # Initial state
        self.flight_state = States.MANUAL

        # Callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
            if self._arrived_to_point(self.target_position):
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
            if self._arrived_to_point(self.target_position):
                if self.target_position == self.all_waypoints[-1]:
                    self.landing_transition()
                else:
                    self.waypoint_transition()

        elif self.flight_state == States.LANDING:
            if self._on_ground():
                self.disarming_transition()

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        self.is_stable = vector_equal(self.local_velocity, [0.0, 0.0, 0.0], TOLERANCE_VEL)

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.armed:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.DISARMING:
                self.manual_transition()
        else:
            if self.flight_state == States.ARMING:
                self.takeoff_transition()

    def calculate_box(self):
        """
        1. Return waypoints to fly a box
        """
        waypoints = \
        [
            [BOX_SIZE, 0.0,      TARGET_HEIGHT],
            [BOX_SIZE, BOX_SIZE, TARGET_HEIGHT],
            [0.0,      BOX_SIZE, TARGET_HEIGHT],
            [0.0,      0.0,      TARGET_HEIGHT]
        ]

        return waypoints

    def arming_transition(self):
        """
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")

        self.take_control()
        self.arm()
        self.set_home_position(*self.global_position)
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")

        self.target_position = [0.0, 0.0, TARGET_HEIGHT]
        self.takeoff(TARGET_HEIGHT)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        current_target_idx = self.all_waypoints.index(self.target_position)
        next_target_idx = (current_target_idx + 1) % len(self.all_waypoints)
        self.target_position = self.all_waypoints[next_target_idx]

        self.cmd_position(*self.target_position, 0.0)
        self.flight_state = States.WAYPOINT

        print("waypoint transition to {}".format(self.target_position))

    def landing_transition(self):
        """
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")

        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")

        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()

    def _arrived_to_point(self, point):
        return vector_equal(ned_to_nea(self.local_position), point, TOLERANCE_XY) and self.is_stable

    def _on_ground(self):
        return vector_equal(self.local_position[2], 0.0, TOLERANCE_Z)

def vector_equal(p1, p2, tolerance):
    """Determine if 2 vectors are close enough, given some tolerance.
    Args:
        p1 (list): first vector
        p2 (list): second vector
        tolerance (float): tolerance to compare the norm of the difference
    Returns:
        bool: True if the norm is smaller than the tolerance; False otherwise
    """
    return np.linalg.norm(np.array(p1) - np.array(p2)) < tolerance

def ned_to_nea(x):
    """Convert from North-East-Down to North-East-Altitude.
    Args:
        x (list): input position vector in North-East-Down format
    Returns:
        list: output position vector in North-East-Altitude format
    """
    return [x[0], x[1], -x[2]]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
