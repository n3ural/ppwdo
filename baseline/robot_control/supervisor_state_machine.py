from baseline.robot_control.control_state import ControlState
from baseline.utils import linalg2_util as linalg
from baseline.sim_exceptions.goal_reached_exception import GoalReachedException

# event parameters
D_STOP = 0.05  # meters from goal
D_CAUTION = 0.15  # meters from obstacle
D_DANGER = 0.04  # meters from obstacle

# progress margin
PROGRESS_EPSILON = 0.05


class SupervisorStateMachine:
    """
    The SupervisorStateMachine class manages the state transitions and conditions for a robot supervisor.
    It evaluates environmental conditions based on sensor readings, determines the robot's behavior in response to
    these conditions, and transitions between states to ensure the robot's safe and effective operation.
    This class acts as a decision-making mechanism, deciding which control strategy to employ based on current circumstances.
    """

    def __init__(self, supervisor):
        self.supervisor = supervisor

        # initialize state
        self.transition_to_state_go_to_goal()

        # progress tracking
        self.best_distance_to_goal = float("inf")

    def update_state(self):
        if self.current_state == ControlState.GO_TO_GOAL:
            self.execute_state_go_to_goal()
        elif self.current_state == ControlState.AVOID_OBSTACLES:
            self.execute_state_avoid_obstacles()
        elif self.current_state == ControlState.SLIDE_LEFT:
            self.execute_state_slide_left()
        elif self.current_state == ControlState.SLIDE_RIGHT:
            self.execute_state_slide_right()
        else:
            raise Exception("undefined supervisor state or behavior")

    # === STATE PROCEDURES ===
    def execute_state_go_to_goal(self):
        if self.condition_at_goal():
            self.transition_to_state_at_goal()
        elif self.condition_danger():
            self.transition_to_state_avoid_obstacles()
        elif self.condition_at_obstacle():
            sl = self.condition_slide_left()
            sr = self.condition_slide_right()
            if sl and not sr:
                self.transition_to_state_slide_left()
            elif sr and not sl:
                self.transition_to_state_slide_right()
            # elif sl and sr: raise Exception( "cannot determine slide direction" )

    def execute_state_avoid_obstacles(self):
        if self.condition_at_goal():
            self.transition_to_state_at_goal()
        elif not self.condition_danger():
            sl = self.condition_slide_left()
            sr = self.condition_slide_right()
            if sl and not sr:
                self.transition_to_state_slide_left()
            elif sr and not sl:
                self.transition_to_state_slide_right()
            elif not sr and not sl:
                self.transition_to_state_go_to_goal()
            # else: raise Exception( "cannot determine slide direction" )

    def execute_state_slide_left(self):
        if self.condition_at_goal():
            self.transition_to_state_at_goal()
        elif self.condition_danger():
            self.transition_to_state_avoid_obstacles()
        elif self.condition_progress_made() and not self.condition_slide_left():
            self.transition_to_state_go_to_goal()

    def execute_state_slide_right(self):
        if self.condition_at_goal():
            self.transistion_to_state_at_goal()
        elif self.condition_danger():
            self.transition_to_state_avoid_obstacles()
        elif self.condition_progress_made() and not self.condition_slide_right():
            self.transition_to_state_go_to_goal()

    # def execute_state_gtg_and_ao( self ):
    #   if self.condition_at_goal():        self.transition_to_state_at_goal()
    #   elif self.condition_danger():       self.transition_to_state_avoid_obstacles()
    #   elif self.condition_no_obstacle():  self.transition_to_state_go_to_goal()

    # === STATE TRANSITIONS ===
    def transition_to_state_at_goal(self):
        self.current_state = ControlState.AT_GOAL
        raise GoalReachedException()

    def transition_to_state_avoid_obstacles(self):
        self.current_state = ControlState.AVOID_OBSTACLES
        self.supervisor.current_controller = self.supervisor.avoid_obstacles_controller

    def transition_to_state_go_to_goal(self):
        self.current_state = ControlState.GO_TO_GOAL
        self.supervisor.current_controller = self.supervisor.go_to_goal_controller

    def transition_to_state_slide_left(self):
        self.current_state = ControlState.SLIDE_LEFT
        self._update_best_distance_to_goal()
        self.supervisor.current_controller = self.supervisor.follow_wall_controller

    def transition_to_state_slide_right(self):
        """
        Transition the robot's behavior to a "slide right" state.

        In scenarios where the robot encounters obstacles on its left or needs to maneuver around an obstacle on its left side,
        this method switches the robot's behavior to a state where it will attempt to slide or move to the right.
        The robot's goal distance is updated, and the controller is adjusted to guide the robot along the wall
        on its right-hand side.
        """
        self.current_state = ControlState.SLIDE_RIGHT
        self._update_best_distance_to_goal()
        self.supervisor.current_controller = self.supervisor.follow_wall_controller

    def transition_to_state_gtg_and_ao(self):
        """
        Transition the robot's behavior to a combined "go-to-goal and avoid-obstacles" strategy.

        This state is activated when the robot needs to move towards its target goal while simultaneously
        being vigilant about and avoiding any obstacles in its path. The corresponding controller
        manages these dual responsibilities, ensuring the robot progresses toward the goal without collisions.
        """
        self.current_state = ControlState.GTG_AND_AO
        self.supervisor.current_controller = self.supervisor.gtg_and_ao_controller

    # === CONDITIONS ===
    def condition_at_goal(self):
        """
        Determine if the robot has reached or is extremely close to its designated goal.

        By assessing the robot's current estimated position and comparing it with its target goal position,
        this method evaluates if the robot is within a certain proximity threshold of the goal. If the robot
        is close enough, it is considered to have effectively reached its goal.

        Returns
        -------
        bool
            True if the robot is at or near its goal, False otherwise.
        """
        return (
            linalg.distance(
                self.supervisor.estimated_pose.vposition(), self.supervisor.goal
            )
            < D_STOP
        )

    def condition_at_obstacle(self):
        """
        Assess if the robot is approaching an obstacle that requires caution.

        Based on sensor readings, particularly from proximity sensors, this method evaluates if there's
        an obstacle within a certain 'cautionary' distance in the robot's forward path. If an obstacle is detected
        within this range, it implies the robot needs to be cautious and potentially change its current behavior
        to avoid a collision.

        Returns
        -------
        bool
            True if an obstacle is within the cautionary range, False otherwise.
        """
        for d in self._forward_sensor_distances():
            if d < D_CAUTION:
                return True
        return False

    def condition_danger(self):
        """
        Evaluate if the robot is in imminent danger of colliding with an obstacle.

        Using proximity sensor data, this method checks if there's an obstacle within a critical 'danger' distance
        directly ahead of the robot. An obstacle within this range indicates an immediate threat, and the robot
        must take urgent action to avoid a collision.

        Returns
        -------
        bool
            True if the robot is in immediate danger of a collision, False otherwise.
        """
        for d in self._forward_sensor_distances():
            if d < D_DANGER:
                return True
        return False

    def condition_no_obstacle(self):
        """
        Check if the robot's forward path is clear of any obstacles within a cautionary range.

        Proximity sensors provide data on nearby objects. This method analyzes this data to determine
        if the robot's immediate forward path is free from obstacles that fall within a specific 'cautionary' range.
        If the path is clear, the robot can proceed without the need for evasive maneuvers.

        Returns
        -------
        bool
            True if the forward path is clear of obstacles in the cautionary range, False otherwise.
        """
        for d in self._forward_sensor_distances():
            if d < D_CAUTION:
                return False
        return True

    def condition_progress_made(self):
        """
        Determine if the robot is making effective progress towards its target goal.

        By comparing the robot's current estimated distance to its goal with a previously recorded 'best' distance,
        this method assesses if the robot is moving closer to its destination. If the current distance is less
        than the previous best, the robot is deemed to be making progress.

        Returns
        -------
        bool
            True if the robot has moved closer to its goal since the last assessment, False otherwise.
        """
        return self._distance_to_goal() < self.best_distance_to_goal - PROGRESS_EPSILON

    def condition_slide_left(self):
        """
        Decide if the robot should adopt a "slide left" behavior based on its heading vectors.

        The robot's heading vectors, including those for going to its goal, avoiding obstacles, and following walls,
        provide directional cues. This method evaluates these vectors to decide if the robot should slide or move to
        its left, especially in situations where it needs to navigate around obstacles on its right side.

        Returns
        -------
        bool
            True if the heading vectors suggest a "slide left" maneuver, False otherwise.
        """
        heading_gtg = self.supervisor.go_to_goal_controller.gtg_heading_vector
        heading_ao = self.supervisor.avoid_obstacles_controller.ao_heading_vector
        heading_fwl = self.supervisor.follow_wall_controller.l_fw_heading_vector

        ao_cross_fwl = linalg.cross(heading_ao, heading_fwl)
        fwl_cross_gtg = linalg.cross(heading_fwl, heading_gtg)
        ao_cross_gtg = linalg.cross(heading_ao, heading_gtg)

        return (ao_cross_gtg > 0.0 and ao_cross_fwl > 0.0 and fwl_cross_gtg > 0.0) or (
            ao_cross_gtg <= 0.0 and ao_cross_fwl <= 0.0 and fwl_cross_gtg <= 0.0
        )

    def condition_slide_right(self):
        heading_gtg = self.supervisor.go_to_goal_controller.gtg_heading_vector
        heading_ao = self.supervisor.avoid_obstacles_controller.ao_heading_vector
        heading_fwr = self.supervisor.follow_wall_controller.r_fw_heading_vector

        ao_cross_fwr = linalg.cross(heading_ao, heading_fwr)
        fwr_cross_gtg = linalg.cross(heading_fwr, heading_gtg)
        ao_cross_gtg = linalg.cross(heading_ao, heading_gtg)

        return (ao_cross_gtg > 0.0 and ao_cross_fwr > 0.0 and fwr_cross_gtg > 0.0) or (
            ao_cross_gtg <= 0.0 and ao_cross_fwr <= 0.0 and fwr_cross_gtg <= 0.0
        )

    # === helper methods ===
    def _forward_sensor_distances(self):
        return self.supervisor.proximity_sensor_distances[1:7]

    def _distance_to_goal(self):
        return linalg.distance(
            self.supervisor.estimated_pose.vposition(), self.supervisor.goal
        )

    def _update_best_distance_to_goal(self):
        self.best_distance_to_goal = min(
            self.best_distance_to_goal, self._distance_to_goal()
        )

    # === FOR DEBUGGING ===
    def _print_debug_info(self):
        print("\n ======== \n")
        print(
            "STATE: "
            + str(
                [
                    "At Goal",
                    "Go to Goal",
                    "Avoid Obstacles",
                    "Blended",
                    "Slide Left",
                    "Slide Right",
                ][self.current_state]
                + "\n"
            )
        )
        print("CONDITIONS:")
        print("At Obstacle: " + str(self.condition_at_obstacle()))
        print("Danger: " + str(self.condition_danger()))
        print("No Obstacle: " + str(self.condition_no_obstacle()))
        print(
            "Progress Made: "
            + str(self.condition_progress_made())
            + " ( Best Dist: "
            + str(round(self.best_distance_to_goal, 3))
            + ", Current Dist: "
            + str(round(self._distance_to_goal(), 3))
            + " )"
        )
        print("Slide Left: " + str(self.condition_slide_left()))
        print("Slide Right: " + str(self.condition_slide_right()))
