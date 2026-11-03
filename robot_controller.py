import numpy as np
from robot_state import RobotState

class RobotController:
    def __init__(self, robot):
        self.robot = robot
        self.speed_constant = 10
        self.turn_duration = 1.0
        
        # Initialisation des moteurs du robot
        self.motors = []
        motors_names = ["Motor2", "Motor1", "Motor4", "Motor3"]
        for name in motors_names:
            motor = self.robot.getDevice(name)
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            self.motors.append(motor)

    def set_motors_velocities(self, left_speed, right_speed):
        """Applique et limite les vitesses sur les moteurs."""
        left_speed = np.clip(left_speed, -self.speed_constant, self.speed_constant)
        right_speed = np.clip(right_speed, -self.speed_constant, self.speed_constant)

        self.motors[0].setVelocity(left_speed)
        self.motors[1].setVelocity(right_speed)
        self.motors[2].setVelocity(left_speed)
        self.motors[3].setVelocity(right_speed)

    def apply_pid(self, error, Kp=0.035):
        """Calcule la correction PID simple pour le suivi de ligne."""
        correction = error * Kp
        base_speed = self.speed_constant
        left_speed = base_speed + correction
        right_speed = base_speed - correction
        return left_speed, right_speed

    def turn_90_degrees(self, left: bool, action_start_time):
        """Gère la rotation de 90 degrés à gauche ou à droite."""
        turn_speed = self.speed_constant * 0.6
        if left:
            left_speed = -turn_speed
            right_speed = turn_speed
        else:
            left_speed = turn_speed
            right_speed = -turn_speed
            
        self.set_motors_velocities(left_speed, right_speed)

        # Vérifie si la durée de rotation est écoulée
        if self.robot.getTime() - action_start_time > self.turn_duration:
            print("Rotation de 90° terminée. Reprise du suivi de ligne.")
            return RobotState.FOLLOWING
        return RobotState.TURNING_90 if left else RobotState.TURNING_MINUS_90
