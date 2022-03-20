#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('v_total_error', 0.0)
        self.vars.create_var('v_previous_error', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            """
            공성식 노트
            Longitudinal Control은 각 input을 통해 target position/velocity에 이르기 위한 throttle, brake 값을 설정하는 것이다.
            현재의 경우 brake output은 간단한 모델 처리를 위해 생략하도록 하고, throttle의 경우 engine torque, engine angular speed를 구하면, 현재 모델의 실험값 데이터 테이블로부터 적절한 throttle angle을 도출할 수 있는 구조이다.
            그렇다면 어떻게 T_eng와 w_eng 을 구할 수 있을까:

            - 아이디어1
            다음의 가정 하에 feedfwd-feedback CLTF를 사용할 수 있다.
                * no brake
                * no torque loss (3+ gear locked)
                * tire slip is negligible
            ??? : vehicle kinmatics/dynamics에 대한 정보가 없는데 어떻게 계산하지..?

            - 아이디어2 [채택]
            그냥 throttle 자체에 대한 PID 제어를 할 수 있다. 어짜피 GT를 현재 알고 있으니..!
            """
            throttle_output = 0
            brake_output    = 0

            # 적합한 PID Gain들을 찾아 튜닝시켜야 하는게 문제...
            KP_throttle = 2
            KI_throttle = 1
            KD_throttle = 0.5

            dt = t - self.vars.t_previous       # 현재시간 - 이전시간 = 시간차 (미적분 계산의 근본을 항상 세운다)
            v_error = v_desired - self.vars.v_previous                  # P항과 곱할 값: 현재 error 값
            v_error_integral = self.vars.v_total_error + v_error * dt      # I항과 곱할 값: 현재 error의 전체 적분값 = total error를 쌓아서 저장
            v_error_differential = (v_error - self.vars.v_previous_error)/dt    # D항과 곱할 값: error의 순간변화율 = error차를 dt로 나눈 값

            KP = KP_throttle * v_error
            KI = KI_throttle * v_error_integral
            KD = KD_throttle * v_error_differential

            throttle_output = KP + KI + KD

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            """
            LATERAL CONTROL은 변수를 받은 후 steering angle에 대한 값을 내놓는 것이다.
            기본적인 방식부터 고차원적인 알고리즘까지 다양한데 : PurePursuit, Stanley, ModelPredictiveControl 등이 있다.
            단, 이들은 geometric path tracking controller의 분류이므로, vehicle kinematics와 ref path에 대한 정보만 사용한다.
            이에 따라 non-slip 상황이라는 가정이 필요하다.

            가장 기본적인 Lateral Control인 Pure Pursuit을 작성하여보자.
            Pure Pursuit은,
            ld = 후륜 초점으로 부터 잡은 target 포인트와의 거리 + K_ld와의 실제 속도보정 (lookahead_distance)
                ** ld의 속도보정
                만약 ld를 단순한 거리로 잡으면, 계산 중 curvature = 2*CrossStackError/(ld**2) 가 되어 ld값에 따라 path plan의 curvature가 mathematically unstable하게 된다.
                따라서 ld값의 최소 보정을 해주어야 한다. 또한 ld의 값을 속도에 proportional하게 잡아주어 속도에 따른 반응을 얻을 수 있게 한다.
            L = 자동차 모델 calibration - 차체 길이
            alpha = 후륜 초점 속도 벡터와 ld 벡터 사이의 각도
            를 사용하여 steering angle (delta) 를 계산하는 방식이다.

            <Pure Pursuit>
            다시 정리하자면, ld는 원래 target point와 후륜 점과의 거리로 정의되는 것이 맞다. 그러나 이렇게 하면 속도값이 고려되지 않고, 수학적 안정성이 보장되지 않는 문제가 발생한다.
            이를 보정하기 위해 우선 ld의 개념 자체를 K_ld(proportional gain)와 v의 곱으로 재정의하여 사용한다. (ld의 의미 자체는 '목적지까지 떨어진 정도' 이므로)
            이후, alpha를 구하기 위해서 물리적 진짜 정의 ld를 다시 계산 한다. 이 두 점 사이의 각은 절대 좌표계에서 우리의 path의 각이다.
            이 path의 각에, 현재 우리 차체가 틀어진 각(우리의 상대좌표계가 틀어진 각)만큼을 보정해주어야 우리의 목적인 alpha (=현재 우리 각도와 목적 path각의 차이)
            에 도달할 수 있게 된다.
            alpha까지 구하였다면, vehicla kinematics와 dynamics에 증명된 대로 delta(=steering angle)를 구해주면 된다.
            """
            #1 model param
            K_ld = 0.8
            min_ld = 10
            L = 3

            #2 후륜 초점 구하기
            x_rear = x - (L*np.cos(yaw))/2
            y_rear = y - (L*np.sin(yaw))/2
            
            #3 proportional lookahead distance
            ld = max(min_ld, K_ld*v)
            
            #4 alpha를 구하기 위해: real ld
            for point in waypoints:
                distance = np.sqrt((point[0]-x_rear)**2 + (point[1] - y_rear)**2) # 우선 후륜 초점과 trajectory 상의 점들 사이의 거리를 구한다.
                if distance > ld: # lookahead distance보다 먼 거리에 있는 point = CrossStackError로서 기능할 수 있다.
                    tmp = point
                    break

            else:
                tmp = waypoints[0] # 첫 시도 예외처리

            alpha = np.arctan2((tmp[1]-y_rear),(tmp[0]-x_rear)) - yaw # pi ~ -pi 범위를 사용하므로 atan2를 사용함! (-pi/2~pi/2 범위일 경우 그냥 arctan이지만)
            # yaw는 절대 좌표에서 차체가 기울어진 각이다. 

            # Change the steer output with the lateral controller. 
            steer_output    = np.arctan2(2*L*np.sin(alpha),ld)


            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t
        self.vars.v_previous_error = v_error
        self.vars.v_total_error = v_error_integral
