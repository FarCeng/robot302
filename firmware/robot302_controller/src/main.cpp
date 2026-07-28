  #include <Arduino.h>
  #include <Wire.h>
  #include <Adafruit_MPU6050.h>
  #include <Adafruit_Sensor.h>
  #include <micro_ros_arduino.h>
  #include <rcl/rcl.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
  #include <geometry_msgs/msg/twist.h>
  #include <geometry_msgs/msg/vector3.h>
  #include <sensor_msgs/msg/imu.h>
  #include <time.h>
  #include <math.h>

  // ======================================================
  // KONFIGURASI PIN
  // ======================================================
  const int L_PIN_PWM = 16;
  const int L_PIN_DIR = 19;

  const int R_PIN_PWM = 23;
  const int R_PIN_DIR = 18;

  const int L_ENC_A = 25;
  const int L_ENC_B = 26;

  const int R_ENC_A = 13;
  const int R_ENC_B = 27;

  // ======================================================
  // MODE TEST
  // ======================================================
  const int TEST_MODE = 0;
  const int TEST_PWM = 220;
  const int PWM_RAMP_STEP = 5;

  // ======================================================
  // HASIL KALIBRASI IMU
  // ======================================================
  const float GYRO_X_OFFSET = -0.043302f;
  const float GYRO_Y_OFFSET = -0.045375f;
  const float GYRO_Z_OFFSET = 0.004033f;

  const float ACCEL_X_OFFSET = 0.054103f;
  const float ACCEL_Y_OFFSET = 0.010481f;
  const float ACCEL_Z_OFFSET = -0.760960f;

  // ======================================================
  // HEADING HOLD (YAW TRACKING)
  // ======================================================
  float current_yaw = 0.0f;
  float target_yaw = 0.0f;
  bool is_yaw_locked = false;
  float cmd_angular_z = 0.0f;
  bool prev_straight_cmd = false;
  

  // ======================================================
  // PARAMETER ENCODER & KINEMATIKA
  // ======================================================
  const int L_PPR = 102;
  const int R_PPR = 102;
  const float Ts = 0.05f;

  const int LEFT_FORWARD_DIR  = HIGH;
  const int RIGHT_FORWARD_DIR = HIGH;

  // ======================================================
  // MODEL MATEMATIS RPM -> PWM (SANGAT STABIL)
  // ======================================================
  static inline float rpmToPwmLeft(float rpm)
  {
    rpm = fabsf(rpm);
    float pwm = (0.01386043f * rpm * rpm) + (0.29900265f * rpm) + 47.82892292f;
    return pwm;
  }

  static inline float rpmToPwmRight(float rpm)
  {
    rpm = fabsf(rpm);
    float pwm = (0.01661597f * rpm * rpm) + (0.14011050f * rpm) + 48.23452690f;
    return pwm;
  }

  static inline int clampPwm(float pwm)
  {
    int out = (int)roundf(pwm);
    if (out < 0) out = 0;
    if (out > 255) out = 255;
    return out;
  }

  static inline int reverseDir(int dir)
  {
    return (dir == HIGH) ? LOW : HIGH;
  }

  // ======================================================
  // IMU & ENCODER OBJECTS
  // ======================================================
  Adafruit_MPU6050 mpu;
  volatile long left_count  = 0;
  volatile long right_count = 0;

  void IRAM_ATTR leftISR()
  {
    (digitalRead(L_ENC_B) == HIGH) ? left_count-- : left_count++;
  }
  void IRAM_ATTR rightISR()
  {
    (digitalRead(R_ENC_B) == HIGH) ? right_count++ : right_count--;
  }

  // ======================================================
  // ROS OBJECTS & VARIABLES
  // ======================================================
  rcl_subscription_t sub_cmd_vel;
  rcl_publisher_t pub_left;
  rcl_publisher_t pub_right;
  rcl_publisher_t pub_imu;

  geometry_msgs__msg__Twist msg_cmd_vel;
  geometry_msgs__msg__Vector3 msg_L;
  geometry_msgs__msg__Vector3 msg_R;
  sensor_msgs__msg__Imu msg_imu;

  rclc_executor_t executor;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  float target_rpm_L = 0.0f;
  float target_rpm_R = 0.0f;
  float smoothed_target_L = 0.0f;
  float smoothed_target_R = 0.0f;

  unsigned long prev_ms = 0;
  long last_L = 0;
  long last_R = 0;
  float filtered_rpm_L = 0.0f;
  float filtered_rpm_R = 0.0f;
  int current_pwm = 0;

  // ======================================================
  // CMD_VEL CALLBACK (SMART CORNERING)
  // ======================================================
  void cmdVelCb(const void * msgin)
  {
    const geometry_msgs__msg__Twist * data = (const geometry_msgs__msg__Twist *)msgin;
    cmd_angular_z = data->angular.z; 

    const float wheelbase = 0.19f;
    const float wheel_radius = 0.0290f;
    const float MAX_RPM = 357.0f;

    float raw_vx = data->linear.x;
    float raw_w  = data->angular.z;

    float smooth_w = raw_w * 0.7f; 
    float turn_penalty = fabsf(smooth_w) * 0.15f; 
    if (turn_penalty > 0.3f) turn_penalty = 0.3f; 
    
    float smooth_vx = raw_vx * (1.0f - turn_penalty);

    float v_l = smooth_vx - (smooth_w * (wheelbase / 2.0f));
    float v_r = smooth_vx + (smooth_w * (wheelbase / 2.0f));

    target_rpm_L = -(v_l * 60.0f) / (2.0f * PI * wheel_radius);
    target_rpm_R = -(v_r * 60.0f) / (2.0f * PI * wheel_radius);

    if (target_rpm_L > MAX_RPM) target_rpm_L = MAX_RPM;
    if (target_rpm_L < -MAX_RPM) target_rpm_L = -MAX_RPM;
    if (target_rpm_R > MAX_RPM) target_rpm_R = MAX_RPM;
    if (target_rpm_R < -MAX_RPM) target_rpm_R = -MAX_RPM;
  }

  // ======================================================
  // SETUP
  // ======================================================
  void setup()
  {
    set_microros_transports();
    Wire.begin(21, 22);

    if (mpu.begin()) {
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    }

    pinMode(L_PIN_DIR, OUTPUT);
    pinMode(L_PIN_PWM, OUTPUT);
    pinMode(R_PIN_DIR, OUTPUT);
    pinMode(R_PIN_PWM, OUTPUT);

    pinMode(L_ENC_A, INPUT_PULLUP);
    pinMode(L_ENC_B, INPUT_PULLUP);
    pinMode(R_ENC_A, INPUT_PULLUP);
    pinMode(R_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, RISING);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_robot_node", "", &support);

    rclc_publisher_init_best_effort(&pub_left, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "left_encoder");
    rclc_publisher_init_best_effort(&pub_right, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "right_encoder");
    rclc_publisher_init_best_effort(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw");
    rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &cmdVelCb, ON_NEW_DATA);

    msg_imu.header.frame_id.data = (char*)"imu_link";
    msg_imu.header.frame_id.size = strlen(msg_imu.header.frame_id.data);
    msg_imu.header.frame_id.capacity = msg_imu.header.frame_id.size + 1;

    digitalWrite(L_PIN_DIR, LEFT_FORWARD_DIR);
    digitalWrite(R_PIN_DIR, RIGHT_FORWARD_DIR);
    analogWrite(L_PIN_PWM, 0);
    analogWrite(R_PIN_PWM, 0);
  }

  // ======================================================
  // LOOP
  // ======================================================
  void loop()
  {
    unsigned long now = millis();

    if (now - prev_ms >= 50) {

      // 1. BACA IMU & INTEGRASI YAW
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      float accel_x_clean = a.acceleration.x - ACCEL_X_OFFSET;
      float accel_y_clean = a.acceleration.y - ACCEL_Y_OFFSET;
      float accel_z_clean = a.acceleration.z - ACCEL_Z_OFFSET;

      float gyro_x_clean = g.gyro.x - GYRO_X_OFFSET;
      float gyro_y_clean = g.gyro.y - GYRO_Y_OFFSET;
      float gyro_z_clean = g.gyro.z - GYRO_Z_OFFSET;

      current_yaw += gyro_z_clean * Ts;

      // Selalu jaga yaw pada rentang [-PI, PI]
      current_yaw = atan2f(
          sinf(current_yaw),
          cosf(current_yaw)
      );

      // 2. SMOOTH TARGET RPM (AKSELERASI HALUS)
      const float RAMP_STEP_RPM = 8.0f;
      if (smoothed_target_L < target_rpm_L) {
        smoothed_target_L = min(smoothed_target_L + RAMP_STEP_RPM, target_rpm_L);
      } else if (smoothed_target_L > target_rpm_L) {
        smoothed_target_L = max(smoothed_target_L - RAMP_STEP_RPM, target_rpm_L);
      }

      if (smoothed_target_R < target_rpm_R) {
        smoothed_target_R = min(smoothed_target_R + RAMP_STEP_RPM, target_rpm_R);
      } else if (smoothed_target_R > target_rpm_R) {
        smoothed_target_R = max(smoothed_target_R - RAMP_STEP_RPM, target_rpm_R);
      }

      // 3. BACA & FILTER ENCODER
      long cur_L = left_count;
      long cur_R = right_count;

      float raw_rpm_L = -((float)(cur_L - last_L) / (float)L_PPR / Ts) * 60.0f;
      float raw_rpm_R = -((float)(cur_R - last_R) / (float)R_PPR / Ts) * 60.0f;

      filtered_rpm_L = (0.8f * filtered_rpm_L) + (0.2f * raw_rpm_L);
      filtered_rpm_R = (0.8f * filtered_rpm_R) + (0.2f * raw_rpm_R);

      last_L = cur_L;
      last_R = cur_R;

      // 4. MOTOR DRIVE LOGIC
      int applied_pwm_L = 0;
      int applied_pwm_R = 0;

      if (TEST_MODE == 0) {
        
        // Jika target mendekati 0, hentikan motor
        if (fabsf(smoothed_target_L) < 1.0f && fabsf(smoothed_target_R) < 1.0f) {
          applied_pwm_L = 0;
          applied_pwm_R = 0;
        } 
        else {
          // A. FEEDFORWARD MODEL MATEMATIS
          applied_pwm_L = clampPwm(rpmToPwmLeft(fabsf(smoothed_target_L)));
          applied_pwm_R = clampPwm(rpmToPwmRight(fabsf(smoothed_target_R)));

          // B. RIGHT WHEEL TRIM (Penyeimbang kekuatan motor bawaan)
          applied_pwm_R = (int)((float)applied_pwm_R * 1.00f);

          // C. SINKRONISASI RODA (Error Correction 1)
          float sync_error = fabsf(filtered_rpm_L) - fabsf(filtered_rpm_R);
          const float Ksync = 1.2f;
          applied_pwm_L -= (int)(sync_error * Ksync);
          applied_pwm_R += (int)(sync_error * Ksync);

          // D. IMU HEADING HOLD
          bool moving_cmd =
          (
              fabsf(smoothed_target_L) > 5.0f ||
              fabsf(smoothed_target_R) > 5.0f
          );

          bool straight_cmd =
          (
              fabsf(cmd_angular_z) < 0.05f
          );

          // Robot berhenti atau sedang belok
          if (!moving_cmd || !straight_cmd)
          {
              is_yaw_locked = false;
              prev_straight_cmd = false;
          }
          else
          {
              // Baru masuk mode lurus
              if (!prev_straight_cmd || !is_yaw_locked)
              {
                  target_yaw = current_yaw;
                  is_yaw_locked = true;
              }

              prev_straight_cmd = true;

              float yaw_error = target_yaw - current_yaw;

              // Wrap ke [-PI, PI]
              while (yaw_error > PI)
                  yaw_error -= 2.0f * PI;
              while (yaw_error < -PI)
                  yaw_error += 2.0f * PI;

              const float Kp_yaw = 135.0f;
              int yaw_correction = (int)roundf(yaw_error * Kp_yaw);
              Serial.print(" YawErr=");
              Serial.print(yaw_error * 180.0f / PI);

              Serial.print(" YawPWM=");
              Serial.print(yaw_correction);
              applied_pwm_L -= yaw_correction;
              applied_pwm_R += yaw_correction;
          }

          // E. CLAMP SETELAH DIKOREKSI
          applied_pwm_L = constrain(applied_pwm_L, 0, 255);
          applied_pwm_R = constrain(applied_pwm_R, 0, 255);
        }

        // F. TERAPKAN ARAH DAN PWM KE MOTOR
        digitalWrite(
          L_PIN_DIR,
          (smoothed_target_L >= 0.0f) ? LEFT_FORWARD_DIR : reverseDir(LEFT_FORWARD_DIR)
        );

        digitalWrite(
          R_PIN_DIR,
          (smoothed_target_R >= 0.0f) ? RIGHT_FORWARD_DIR : reverseDir(RIGHT_FORWARD_DIR)
        );

        Serial.print("Target L:");
        Serial.print(smoothed_target_L);

        Serial.print(" Target R:");
        Serial.print(smoothed_target_R);

        Serial.print(" RPM_L=");
        Serial.print(filtered_rpm_L);

        Serial.print(" RPM_R=");
        Serial.print(filtered_rpm_R);

        Serial.print(" PWM L:");
        Serial.print(applied_pwm_L);

        Serial.print(" PWM R:");
        Serial.println(applied_pwm_R);

        analogWrite(L_PIN_PWM, applied_pwm_L);
        analogWrite(R_PIN_PWM, applied_pwm_R);
      }
      else if (TEST_MODE == 1) {
        if (current_pwm < TEST_PWM) {
          current_pwm += PWM_RAMP_STEP;
          if (current_pwm > TEST_PWM) current_pwm = TEST_PWM;
        }
        digitalWrite(L_PIN_DIR, LEFT_FORWARD_DIR);
        digitalWrite(R_PIN_DIR, RIGHT_FORWARD_DIR);
        analogWrite(L_PIN_PWM, current_pwm);
        analogWrite(R_PIN_PWM, 0);
        applied_pwm_L = current_pwm;
        applied_pwm_R = 0;
      }
      else if (TEST_MODE == 2) {
        if (current_pwm < TEST_PWM) {
          current_pwm += PWM_RAMP_STEP;
          if (current_pwm > TEST_PWM) current_pwm = TEST_PWM;
        }
        digitalWrite(L_PIN_DIR, LEFT_FORWARD_DIR);
        digitalWrite(R_PIN_DIR, RIGHT_FORWARD_DIR);
        analogWrite(L_PIN_PWM, 0);
        analogWrite(R_PIN_PWM, current_pwm);
        applied_pwm_L = 0;
        applied_pwm_R = current_pwm;
      }

      // 5. PUBLISH DATA ROS
      struct timespec tv;
      clock_gettime(CLOCK_REALTIME, &tv);

      msg_imu.header.stamp.sec = tv.tv_sec;
      msg_imu.header.stamp.nanosec = tv.tv_nsec;
      msg_imu.linear_acceleration.x = accel_x_clean;
      msg_imu.linear_acceleration.y = accel_y_clean;
      msg_imu.linear_acceleration.z = accel_z_clean;
      msg_imu.angular_velocity.x = gyro_x_clean;
      msg_imu.angular_velocity.y = gyro_y_clean;
      msg_imu.angular_velocity.z = gyro_z_clean;

      msg_L.x = raw_rpm_L;
      msg_L.y = filtered_rpm_L;
      msg_L.z = (float)applied_pwm_L;

      msg_R.x = raw_rpm_R;
      msg_R.y = filtered_rpm_R;
      msg_R.z = (float)applied_pwm_R;

      rcl_publish(&pub_left, &msg_L, NULL);
      rcl_publish(&pub_right, &msg_R, NULL);
      rcl_publish(&pub_imu, &msg_imu, NULL);

      prev_ms = now;
    }

    // SPIN ROS
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  


  }