//void landing() {
//  while (true) {
//    C = 0.0343;
//
//    duration = sonar.ping_median(iterations);
//
//    distance = ((duration / 2) * C) - 10.54;
//
//    Serial.print("Distance = ");
//    if (distance >= 400 || distance < 0) {
//      Serial.println("Out of range");
//    }
//    else {
//      Serial.print(distance);
//      Serial.println(" cm");
//      delay(500);
//    }
//    delay(500);
//
//    /*///////////////////////////P I D///////////////////////////////////*/
//    /*Remember that for the balance we will use just one axis. I've choose the x angle
//      to implement the PID with. That means that the x axis of the IMU has to be paralel to
//      the balance*/
//
//    /*First calculate the error between the desired angle and
//      the real measured angle*/
//    errors = distance - desired_Height1 ;
//
//    /*Next the proportional value of the PID is just a proportional constant
//      multiplied by the error*/
//
//    pid_p = kp * errors;
//
//    /*The integral part should only act if we are close to the
//      desired position but we want to fine tune the error. That's
//      why I've made a if operation for an error between -2 and 2 degree.
//      To integrate we just sum the previous integral value with the
//      error multiplied by  the integral constant. This will integrate (increase)
//      the value each loop till we reach the 0 point*/
//    if (-3 < errors < 3)
//    {
//      pid_i += (ki * errors);
//    }
//
//    /*The last part is the derivate. The derivate acts upon the speed of the error.
//      As we know the speed is the amount of error that produced in a certain amount of
//      time divided by that time. For that we will use a variable called previous_error.
//      We substract that value from the actual error and divide all by the elapsed time.
//      Finnaly we multiply the result by the derivate constant*/
//
//    pid_d = kd * ((errors - previous_error) / DT);
//
//    /*The final PID values is the sum of each of this 3 parts*/
//    PID = pid_p + pid_i + pid_d;
//
//    /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
//      tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
//      have a value of 2000us the maximum value taht we could sybstract is 1000 and when
//      we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
//      to reach the maximum 2000us*/
//    if (PID < -1000)
//    {
//      PID = -1000;
//    }
//    if (PID > 1000)
//    {
//      PID = 1000;
//    }
//
//    /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
//
//    pwm_B_Left = throttle - PID;
//    pwm_B_Right = throttle - PID;
//    pwm_F_Left = throttle - PID;
//    pwm_F_Right = throttle - PID;
//
//    /*Once again we map the PWM values to be sure that we won't pass the min
//      and max values. Yes, we've already maped the PID values. But for example, for
//      throttle value of 1300, if we sum the max PID value we would have 2300us and
//      that will mess up the ESC.*/
//
//    //Right Front
//    if (pwm_F_Right < 1000)
//    {
//      pwm_F_Right = 1000;
//    }
//    if (pwm_F_Right > 2000)
//    {
//      pwm_F_Right = 2000;
//    }
//    //Left Front
//    if (pwm_F_Left < 1000)
//    {
//      pwm_F_Left = 1000;
//    }
//    if (pwm_F_Left > 2000)
//    {
//      pwm_F_Left = 2000;
//    }
//
//    //Right Back
//    if (pwm_B_Right < 1000)
//    {
//      pwm_B_Right = 1000;
//    }
//    if (pwm_B_Right > 2000)
//    {
//      pwm_B_Right = 2000;
//    }
//    //Left Back
//    if (pwm_B_Left < 1000)
//    {
//      pwm_B_Left = 1000;
//    }
//    if (pwm_B_Left > 2000)
//    {
//      pwm_B_Left = 2000;
//    }
//
//    /*Finnaly using the servo function we create the PWM pulses with the calculated
//      width for each pulse*/
//
//    left_F_prop.writeMicroseconds(pwm_F_Left);
//    left_B_prop.writeMicroseconds(pwm_B_Left);
//    right_F_prop.writeMicroseconds(pwm_F_Right);
//    right_B_prop.writeMicroseconds(pwm_B_Right);
//
//    previous_error = errors; //Remember to store the previous error.
//  }
//}
