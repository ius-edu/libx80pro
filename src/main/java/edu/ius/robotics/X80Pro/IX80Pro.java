package edu.ius.robotics.X80Pro;

/* 
 * Java implmentation written by Jesse Riddle, with significant documentation 
 * taken from the DrRobot WiRobot SDK Application Programming Interface 
 * (API) Reference Manual - (For MS Windows) Version: 1.0.8 Feb. 2004 by 
 * DrRobot, Inc. and some parts from the DrRobot Java motion control demo 
 * program.
 */

interface IX80Pro 
{
    /**
     * Sends a request command to the Sensing and Motion Controller (PMS5005) 
     * in order to get the sensor data related to motor control.
     *
     * @param packetNumber describes how many times sampled data should be sent
     *     packetNumber == 0 -- Stop sending the sensor data packets.
     *     packetNumber == 1 -- Send sensor data packets continuously 
     *                          until being asked to stop (default).
     *     packetNumber > 0 -- Send n = packetNumber packets of sensor data 
     *                         and then stop sending.
     *
     * Please Note:
     * Though adjustable, the maximum refresh rate for the PMS5005 is 20Hz or 
     * 50ms (default).
     *
     * See Also: setMotorSensorPeriod
     */
    void motorSensorRequest(int packetNumber);

    /**
     * Sends a request command to the Sensing and Motion Controller (PMS5005) 
     * in order to get all the standard sensor data.
     *
     * @param packetNumber describes how many times sampled data should be 
     * sent.
     *     packetNumber == 0 -- Stop sending the sensor data packets.
     *     packetNumber == 1 -- Send sensor data packets continuously 
     *                          until being asked to stop.
     *     packetNumber > 0 -- Send n = packetNumber packets of sensor data 
     *                         and then stop sending.
     *
     * Please Note:
     * Though adjustable, the maximum refresh rate for the PMS5005 is 20Hz or 
     * 50ms (default).
     *
     * See Also: setStandardSensorPeriod
     */
    void standardSensorRequest(int packetNumber);

    /**
     * Sends a request command to the Sensing and Motion Controller (PMS5005) 
     * in order to get all custom sensor data.
     *
     * @param packetNumber describes how many times sampled data should be 
     * sent.
     *     packetNumber == 0 -- Stop sending the sensor data packets.
     *     packetNumber == 1 -- Send sensor data packets continuously 
     *                          until being asked to stop (default).
     *     packetNumber > 0 -- Send n = packetNumber packets of sensor data 
     *                         and then stop sending.
     *
     * Please Note:
     * Though adjustable, the maximum refresh rate for the PMS5005 is 20Hz or 
     * 50ms (default).
     *
     * See Also: setCustomSensorPeriod
     */
    void customSensorRequest(int packetNumber);

    /**
     * Sends a request command to the Sensing and Motion Controller (PMS5005) 
     * in order to get all sensor data.
     *
     * @param packetNumber describes how many times sampled data should be 
     * sent.
     *     packetNumber == 0 -- Stop sending the sensor data packets.
     *     packetNumber == 1 -- Send sensor data packets continuously 
     *                          until being asked to stop.
     *     packetNumber > 0 -- Send n = packetNumber packets of sensor data 
     *                         and then stop sending.
     *
     * Please note:
     * Though adjustable, the maximum refresh rate for the PMS5005 is 20Hz or 
     * 50ms (default).
     *
     * See Also: setAllSensorPeriod
     */
    void allSensorRequest(int packetNumber);

    /**
     * Enables batch updating of motor-related sensor packets.
     *
     * Please note:
     * 1) The latest request setting of the packet number and the update rates 
     *    are used.
     * 2) By default, "all sensor data sending" is enabled.
     *
     * @see motorSensorRequest
     */
    void enableMotorSensorSending();

    /**
     * Enables batch updating of standard sensor packets.
     *
     * Please note:
     * 1) The latest request setting of the packet number and the update rates 
     *   are used.
     * 2) By default, "all sensor data sending" is enabled.
     *
     * @see disableMotorSensorSending
     */
    void enableStandardSensorSending();

    /**
     * Enables batch updating of custom sensor packets.
     *
     * Please note:
     * 1) The latest request setting of the packet number and the update rates 
     *    are used.
     * 2) By default, "all sensor data sending" is enabled.
     *
     * @see disableStandardSensorSending
     */
    void enableCustomSensorSending();

    /**
     * Enables batch updating of all sensor packets.
     *
     * Please note:
     * 1) The latest request setting of the packet number and the update rates 
     *    are used.
     * 2) By default, "all sensor data sending" is enabled.
     *
     * @see disableCustomSensorSending
     */
    void enableAllSensorSending();

    /**
     * Disables batch updating of motor-related sensor packets.
     *
     * @see enableMotorSensorSending
     */
    void disableMotorSensorSending();

    /**
     * Disables batch updating of standard sensor packets.
     *
     * @see enableStandardSensorSending
     */
    void disableStandardSensorSending();

    /**
     * Disables batch updating of custom sensor packets.
     *
     * @see enableCustomSensorSending
     */
    void disableCustomSensorSending();

    /**
     * Disables batch updating of all sensor packets.
     *
     * @see enableAllSensorSending
     */
    void disableAllSensorSending();

    /**
     * Sets the refresh rate for batch updating by motor-related sensor packets.
     *
     * @param timePeriod The update period in milliseconds for batch sensing 
     * packets to the PC central controller.
     *
     * Please note: The default timePeriod is 50ms.  The maximum refresh rate 
     * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
     *
     * @see motorSensorRequest
     */
    void setMotorSensorPeriod(int timePeriod);

    /**
     * Sets the refresh rate for batch updating by standard sensor packets.
     *
     * @param timePeriod The update period in milliseconds for batch sensing 
     * packets to the PC central controller.
     *
     * Please note: The default timePeriod is 50ms.  The maximum refresh rate 
     * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
     *
     * @see standardSensorRequest
     */
    void setStandardSensorPeriod(int timePeriod);

    /**
     * Sets the refresh rate for batch updating by custom sensor packets.
     *
     * @param timePeriod The update period in milliseconds for batch sensing 
     * packets to the PC central controller.
     *
     * Please note: The default timePeriod is 50ms.  The maximum refresh rate 
     * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
     *
     * @see customSensorRequest
     */
    void setCustomSensorPeriod(int timePeriod);

    /**
     * Sets the refresh rate for batch updating of all sensor packets.
     *
     * @param timePeriod The update period in milliseconds for batch sensing 
     * packets to the PC central controller.
     *
     * Please note: The default timePeriod is 50ms.  The maximum refresh rate 
     * possible for the PMS5005 Sensing and Motion Controller is 50ms @ 20Hz.
     *
     * @see allSensorRequest
     */
    void setAllSensorPeriod(int timePeriod);

    /**
     * Returns the current distance value between the relevant ultrasonic 
     * range sensor module (DUR5200) and the object in front of it.
     *
     * @param channel 0, 1, 2, 3, 4 or 5 for Sonar #1 through #6.
     *
     * @return 4 means a distance of 0 to 4 cm to object.
     * 4 to 254 means a distance of 4 to 254 cm to object.
     * 255 means 255 cm or longer distance to object.
     *
     * Please note: By default, the sensors are indexed clockwise, starting 
     * with the left-front sensor (robot first person perspective) at Sonar #1 
     * (channel 0).
     */
    int getSensorSonar(int channel);

    /**
     * Returns the current distance measurement value between an infrared 
     * sensor and the object in front of it.
     *
     * @param channel 0, 1, 2, 3, 4 or 5 for IR #1 through #6.
     *
     * @return <= 585 means a distance of 80 cm or longer to object.
     * 585 to 3446 means a distance of 80 to 8 cm to object.
     * >= 3446 means a distance of 0 to 8 cm to object.
     *
     * Please Note: The relationship between the return data and the distance 
     * is not linear.  Please refer to the sensor's datashee for distance-
     * voltage curve.  The data returned is the raw data of the analog to 
     * digital converter.  The output voltage of the sensor can be calculated 
     * from the following equation: sensorOutputVoltage = (ival)*3.0/4095(v)
     */
    int getSensorIrRange(int channel);

    /**
     * Returns the current human alarm data from the DHM5150 Human Motion 
     * Sensor Module.  Please refer to the DHM5150 hardware manual for 
     * detailed information.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     *
     * @return the raw value from the analog to digital converter indicating 
     * the amplified (5x) output voltage of the sensor device.  The data range 
     * is between 0 and 4095.  When there is no human present, the module 
     * output voltage is ~1.5V and the return value is about 2047.
     *
     * Please note: To detect human presence, the application should compare 
     * the difference of two samples (to detect the change from "absence" to 
     * "presence"), and also compare the sample data to a user defined 
     * threshold (to determine whether to report an alarm or not).  The 
     * threshold determines the sensitivity of the sensor.  The higher the 
     * threshold, the lower the sensitivity will be.
     */
    int getSensorHumanAlarm(int channel);

    /**
     * Returns the current human motion value from the DHM5150 Human Motion 
     * Sensor Module.  Please refer to the DHM5150 hardware manual for 
     * detailed information.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     *
     * @return the un-amplified raw value of the analog to digital converter 
     * indicating the output voltage of the sensor device.  The data ranges 
     * between 0 and 4095.
     *
     * Please note: To detect human motion direction, the application should 
     * compare the difference of two samples of each sensor module's output 
     * (to detect the change from "absence" to "presence"), and then compare 
     * the sample data of the two sensor modules.  For a single source of 
     * human motion, the different patterns of the two sensor modules manifest 
     * the direction of motion.  The relationship can be obtained emperically.
     */
    int getSensorHumanMotion(int channel);

    /**
     * Returns the current tilt angle value in the horizontal direction from 
     * the DTA5102 Tilting and Acceleration Sensor Module.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     *
     * @return Tilting Angle = arcsin((ival - ZeroGValue)/abs(90DegreeGValue - 
     * ZeroGValue));
     * Where 90DegreeGValue and ZeroGValue are module-specific values that can 
     * be measured by experiment:
     * 1) Place the sensor level, so that the gravity vector is perpendicular 
     *    to the measured sensor axis.
     * 2) Take the measurement and this value would be the ZeroGValue.
     * 3) Rotate the sensor so that the gravity vector is parallel with the 
     *    measured axis.
     * 4) Take the measurement and this value would be the 90DegreeGValue.
     * 5) Repeat this step for the other direction.
     * Typical value of ZeroGValue is about 2048 and abs(90DegreeGValue - 
     * ZeroGValue) is about 1250.
     */
    int getSensorTiltingX(int channel);

    /**
     * Returns the current tilt angle value in the vertical direction from 
     * the DTA5102 Tilting and Acceleration Sensor Module.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     *
     * @return Tilting Angle = arcsin((ival - ZeroGValue)/abs(90DegreeGValue - 
     * ZeroGValue));
     * Where 90DegreeGValue and ZeroGValue are module-specific values that can 
     * be measured by experiment:
     * 1) Place the sensor level, so that the gravity vector is perpendicular 
     *    to the measured sensor axis.
     * 2) Take the measurement and this value would be the ZeroGValue.
     * 3) Rotate the sensor so that the gravity vector is parallel with the 
     *    measured axis.
     * 4) Take the measurement and this value would be the 90DegreeGValue.
     * 5) Repeat this step for the other direction.
     * Typical value of ZeroGValue is about 2048 and abs(90DegreeGValue - 
     * ZeroGValue) is about 1250.
     */
    int getSensorTiltingY(int channel);

    /**
     * Returns the current air temperature values near the relevant DC motor 
     * drive modules (MDM5253).
     *
     * The current air temperature values could be used for 
     * monitoring whether the motor drivers are overheating or not.  This 
     * situation usually occurs if the motor currents are kept high (but still 
     * lower than the current limit of the motor driver module) for a 
     * significant amount of time, which may result in unfavorable inner or 
     * external system conditions and is not recommended for regular system 
     * operations.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     * 
     * @return The raw value of the analog to digital converter indicating the 
     * output voltage of the sensor.  The data range of the return value is 
     * between 0 and 4095.  The output voltage of the sensor can be calculated 
     * from the following equation: Temperature = 100 - (ival - 980)/11.6 
     * where Temperature is in degrees Celsius.
     */
    int getSensorOverheat(int channel);

    /**
     * Returns the current temperature value from the 
     * DA5280 Ambient Temperature Sensor Module.
     *
     * @return Temperature = (ival - 1256) / 34.8, where Temperature is in 
     * degrees Celsius.
     */
    int getSensorTemperature();

    /**
     * Returns the four parts of a two-16-bit-code infrared 
     * remote control command captured by the Sensing and Motion Controller 
     * (PMS5005) through the Infrared Remote Controller Module (MIR5500).
     *
     * @param index Refers to byte 0, 1, 2, or 3.  See return value for details
     *
     * @return The infrared remote control command (4 byte code) is as follows:
     * Key Code: byte[2] byte[1] byte[0]
     * Repeat Code: byte[3]
     * Where the repeat byte would be 255 if the button is pressed continuously
     */
    int getSensorIrCode(int index);

    /**
     * Sends two 16-bit words of infrared communication output data to the 
     * Sensing and Motion Controller (PMS5005).
     *
     * The PMS5005 will then send the data out through the infrared Remote 
     * Controller Module (MIR5500).  In the case it is used as an infrared 
     * remote control, the output data serves as the remote control command.
     *
     * @param lowWord First word
     * @param highWord Second word
     *
     * Please note:
     * 1) With infrared communication, the data format and interpretation can 
     *    be defined by the user at the application level.
     * 2) The control command should be compatible with the device to which 
     *    the command is sent.
     * 3) This API method is under development and will be available soon.
     */
    void setInfraredControlOutput(int lowWord, int highWord);

    /**
     * Returns the current value of the power supply voltage for the channel 
     * specified.
     *
     * Returns the current value of the relevant power supply voltage if the 
     * battery voltage monitor is enabled (default).
     * If the battery voltage monitor is disabled, this method  
     * returns the relevant custom A/D inputs, if the custom A/D input is 
     * enabled -- which may be configured by the jumpers on the PMS5005 board.
     * Please refer to the PMS5005 Robot Sensing and Motion Controller User 
     * Manual for detailed information on hardware settings.
     *
     * @param channel 0 -- battery of DSP circuits (or custom A/D channel #1)
     * channel 1 -- battery of DC motors (or custom A/D channel #2)
     * channel 2 -- battery for servo motors (or custom A/D channel #3)
     *
     * @return The raw value of the analog to digital converter indicating 
     * the output voltage of the monitor.  The data range is between 0 and 4095
     *
     * Please note: When monitoring the voltage of the power supply, the 
     * following equations can be used to calculate the real voltage values.
     * 1) Power supply voltage of DSP circuits = 9v*(ival/4095)
     * 2) Power supply voltage of DC motors = 24v*(ival/4095)
     * 3) Power supply voltage of servo motors = 9v*(ival/4095)
     */
    int getSensorBatteryAd(int channel);

    /**
     * Returns the current value of the reference voltage of the A/D converter 
     * of the controller DSP.
     *
     * @return The raw value of the analog to digital converter indicating the 
     * output voltage of the monitor.  The data range is between 0 and 4095.  
     * The following equation can be used to calculate the actual voltage 
     * values: Voltage = 6v*(ival/4095)
     */
    int getSensorRefVoltage();

    /**
     * Returns the current value of the reference voltage of the A/D converter 
     * of the controller DSP.
     *
     * @return The raw value of the analog to digital converter indicating the 
     * output voltage of the monitor.  The data range is between 0 and 4095.  
     * The following equation can be used to calculate the actual voltage 
     * values: Voltage = 6v*(ival/4095)
     */
    int getSensorPotVoltage();

    /**
     * Returns the current value of the specified potentiometer position sensor
     *
     * @param channel 0, 1, 2, 3, 4, or 5 for Potentiometer sensor #1 through 6
     *
     * @return The raw value given by the analog to digital converter 
     * indicating the output voltage of the sensor.  The data range is between 
     * 0 and 4095.  The angular position can be calculated as follows, with the 
     * 180 degree position defind at the sensors' physical middle position.  
     * Single sensor or dual sensor can be used for rotation measurement.
     *
     * Please note:
     * 1) Single sensor mode is mainly used for the control of a robot joint 
     *    with a limited rotation range.  The effective mechanical rotation 
     *    range is 14 degrees to 346 degrees, corresponding to the effective 
     *    electrical rotation range 0 degrees to 332 degrees.
     *      Angle position (degrees) = (ival - 2048)/4095*333 + 180
     * 2) Dual-sensor mode is mainly used for continuous rotating joint control 
     *    (e.g. a wheel).  The effective rotation range is 0 degrees to 360 
     *    degrees.  Dual sensorconfiguration is only available for channel 0 
     *    and channel 1.  By connecting two potentiometers to potentiometer 
     *    channel 0 and channel 5, and by specifying the sensor type with 
     *    command setDcMotorSensorUsage set to "Dual potentiometer sensor" 
     *    the channel 0 reading will combine these two sensor readings into 
     *    0 degrees to 360 degree range.  For channel 1, channel 1 and channel 
     *    4 would be combined instead.
     *      Angle position (degrees) = (ival - 2214)/2214*180 + 180
     *
     * @see setDcMotorSensorUsage
     */
    int getSensorPot(int channel);

    /**
     * Returns the sampling value of the selected motor current sensor.
     *
     * @param channel 0, 1, 2, 3, 4, or 5 for sensor #1, #2, #3, #4, #5, or #6.
     *
     * @return The raw value of the analog to digital converter indicating the 
     * motor current.  The data range is between 0 and 4095.  The actual 
     * current value can be calculated with the following formula: 
     * Motor Current (amperes) = ival/728 ( = ival*3*375/200/4095)
     */
    int getMotorCurrent(int channel);

    /**
     * Returns +1, 0, or -1 to indicate the direction of rotation.
     *
     * @param channel 0 for left encoder, 1 for right encoder (robot first 
     * person perspective).
     *
     * @return 1 to indicate positive direction, 0 to indicate no movement, 
     * and -1 to indicate negative direction.
     */
    int getEncoderDirection(int channel);

    /**
     * Returns the current pulse counter to indicate the position of rotation.
     *
     * @param channel 0 for left encoder, 1 for right encoder (robot first 
     * person perspective).
     *
     * @return Pulse counter, an integral value to rotation with range of 
     * 0 to 32767 in cycles.
     */
    int getEncoderPulse(int channel);

    /**
     * Returns the rotation speed.  The unit is defined as the absolute value 
     * of the pulse change within 1 second.
     *
     * @param channel 0 for left encoder, 1 for right encoder (robot first 
     * person perspective).
     *
     * @see setDcMotorSensorUsage
     */
    int getEncoderSpeed(int channel);

    /**
     * Returns the sampling value of the custom analog to digital 
     * input signals.  By default, custom AD1 - AD3 are used as the inputs of 
     * power supply voltage monitors for DSP circuits, DC motors, and servo 
     * motors.  Users may change this setting by configuring the jumpers on 
     * the PMS5005.  Please refer to the PMS5005 Robot Sensing and Motion 
     * Controller hardware user's manual for detailed information on hardware 
     * jumper settings.
     *
     * @param channel 0, 1, 2, 3, 4, 5, 6, or 7 for channel #1 through #8
     *
     * @return The raw value from the analog to digital converter indicating 
     * the input voltage levels.  The data range is between 0 and 4095. The 
     * voltage levels can be calculated from the following equation: 
     * Sensor output voltage = (ival) * 3.0v/4095
     *
     * @see getSensorBatteryAd
     */
    int getCustomAd(int channel);

    /**
     * Returns a value with the lower 8 bits corresponding to the 8 channel 
     * custom digital inputs.
     *
     * @param channel 0, 1, 2, 3, 4, 5, 6, or 7 for channel #1 through #8.
     */
    int getCustomDIn(int channel);

    /**
     * Sets the 8-channel custom digital outputs.
     *
     * @param ival -- only the lower 8 bits are valid and can change the 
     * corresponding outputs of the 8 channels.  The MSB of the lower byte 
     * represents channel #8 and LSB of the lower byte represents channel #1.
     */
    void setCustomDOut(int ival);

    // Motion Control

    /**
     * Sets the motor polarity to 1 or -1 for the motor channel specified.
     * 
     * When the motor is running in the positive dirction, the pontentiometer 
     * value is also increasing; motor polarity should be set to 1 (default).
     * When the motor is running in the negative direction, the value is 
     * decreasing, and the motor polarity should be set to -1 to change the 
     * sensor mounting value so that the potentiometer value increases.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     * 
     * @param polarity 1 or -1
     */
    void setMotorPolarity(int channel, int polarity);

    /**
     * Enables the specified DC motor control channel.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     *
     * @deprecated
     *
     * @see resumeDcMotor
     */
    void enableDcMotor(int channel);

    /**
     * Disables the specified DC motor control channel.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     *
     * @deprecated
     *
     * @see suspendDcMotor
     */
    void disableDcMotor(int channel);

    /**
     * Resumes the specified DC motor control channel.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     */
    void resumeDcMotor(int channel);

    /**
     * Suspends the specified DC motor control channel.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     *
     * All motor control channels are initially suspended at boot-up.
     */
    void suspendDcMotor(int channel);

    /**
     * Sets up the PID control parameters of the specified DC motor channel 
     * for position control.
     *
     * @param channel 0 for left, 1 for right (robot first person perspective)
     * @param Kp proportional gain (default is 50)
     * @param Kd derivative gain (default is 5)
     * @param Ki_x100 the desired integral gain * 100.  when Ki_100 = 100, 
     * the actual integral control term is Ki = 1.  Ki_x100 has a range of 
     * 0 to 25599, where 0 means no integral control (default).
     *
     * @see setDcMotorControlMode
     */
    void setDcMotorPositionControlPid(int channel, int Kp, int Kd, int Ki_x100);

    void setDcMotorVelocityControlPID(byte channel, int Kp, int Kd, int Ki);
    
    /**
     * This filtering feature is still under development. All data will be 
     * treated as raw data.
     */
    void setDcMotorSensorFilter(int channel, int filterMethod);

    /**
     * Set the sensor type for the specified DC motor control channel on the 
     * Sensing and Motion Controller (PMS5005).
     * 
     * The available sensor types are 
     * single potentiometer, dual potentiometers, and quadrature encoder.  The 
     * single potentiometer sensor is for the control of robot joint with 
     * limited rotation range (0 degrees to 332 degrees).  The dual 
     * potentiometers and the quadrature sensor are for use with continuously 
     * rotating joints (e.g. wheels).
     *
     * @param channel 0, 1, 2, 3, 4, or 5 for single potentiometer sensor
     * channel 0, 1, or 2 for dual potentiometer sensor
     * channel 0 or 1 for quadrature encoder
     * @param sensorType 0 -- single potentiometer sensor
     * 1 -- dual potentiometer sensor
     * 2 -- quadrature encoder
     *
     * Please note
     * 1) The electrical angular range of the potentiometer position sensor is 
     *    0 degrees to 332 degrees and the corresponding mechanical rotation 
     *    range is 14 degrees to 346 degrees, with the 180 degree position 
     *    defined as the sensor's physical middle position.
     * 2) Each DC motor channel for dual potentiometer sensors utilizes two 
     *    potentiometer channels.  DC motor channel 0 
     *
     * @see getSensorPot
     */
    void setDcMotorSensorUsage(int channel, int sensorType);

    /**
     * Sets the control mode of the specified DC motor control channel on the 
     * Sensing and Motion Controller (PMS5005).  The available control modes 
     * are open-loop PWM control, closed-loop position control, and closed-
     * loop velocity control.
     *
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param controlMode 0 == open-loop PWM control, 1 == closed-loop position
     * control, 2 == closed-loop velocity control
     *
     * @see setDcMotorPositionControlPid
     * @see setDcMotorVelocityControlPid
     */
    void setDcMotorControlMode(int channel, int controlMode);

    /**
     * Sends the position control command to the specified motion control 
     * channel on the Sensing and Motion Controller (PMS5005).  
     * The command includes the target position and the target time 
     * period to execute the command.  The current trajectory planning method 
     * with time control is linear.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target position value
     * @param timePeriod Executing time in milliseconds
     */
    void dcMotorPositionTimeCtrl(int channel, int cmdValue, int timePeriod);
    
    /**
     * Sends the position control command to the specified motion control 
     * channel on the Sensing and Motion Controller (PMS5005).  The command 
     * includes the target position but no time period specified to execute 
     * the command.  The motion controller will drive the motor to the target 
     * position at the maximum speed.
     *
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target position value
     *
     * 1) Motor will be enabled automatically by the system when this command 
     *    is received.
     * 2) No velocity is available for motor channel using single potentiometer 
     *    sensor.
     * 3) The unit of the velocity is (Position change in A/D sampling data) / 
     *    second when using dual potentiometer sensor for rotational postion 
     *    measurement and pulse/second when using quadrature encoder.
     * 
     * @see dcMotorVelocityTimeCtrl
     * @see getSensorPot
     */
    void dcMotorPositionNonTimeCtrl(int channel, int cmdValue);
    
    /**
     * Sends the PWM control command to the specified motion control channel on 
     * the Sensing and Motion Controller (PMS5005).  The command includes the 
     * target pulse width value and the time period to execute the command. 
     * The current trajectory planning method for time control is linear.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target pulse width value
     * @param timePeriod Executing time in milliseconds
     * 
     * 1) The specified channel (motor) will be enabled automatically by the 
     *    system when this command is received.
     * 2) Target pulse width value range is 0 to 32767 (0x7FFF), corresponding 
     *    to the duty cycle of 0 to 100% linearly.
     * 3) A pulse width value of 16383 means 50% duty cycle, putting the motor 
     *    in the stop (neutral) stage.  Any value in between 16384 - 32767 will 
     *    cause the motor to turn clockwise (facing the front side of the 
     *    motor) and any value in between 0 - 16362 will cause the motor to 
     *    turn counter-clockwise.
     * 
     * @see dcMotorPwmNonTimeCtrl
     */
    void dcMotorPwmTimeCtrl(int channel, int cmdValue, int timePeriod);

    /**
     * Sends the PWM control command to the specified motion control channel on 
     * the Sensing and Motion Controller (PMS5005).  The command includes the 
     * target pulse width value without an execution time period specified.  
     * The motion controller will set the PWM output of this channel to the 
     * target value immediately.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target pulse width value
     * 
     * 1) The specified channel (motor) will be enabled automatically by the 
     *    system when this command is received.
     * 2) Target pulse width value range is 0 to 32767 (0x7FFF), corresponding 
     *    to the duty cycle of 0 to 100% linearly.
     * 3) A pulse width value of 16383 means 50% duty cycle, putting the motor 
     *    in the "Stop" stage.  Any value between 16364 - 32767 will cause the 
     *    motor to turn clockwise (facing the front side of the motor) and any 
     *    value in between 0 - 16362 will cause the motor to turn 
     *    counter-clockwise.
     * 
     * @see dcMotorPwmTimeCtrl
     */
    void dcMotorPwmNonTimeCtrl(int channel, int cmdValue);
    
    /**
     * Sends the position control command to all 6 DC motor control channels on 
     * the sensing and motion controller (PMS5005) at the same time.  The 
     * command includes the target positions and the time period to execute the 
     * command.  The current trajectory planning method for time control is 
     * linear.
     * 
     * @param pos1 Target position for channel #1
     * @param pos2 Target position for channel #2
     * @param pos3 Target position for channel #3
     * @param pos4 Target position for channel #4
     * @param pos5 Target position for channel #5
     * @param pos6 Target position for channel #6
     * @param timePeriod Execution time in milliseconds
     * 
     * 1) All DC Motors will be enabled automatically by the system when this 
     *    command is received.
     * 2) Target position value is the A/D sampling data range 0 to 4095 when 
     *    using single potentiometer, 0-4428 when using dual potentiometers.
     * 3) Please refer to the description of getSensorPot for data conversion 
     *    between angular values and the A/D sampling data values.
     * 4) When using the encoder as sensor input, the target position value is 
     *    the pulse count in the range of 0-32767.
     * 5) When omitting motor channels from control, the command value should 
     *    be set to -32768 (0x8000), which implies NO_CONTROL.
     * 
     * @see getSensorPot
     * @see dcMotorPositionTimeCtrl
     */
    void dcMotorPositionTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, 
                                   int pos5, int pos6, int timePeriod);
    
    /**
     * Sends the position control command to all 6 DC motor control channels on 
     * the Sensing and Motion Controller (PMS5005) at the same time.  The 
     * command includes the target positions without a specified time period 
     * for execution.  The motion controller will drive the motor to reach the 
     * target position with maximum effort.
     * 
     * @param pos1 Target position for channel #1
     * @param pos2 Target position for channel #2
     * @param pos3 Target position for channel #3
     * @param pos4 Target position for channel #4
     * @param pos5 Target position for channel #5
     * @param pos6 Target position for channel #6
     * 
     * 1) All DC Motors will be enabled automatically by the system when this 
     *    command is received.
     * 2) Target position value is the A/D sampling data range 0 to 4095 when 
     *    using single potentiometer, 0-4428 when using dual potentiometers.
     * 3) Please refer to the description of getSensorPot for data conversion 
     *    between angular values and the A/D sampling data values.
     * 4) When using the encoder as sensor input, the target position value is 
     *    the pulse count in the range of 0-32767.
     * 5) When omitting motor channels from control, the command value should 
     *    be set to -32768 (0x8000), which implies NO_CONTROL.
     * 
     * @see getSensorPot
     * @see dcMotorPositionNonTimeCtrl
     */
    void dcMotorPositionNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, 
                                       int pos5, int pos6);
    
    /**
     * Sends the velocity control command to all 6 DC motor control channels on 
     * the Sensing and Motion Controller (PMS5005) at the same time.  The 
     * command includes the target velocities and the time period to execute 
     * the command.  The trajectory planning method for time control is linear.
     * 
     * @param pos1 Target velocity for channel #1
     * @param pos2 Target velocity for channel #2
     * @param pos3 Target velocity for channel #3
     * @param pos4 Target velocity for channel #4
     * @param pos5 Target velocity for channel #5
     * @param pos6 Target velocity for channel #6
     * @param timePeriod Execution time in milliseconds
     * 
     * 1) Motor will be enabled automatically by the system when this command 
     *    is received.
     * 2) No velocity control is available for a motor channel operating in 
     *    single potentiometer mode.
     * 3) The unit of the velocity is (Position change in A/D sampling data) / 
     *    second when using dual potentiometer sensors for rotational position 
     *    measurements and pulse/second when using quadrature encoder.
     * 4) Please refer to the description of getSensorPot for data conversion 
     *    between angular values and the A/D sampling data values.
     * 5) When omitting motors from control, send the command value -32768
     *    (0x8000), which implies NO_CONTROL.
     * 
     * @see dcMotorVelocityTimeCtrl
     */
    void dcMotorVelocityTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, 
                                    int pos5, int pos6, int timePeriod);

        /**
     * Sends the velocity control command to all 6 DC motor control channels on 
     * the Sensing and Motion Controller (PMS5005) at the same time.  The 
     * command includes the target velocities without specifying an execution 
     * time period.  The motion controller will drive the motor to achieve 
     * the target velocity with maximum effort.
     * 
     * @param pos1 Target velocity for channel #1
     * @param pos2 Target velocity for channel #2
     * @param pos3 Target velocity for channel #3
     * @param pos4 Target velocity for channel #4
     * @param pos5 Target velocity for channel #5
     * @param pos6 Target velocity for channel #6
     * 
     * 1) Motor will be enabled automatically by the system when this command 
     *    is received.
     * 2) No velocity control is available for a motor channel operating in 
     *    single potentiometer mode.
     * 3) The unit of the velocity is (Position change in A/D sampling data) / 
     *    second when using dual potentiometer sensors for rotational position 
     *    measurements and pulse/second when using quadrature encoder.
     * 4) Please refer to the description of getSensorPot for data conversion 
     *    between angular values and the A/D sampling data values.
     * 5) When omitting motors from control, send the command value -32768
     *    (0x8000), which implies NO_CONTROL.
     * 
     * @see dcMotorVelocityNonTimeCtrl
     */
    void dcMotorVelocityNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, 
                                      int pos5, int pos6);
    
    /**
     * Sends the PWM control command to all 6 DC motor control channels on the 
     * Sensing and Motion Controller (PMS5005) at the same time.  The command 
     * includes the target PWM values and the time period for execution.  The 
     * current trajectory planning method for time control is linear.
     * 
     * @param pos1 Target PWM value for channel #1
     * @param pos2 Target PWM value for channel #2
     * @param pos3 Target PWM value for channel #3
     * @param pos4 Target PWM value for channel #4
     * @param pos5 Target PWM value for channel #5
     * @param pos6 Target PWM value for channel #6
     * @param timePeriod Execution time in milliseconds
     * 
     * 1) All channels (motors) will be enable automatically by the system when 
     *    this command is received.
     * 2) Target pulse width value range is 0 to 32767 (0x7FFF), corresponding 
     *    to the duty cycle of 0 to 100% linearly.
     * 3) A pulse width value of 16383 means 50% duty cycle, putting the motor 
     *    in the stop (neutral) stage.  Any value in between 16384 - 32767 will 
     *    cause the motor to turn clockwise (facing the front side of the 
     *    motor) and any value in between 0 - 16362 will cause the motor to 
     *    turn counter-clockwise.
     * 4) When omitting motors from control, the command value of -32768
     *    (0x8000), should be sent.  This implies NO_CONTROL.
     */
    void dcMotorPwmTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, 
                              int pos6, int timePeriod);
    
        /**
     * Sends the PWM control command to all 6 DC motor control channels on the 
     * Sensing and Motion Controller (PMS5005) at the same time.  The command 
     * includes the target PWM values without a specified time period for 
     * execution.  The motion controller will adjust the pulse width right away.
     * 
     * @param pos1 Target PWM value for channel #1
     * @param pos2 Target PWM value for channel #2
     * @param pos3 Target PWM value for channel #3
     * @param pos4 Target PWM value for channel #4
     * @param pos5 Target PWM value for channel #5
     * @param pos6 Target PWM value for channel #6
     * 
     * 1) All channels (motors) will be enable automatically by the system when 
     *    this command is received.
     * 2) Target pulse width value range is 0 to 32767 (0x7FFF), corresponding 
     *    to the duty cycle of 0 to 100% linearly.
     * 3) A pulse width value of 16383 means 50% duty cycle, putting the motor 
     *    in the stop (neutral) stage.  Any value in between 16384 - 32767 will 
     *    cause the motor to turn clockwise (facing the front side of the 
     *    motor) and any value in between 0 - 16362 will cause the motor to 
     *    turn counter-clockwise.
     * 4) When omitting motors from control, the command value of -32768
     *    (0x8000), should be sent.  This implies NO_CONTROL.
     */
    void dcMotorPwmNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, 
                                 int pos5, int pos6);
    
    /**
     * Enables the specified servo motor control channel.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * 
     * All servo motor channels are disable initially at system startup.  They 
     * need to be enabled explicitly before use.
     * 
     * @see disableServo
     */
    void enableServo(int channel);
    
    /**
     * Disables the specified servo motor control channel.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * 
     * All servo motor channels are disable initially at system startup.  They 
     * need to be enabled explicitly before use.
     * 
     * @see enableServo
     */
    void disableServo(int channel);
    
    /**
     * Sends the position control command to the specified servo motor control 
     * channel on the Sensing and Motion Controller (PMS5005).  The command 
     * includes the target position command and the time period to execute the 
     * command.  The current trajectory planning method for time control is 
     * linear.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target Pulse Width (in milliseconds) * 2250
     * @param timePeriod Executing time in milliseconds
     * 
     * Usually, a standard remote control servo motor expects to get the 
     * specified pulse width every 20 seconds in order to hold the 
     * corresponding angle position.  The pulse width value in milliseconds 
     * for 0 degrees, 90 degrees, and 180 degrees are servo manufacturer and 
     * model dependent.  They are around 1ms, 1.5ms, and 2.0ms respectively for 
     * most common servos.  Experiments are required to obtain the exact value 
     * for a specific servo motor.
     * 
     * @see servoNonTimeCtrl
     */
    void servoTimeCtrl(int channel, int cmdValue, int timePeriod);
    
    /**
     * Sends the position control command to the specified servo motor control 
     * channel on the Sensing and Motion Controller (PMS5005).  The command 
     * includes the target position command without a specific time period for 
     * execution.  The motion controller will send the desired pulse width to 
     * the servo motor right away.
     * 
     * @param channel 0, 1, 2, 3, 4, or 5
     * @param cmdValue Target pulse width (ms) * 2250
     * 
     * @see servoTimeCtrl
     */
    void servoNonTimeCtrl(int channel, int cmdValue);
    
    /**
     * Sends the position control command to all 6 servo motor 
     * control channels on the Sensing and Motion Controller (PMS5005) at the 
     * same time.
     *
     * The command includes the target position commands and the 
     * time period to execute the command.  The current trajectory planning 
     * method for time control is linear.
     *
     * @param pos1 Target position for channel #1 (Left Motor on X80Pro)
     * @param pos2 Target position for channel #2 (-Right Motor on X80Pro)
     * @param pos3 Target position for channel #3 (NO_CONTROL on X80Pro)
     * @param pos4 Target position for channel #4 (NO_CONTROL on X80Pro)
     * @param pos5 Target position for channel #5 (NO_CONTROL on X80Pro)
     * @param pos6 Target position for channel #6 (NO_CONTROL on X80Pro)
     * @param timePeriod Executing time in milliseconds
     * 
     * When omitting servo motors from control, please send the command value 
     * -32768 (0x8000), which implies NO_CONTROL.
     * 
     * @see servoTimeCtrl
     */
    void servoTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, 
			 int pos5, int pos6, int timePeriod);

    /**
     * Sends the position control command to all 6 servo motor 
     * control channels on the Sensing and Motion Controller (PMS5005) at the 
     * same time.
     *
     * The command includes the target position commands without specifying a 
     * time period in which to execute the command.  The motion controller 
     * sends the desired pulse width to the servo motor right away.
     *
     * @param pos1 Target position for channel #1 (Left Motor on X80Pro)
     * @param pos2 Target position for channel #2 (-Right Motor on X80Pro)
     * @param pos3 Target position for channel #3 (NO_CONTROL on X80Pro)
     * @param pos4 Target position for channel #4 (NO_CONTROL on X80Pro)
     * @param pos5 Target position for channel #5 (NO_CONTROL on X80Pro)
     * @param pos6 Target position for channel #6 (NO_CONTROL on X80Pro)
     * 
     * When omitting servo motors from control, please send the command value 
     * -32768 (0x8000), which implies NO_CONTROL.
     * 
     * @see servoNonTimeCtrl
     */
    void servoNonTimeCtrlAll(int pos1, int pos2, int pos3, int pos4, int pos5, 
                            int pos6);
    
    /**
     * Displays the image data in the file bmpFileName (BMP format) on the 
     * graphic LCD connected to the Sensing and Motion Controller (PMS5005).
     * 
     * @param bmpFileName Full path of the BMP file for displaying
     * 
     * The graphic LCD display is monochrome with dimensions 128 by 64 pixels.  
     * The bmp image must be 128x64 pixels in mono.
     */
    void lcdDisplayPMS(String bmpFileName);
    
    /*
    void standardSensorEvent();
    void customSensorEvent();
    void motorSensorEvent();
     */
}

