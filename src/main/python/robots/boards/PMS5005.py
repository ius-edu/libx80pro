class PMS5005(object):
    """ Python implementation for PMS5005 controller written by Jesse Riddle """

    # The structure for a packet to the PMS5005 is the following: STX0 = 94 
    # (always) STX1 = 2 (always) RID = 1 (generally) Reserved = 0 (generally) 
    # DID (Data ID) = <DID> (your choice of DID) LENGTH = <len> (Length from 
    # next element to CRC byte) DATA = <data> (may be more than 1 byte in 
    # length) CHECKSUM = <checksum> (use calcCRC() to calculate, passing in 
    # the cmd, ETX0 = 94 (always) ETX1 = 13 (always)
    HEADER_LENGTH = 7
    PAYLOAD_OFFSET = 6
    DID_OFFSET = 4

    # Start transmission, End transmission
    STX0 = 94
    STX1 = 2
    ETX0 = 94
    ETX1 = 13

    POSITION_CTRL = 3
    ALL_POSITION_CTRL = 4
    PWM_CTRL = 5
    ALL_PWM_CTRL = 6
    PARAM_SET = 7

    DC_POSITION_PID = 7
    DC_VELOCITY_PID = 8
    DC_SENSOR_USAGE = 13
    DC_CTRL_MODE = 14

    POWER_CTRL = 22
    LCD_CTRL = 23
    VELOCITY_CTRL = 26
    ALL_VELOCITY_CTRL = 27
    TOGGLE_DC_MOTORS = 30
    CONSTELLATION_CTRL = 80
    GET_MOTOR_SENSOR_DATA = 123
    GET_CUSTOM_SENSOR_DATA = 124
    GET_STANDARD_SENSOR_DATA = 127
    SETUP_COM = 255

    PWM_CTRL_MODE = 0
    POSITION_CTRL_MODE = 1
    VELOCITY_CTRL_MODE = 2

    KP_ID = 1
    KD_ID = 2
    KI_ID = 3

    NON_CTRL_CMD = 0xffff;
    NO_CTRL = 0x8000

    ULTRASONIC_OFFSET = 0 + HEADER_LENGTH
    ENCODER_PULSE_OFFSET = 24 + HEADER_LENGTH
    ENCODER_SPEED_OFFSET = 32 + HEADER_LENGTH
    STANDARD_IR_RANGE_OFFSET = 24 + HEADER_LENGTH
    CUSTOM_IR_RANGE_OFFSET = 4 + HEADER_LENGTH # CustomAD3
    HUMAN_ALARM_OFFSET = 6 + HEADER_LENGTH
    HUMAN_MOTION_OFFSET = 8 + HEADER_LENGTH
    TILTING_X_OFFSET = 14 + HEADER_LENGTH
    TILTING_Y_OFFSET = 16 + HEADER_LENGTH
    ENCODER_DIRECTION_OFFSET = 32 + HEADER_LENGTH
    MOTOR_SPEED_OFFSET = 26 + HEADER_LENGTH
    CUSTOM_AD_OFFSET = 0 + HEADER_LENGTH
    TEMPERATURE_AD_OFFSET = 22 + HEADER_LENGTH
    OVERHEAT_SENSOR_OFFSET = 18 + HEADER_LENGTH
    INFRARED_COMMAND_OFFSET = 26 + HEADER_LENGTH
    BATTERY_SENSOR_OFFSET = 30 + HEADER_LENGTH
    REFERENCE_VOLTAGE_OFFSET = 36 + HEADER_LENGTH
    POTENTIOMETER_POWER_OFFSET = 38 + HEADER_LENGTH
    POTENTIOMETER_SENSOR_OFFSET = 0 + HEADER_LENGTH
    MOTOR_CURRENT_SENSOR_OFFSET = 12 + HEADER_LENGTH

    @staticmethod
    def calcCRC(buf):
        """ Calculates a valid CRC value to be used in order to check the integrity of the contents of a request packet. """
        shift_reg = 0
        z = buf.len - 3
        for i in range(2, z):
            v = buf[i]
            for j in range(0, 8):
                # isolate the least sign bit
                data_bit = v & 0x01
                sr_lsb = shift_reg & 0x01
                # calculate the feedback bit
                fb_bit = data_bit ^ sr_lsb
                shift_reg = shift_reg >> 1
                if fb_bit == 1:
                    shift_reg = shift_reg ^ 0x8c
                v = v >> 1

        return shift_reg

    @staticmethod
    def motorSensorRequest(packetNumber):
        cmd = bytes([STX0, STX1, 1, 0, GET_MOTOR_SENSOR_DATA, 1, packetNumber, calcCRC(cmd), ETX0, ETX1])
        return cmd

    @staticmethod
    def standardSensorRequest(packetNumber):
        cmd = bytes([STX0, STX1, 1, 0, GET_MOTOR_SENSOR_DATA, 1, packetNumber, calcCRC(cmd), ETX0, ETX1])
        return cmd

    @staticmethod
    def customSensorRequest(packetNumber):
        cmd = bytes([STX0, STX1, 1, 0, GET_CUSTOM_SENSOR_DATA, 1, packetNumber, calcCRC(cmd), ETX0, ETX1])
        return cmd

    @staticmethod
    def allSensorRequest(packetNumber):
        cmd = bytes([STX0, STX1, 1, 0, GET_ALL_SENSOR_DATA, 1, packetNumber, calcCRC(cmd), ETX0, ETX1])
        return cmd

    @staticmethod
    def enableMotorSensorSending():
        cmd = bytes([STX0, STX1, 1, 0, GET_MOTOR_SENSOR_DATA, 0, calcCRC(cmd), ETX0, ETX1])
        return cmd

    def enableStandardSensorSending():
        cmd = bytes([STX0, STX1, 1, 0, GET_STANDARD_SENSOR_DATA, 0, calcCRC(cmd), ETX0, ETX1])
        return cmd

    def enableCustomSensorSending():
        cmd = bytes([STX0, STX1, 1, 0, GET_CUSTOM_SENSOR_DATA, 0, calcCRC(cmd), ETX0, ETX1])
        return cmd

    def enableAllSensorSending():
        cmd = bytes([STX0, STX1, 1, 0, GET_ALL_SENSOR_DATA, 0, calcCRC(cmd), ETX0, ETX1])
        return cmd

    def disableMotorSensorSending():
        cmd = bytes([STX0, STX1, 1, 0, GET_ALL_SENSOR_DATA, 0, calcCRC(cmd), ETX0, ETX1])
        return cmd

    def disableStandardSensorSending():
        cmd = bytes([STX0, STX1, 1, 0, GET_STANDARD_SENSOR_DATA, 1, 0, calcCRC(cmd), ETX0, ETX1])
        return cmd

    def disableCustomSensorSending():
        cmd = bytes([STX0, STX1, 1, 0, GET_CUSTOM_SENSOR_DATA, 1, 0, calcCRC(cmd), ETX0, ETX1])
        return cmd

    def disableAllSensorSending():
        cmd = bytes([STX0, STX1, 1, 0, GET_ALL_SENSOR_DATA, 1, 0, calcCRC(cmd), ETX0, ETX1])
        return cmd

    def setMotorSensingPeriod():
        cmd = bytes([])
        return cmd

    def setStandardSensingPeriod():
        cmd = bytes([])
        return cmd

    def setStandardSensingPeriod():
        cmd = bytes([])
        return cmd

    def setCustomSensingPeriod():
        cmd = bytes([])
        return cmd

    def setAllSensingPeriod():
        cmd = bytes([])
        return cmd

    def getSensorSonar(channel, standardSensorAry):
        return standardSensorAry[channel + ULTRASONIC_OFFSET])

    def getSensorIRRange(channel, standardSensorAry, customSensorAry):
        if 0 <= channel < 1:
            return standardSensorAry[STANDARD_IR_RANGE_OFFSET + 1] << 8 | standardSensorAry[STANDARD_IR_RANGE_OFFSET]
        else:
            return customSensorAry[2 * (channel - 1) + CUSTOM_IR_RANGE_OFFSET + 1] << 8 | customSensorAry[2 * (channel - 1) + CUSTOM_IR_RANGE_OFFSET]

    def getSensorHumanAlarm(channel, standardSensorAry):
        offset = 2 * channel + HUMAN_ALARM_OFFSET
        return standardSensorAry[offset + 1] << 8 | standardSensorAry[offset]

    def getSensorHumanMotion(channel, standardSensorAry):
        offset = 2 * channel + HUMAN_MOTION_OFFSET
        return standardSensorAry[offset + 1] << 8 | standardSensorAry[offset]

    def getSensorTiltingX(standardSensorAry):
        return standardSensorAry[TILTING_X_OFFSET + 1] << 8 | standardSensorAry[TILTING_X_OFFSET]

    def getSensorTiltingY(standardSensorAry):
        return standardSensorAry[TILTING_Y_OFFSET + 1] << 8 | standardSensorAry[TILTING_Y_OFFSET]

    def getSensorOverheat(channel, standardSensorAry):
        offset = 2 * channel + OVERHEAT_SENSOR_OFFSET
        return standardSensorAry[offset + 1] << 8 | standardSensorAry[offset]

