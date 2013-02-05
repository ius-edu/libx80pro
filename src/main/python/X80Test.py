def main():
    robot = X80Pro("192.168.0.203")

    sys.stderr.write("Reset Head")
    robot.resetHead()

    sys.stderr.write("Resume Both DC Motors")
    robot.resumeBothDCMotors()

    sys.stderr.write("Move the robots")
    robot.setBothDCMotorVelocities(3000, 3000)

    sys.stderr.write("Hold the move for 3 seconds")
    sleep(3);

    sys.stderr.write("Suspend Both DC Motors")
    robot.suspendBothDCMotors()
