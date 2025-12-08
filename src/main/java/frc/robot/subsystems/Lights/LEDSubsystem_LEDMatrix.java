package frc.robot.subsystems.Lights;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

//Unit Imports
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj.SerialPort;

//Serial Port and LED Imports
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;

public class LEDSubsystem_LEDMatrix extends SubsystemBase {

  //SETUP
  private SerialPort serialPort;
  private enum States {
    OFF,
    DEFAULT,
    LOADED
  };
  private States currentState = States.OFF;
  private boolean stateChanged = false;
  String matrixText = "FRC 3161";

  public LEDSubsystem_LEDMatrix() {
    // Serial Port Startup
    try {
        // Initialize MXP UART at 115200 baud, 8N1
        serialPort = new SerialPort(115200, SerialPort.Port.kMXP);
        serialPort.setTimeout(0.1); // 100ms timeout
        serialPort.setReadBufferSize(128); // Buffer size
        System.out.println("MXP UART initialized");
    } catch (Exception e) {
        System.out.println("MXP UART init failed: " + e.getMessage());
    }
    setState(States.DEFAULT);
  }

  // COMMANDS
  public void setState(States state) {
    currentState = state;
    switch (currentState) {
      case OFF:
        sendCommand("off");
        break;
      case DEFAULT:
        sendCommand("brightness 16")
        sendCommand("numbers");
        break;
      case LOADED:
        sendCommand("brightness 32")
        sendCommand("text Coral Loaded!");
        break;
      default:
        sendCommand("off");
        break;
    }
  }

  @Override
  public void periodic() {
    // State Switch
    // switch (currentState) {
    //   case OFF:
    //     break;
    //   case DEFAULT:
    //     break;
    //   case LOADED:
    //     break;
    //   default:
    //     break;
    // }

    // Serial Port Read
    if (serialPort.getBytesReceived() > 0) {
      String received = serialPort.readString();
      System.out.println("RECEIVED [LEDMatrix | UART]: " + received.trim()); // Log to Driver Station
    }
  }

  public void sendCommand(String command) {
    serialPort.writeString(command + "\n");
    System.out.println("SENT [LEDMatrix | UART]: " + command.trim()); // Log to Driver Station
  }
}