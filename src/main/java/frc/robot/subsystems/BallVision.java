// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallVision extends SubsystemBase {
    //NetworkTableInstance inst = NetworkTableInstance.getDefault();
    //NetworkTable table = inst.getTable("datatable");    
  public BallVision() {}

  public void getCamera() {
    new Thread(() -> {
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);

        CvSink cvSink = CameraServer.getInstance().getVideo();
        CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

        Mat source = new Mat();
        Mat output = new Mat();

        while(!Thread.interrupted()) {
            cvSink.grabFrame(source);
            Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
            outputStream.putFrame(output);
        }
    }).start();
  }

  /* public void sendColor()
  {
      Alliance color = DriverStation.getAlliance();
      String colorString;
      if(color == Alliance.Red)
      {
        colorString = "Red";
      }
      else if(color == Alliance.Blue)
      {
        colorString = "Blue";
      }
      else
      {
        colorString = "Can't find color";
      }
      table.getEntry("color").setString(colorString);
  }
  public double getCenter()
  {
    return table.getEntry("center").getDouble(-1);
  } */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
