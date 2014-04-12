package com.edn.albert;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;


public class Albert {
	
	public static final int LED_PATTERN_DEFAULT = 0;
	public static final int LED_PATTERN_HAPPY = 1;
	public static final int LED_PATTERN_PROBLEM = 2;
	public static final int LED_PATTERN_INPUT = 6;

	public static void main(String[] args) {
		AlbertController controller = new AlbertController();
		controller.start();
	}
}

class AlbertController {
	
	private static final int DEFAULT_MOTOR_SPEED = 300;
	private static final int DEFAULT_MOTOR_ACCELERATION = 800;
	
	private RegulatedMotor leftMotor = Motor.B;
	private RegulatedMotor rightMotor = Motor.C;
	private IRSensorThread sensorThread;
	
	public void start() {
		
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		leftMotor.rotateTo(0);
		rightMotor.rotateTo(0);
		leftMotor.setSpeed(DEFAULT_MOTOR_SPEED);
		rightMotor.setSpeed(DEFAULT_MOTOR_SPEED);
		leftMotor.setAcceleration(DEFAULT_MOTOR_ACCELERATION);
		rightMotor.setAcceleration(DEFAULT_MOTOR_ACCELERATION);
		
		sensorThread = new IRSensorThread();
		sensorThread.setDaemon(true);
		sensorThread.start();
		
		Behavior[] behaviorList = {
			new DriveForward(this, leftMotor, rightMotor),
			new AvoidWall(this, sensorThread),
			new ProgramControl(this)
		};
		
		Arbitrator arbitrator = new Arbitrator(behaviorList);
		
		LCD.drawString("Albert 0.1",0,1);
		LCD.drawString("Push to Start",0,2);
		Button.LEDPattern(Albert.LED_PATTERN_INPUT);
		Button.waitForAnyPress();
		
		endAction();
		arbitrator.start();
	}
	
	public void endAction() {
		leftMotor.stop(true); 
		rightMotor.stop(true);
		Button.LEDPattern(Albert.LED_PATTERN_DEFAULT);
		LCD.clear();
	}
	
	public void exit() {
		endAction();
		System.exit(1);
	}
}

class IRSensorThread extends Thread {
	
	private EV3IRSensor ir = new EV3IRSensor(SensorPort.S2);
	private int distance = 255;
	private SampleProvider average = new MeanFilter(ir, 5);
	
	public void run() {
		
		while (true) {
			float [] sample = new float[ir.sampleSize()];
			average.fetchSample(sample, 0);
			distance = (int)sample[0];
			System.out.println("Control: Distance: " + distance);
		}
		
	}
	
	public int getDistance() {
		return distance;
	}
	
}

class DriveForward implements Behavior {

	private boolean suppressed = false;
	private RegulatedMotor leftMotor;
	private RegulatedMotor rightMotor;
	private AlbertController controller;
	
	public DriveForward(AlbertController controller, RegulatedMotor leftMotor, RegulatedMotor rightMotor) {
		this.controller = controller;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	public boolean takeControl() {
		return true;  // DriveForward always wants control
	}

	public void suppress() {
		suppressed = true; // DriveForward supressed
	}

	public void action() {
		suppressed = false;
		
		Button.LEDPattern(Albert.LED_PATTERN_HAPPY);
		LCD.drawString("DriveForward",0,1);
		
		leftMotor.setSpeed(400);
		rightMotor.setSpeed(400);
		leftMotor.forward();
		rightMotor.forward();
		
		while (!suppressed) {
			Thread.yield();
		}

		controller.endAction();
	}
}


class AvoidWall implements Behavior {

	private boolean supressed;
	private IRSensorThread sensorThread;
	private AlbertController controller;
	
	public AvoidWall(AlbertController controller, IRSensorThread sensorThread) {
		this.controller = controller;
		this.sensorThread = sensorThread;
	}
	
	private boolean isWallDetected() {

		if (sensorThread.getDistance() < 30) {
			return true;
		}
		else {
			return false;
		}
	}

	public boolean takeControl() {
		return isWallDetected();
	}

	public void suppress() {
		supressed = true;
	}

	public void action() {
		
		LCD.drawString("AvoidWall",0,1);
		Button.LEDPattern(Albert.LED_PATTERN_PROBLEM);
		
		while(isWallDetected() && !supressed) {
			Thread.yield();
		}
		
		controller.endAction();
		
		//      EV3BumperCar.leftMotor.rotate(-180, true);// start Motor.A rotating backward
		//      EV3BumperCar.rightMotor.rotate(-180);  // rotate C farther to make the turn
		//    if ((System.currentTimeMillis() & 0x1) != 0)
		//    {
		//        EV3BumperCar.leftMotor.rotate(-180, true);// start Motor.A rotating backward
		//        EV3BumperCar.rightMotor.rotate(180);  // rotate C farther to make the turn
		//    }
		//    else
		//    {
		//        EV3BumperCar.rightMotor.rotate(-180, true);// start Motor.A rotating backward
		//        EV3BumperCar.leftMotor.rotate(180);  // rotate C farther to make the turn        
		//    }


	}

}

class ProgramControl implements Behavior {

	private AlbertController controller;
	
	public ProgramControl(AlbertController controller) {
		this.controller = controller;
	}
	
	@Override
	public boolean takeControl() {
		
		if (Button.readButtons() != 0) {      
			return true;
		}
		
		return false;
	}

	@Override
	public void action() {
		
		LCD.drawString("Program Control",0,1);
		LCD.drawString("Enter: Continue",0,2);
		LCD.drawString("Back: Quit",0,3);
		Button.LEDPattern(Albert.LED_PATTERN_INPUT);
		
		Button.discardEvents();
		System.out.println("Button pressed");
		if ((Button.waitForAnyPress() & Button.ID_ESCAPE) != 0) {
			controller.exit();
		}
		System.out.println("Button pressed 2");
		Button.waitForAnyEvent();
		System.out.println("Button released");
		
		controller.endAction();
	}

	@Override
	public void suppress() {}
	
}


