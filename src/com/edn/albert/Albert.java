package com.edn.albert;

import java.util.Random;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
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
	private SensorThread sensorThread;
	
	public void start() {
		
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		leftMotor.rotateTo(0);
		rightMotor.rotateTo(0);
		
		sensorThread = new SensorThread();
		sensorThread.setDaemon(true);
		sensorThread.start();
		
		Behavior[] behaviorList = {
			new DriveForward(this, leftMotor, rightMotor),
			new AvoidWall(this, sensorThread, leftMotor, rightMotor),
			new ProgramControl()
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
		leftMotor.setSpeed(DEFAULT_MOTOR_SPEED);
		rightMotor.setSpeed(DEFAULT_MOTOR_SPEED);
		leftMotor.setAcceleration(DEFAULT_MOTOR_ACCELERATION);
		rightMotor.setAcceleration(DEFAULT_MOTOR_ACCELERATION);
		leftMotor.stop(true); 
		rightMotor.stop(true);
		Button.LEDPattern(Albert.LED_PATTERN_DEFAULT);
		LCD.clear();
	}
	
}

class SensorThread extends Thread {
	
	private EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S4);
	private boolean touchActivated = false;
	
	private EV3IRSensor ir = new EV3IRSensor(SensorPort.S2);
	private int infaredDistance = 255;
	private SampleProvider infaredAverage = new MeanFilter(ir, 5);
	
	public void run() {
		
		while (true) {
			float [] sample = new float[ir.sampleSize()];
			infaredAverage.fetchSample(sample, 0);
			infaredDistance = (int)sample[0];
			
			sample = new float[touch.sampleSize()];
			touch.fetchSample(sample, 0);
			
			touchActivated = sample[0] > 0;
			System.out.println("Control: Distance: " + infaredDistance+" touch "+touchActivated);
			
			try {
				Thread.sleep(150);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
	}
	
	public int getIRDistance() {
		return infaredDistance;
	}
	
	public boolean isTouched() {
		return touchActivated;
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
		return true;  //always wants control
	}

	public void suppress() {
		suppressed = true;
	}

	public void action() {
		suppressed = false;
		
		Button.LEDPattern(Albert.LED_PATTERN_HAPPY);
		LCD.drawString("DriveForward",0,1);
		
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
	private SensorThread sensorThread;
	private AlbertController controller;
	private RegulatedMotor leftMotor, rightMotor;
	
	private static final int WALL_DISTANCE = 40;
	
	public AvoidWall(AlbertController controller, SensorThread sensorThread, RegulatedMotor leftMotor, RegulatedMotor rightMotor) {
		this.controller = controller;
		this.sensorThread = sensorThread;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	private boolean isWallDetected() {

		if (sensorThread.getIRDistance() < WALL_DISTANCE || sensorThread.isTouched()) {
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
		
		//TODO make async and check supressed
		
		LCD.drawString("AvoidWall",0,1);
		Button.LEDPattern(Albert.LED_PATTERN_PROBLEM);
		
		if(sensorThread.isTouched()) {
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(100);
			leftMotor.rotate(-360);
			rightMotor.rotate(-360);
		}
		
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.rotate(360, true);
		rightMotor.rotate(-360);
		
		controller.endAction();
	}

}

class ProgramControl implements Behavior {
	
	//TODO add total program time
	
	@Override
	public boolean takeControl() {
		
		if (Button.readButtons() == Button.ID_ESCAPE) {      
			return true;
		}
		
		return false;
	}

	@Override
	public void action() {
		System.exit(1);
	}

	@Override
	public void suppress() {}
	
}


