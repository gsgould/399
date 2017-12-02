package com.AJD1.bluetoothrc;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;

//To be run on the EV3
public class BluetoothRCServer {
	public static void main(String args[]) throws IOException {
		
		//Creating various resources to be used in the class.
		int input;
		ServerSocket server = new ServerSocket(1111);
		EV3LargeRegulatedMotor motorA = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor motorB = new EV3LargeRegulatedMotor(MotorPort.D);
		
		//Setting up the IsEscapeDownChecker thread, and passing it the ServerSocket to be used.
		IsEscapeDownChecker isEscapeDown = new IsEscapeDownChecker(server);
		isEscapeDown.setDaemon(true);
		isEscapeDown.start();
		
		//Main loop of the program. The EV3 checks incoming data from the socket in the form of integers. 
		//Depending on the incoming integer the EV3 executes a action.
		while (true) {
			Socket socket;
			try {
				socket = server.accept();
			} catch (IOException e) {
				break;
			}
			DataInputStream in = new DataInputStream(socket.getInputStream());
			input = in.readInt();
            
            //forward
			if (input == 1) {
				motorA.forward();
				motorD.forward();
			} 
			
            //backwards
			if (input == 2) {
				motorA.backward();
				motorD.backward();
			}
			
            // Turn
			if (input == 3) {
				motorA.backward();
				motorB.forward();
			}
			
            // Turn
			if (input == 4) {
				motorA.forward();
				motorB.backward();
			}
			
            // Stop
			if (input == 5) {
				motorA.stop();
				motorD.stop();
			}
            
			// End
			if (input == 6) {
				Sound.setVolume(100);
				Sound.buzz();
				server.close();
				motorA.close();
				motorD.close();
				System.exit(0);
			}
			
			if (input == 7) {
				Sound.setVolume(100);
				Sound.beep();
			}

		}
		
		//If IsEscapeDown instance of IsEscapeDownChecker closes the socket connection, an error occurs and
		//the main program loop is broken out of to arrive at this point. Thus ending the program.
		Sound.setVolume(100);
		Sound.buzz();
		server.close();
		motorA.close();
		motorD.close();
		System.exit(0);
	}
}
