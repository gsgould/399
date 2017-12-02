package newproject;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;

public class part6Bot {
	
	// === Network Variables
	// Change this to match port on PC
	public static int port = 1111; 
	
	// === Define Motors here
	public static UnregulatedMotor UR = new UnregulatedMotor(MotorPort.D);
	public static UnregulatedMotor UL = new UnregulatedMotor(MotorPort.A);
	
	public static void main(String[] args) {
		//Attempt to connect to PC
		try {
			ServerSocket serv = new ServerSocket(port);
			System.out.println("Waiting for connection...");
			Socket s = serv.accept(); //Wait for Lap top to connect
			System.out.println("Connected!");
			DataInputStream in = new DataInputStream(s.getInputStream());
			DataOutputStream out = new DataOutputStream(s.getOutputStream());
		
			// == Variables for Controller
			String action = "";
			String actionPrev = "";
			int speedL = 0;
			int speedR = 0;
		
			boolean f = false;
			boolean b = false;
			boolean r = false;
			boolean l = false;
			
			while(Button.getButtons()==0) {
				
				action = in.readUTF();
				System.out.println(action);
				
				if (action != actionPrev) {
					// == Write robot responses to actions given by PC
					// These are the command when pushed and released defined earlier in part6PC.java
					
					//Go Forward (W is pushed)
					if(action.equals("1") && f == false) {
						speedL += 75;
						speedR += 75;
						f = true;
					}else if(action.equals("2")){
						speedL += -75;
						speedR += -75;
						f = false;
					}
					//Go Backwards (S is pushed)
					if(action.equals("3") && b == false) {				
						speedL += -50;
						speedR += -50;
						b = true;
					}else if(action.equals("4")){
						speedL += 50;
						speedR += 50;
						b = false;
					}
					//Go Left (A is pushed)
						// The booleans used here are to ensure that pushing
						// Left and right won't mess up the movement.
					if(action.equals("5") && l == false && r == false) {
						speedL += -50;
						speedR += 50;
						l = true;
					}else if(action.equals("6") && l == true && r == false){
						speedL += 50;
						speedR += -50;
						l = false;
					}
					//Go Right (D is pressed)
					if(action.equals("7") && r == false && l == false) {
						speedL += 50;
						speedR += -50;
						r = true;
					}else if(action.equals("8") && r == true && l == false){
						speedL += -50;
						speedR += 50;
						r = false;
					}
					//Disconnect robot
					if(action.equals("9")) {
						break;
					}
					//Reset robot, in case the movement code causes a bug or glitchy movement
					//This will reset the variables and speed.
					if(action.equals("10")) {
						speedL = 0;
						speedR = 0;

						f = false;
						b = false;
						r = false;
						l = false;
					}
					//Action buffer to prevent repeated presses adding more speed than intended
					actionPrev = action;
				}
				//Set motor power
				UL.setPower(speedL);
				UR.setPower(speedR);

				UL.forward();
				UR.forward();
			}
			//Close connection
			serv.close();
			out.close();
		}catch(IOException ioe){
			System.out.println("Error in connection");
		}
	}
}
