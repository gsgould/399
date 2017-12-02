//package newproject;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import lejos.hardware.Button;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;

public class teleop {
	
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
			
			int fwd = 0; 
			int turn = 0;
		
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
						fwd = 1;
						f = true;
					}else if(action.equals("2")){
						fwd = 0;
						f = false;
					}
					//Go Backwards (S is pushed)
					if(action.equals("3") && b == false) {				
						fwd = -1;
						b = true;
					}else if(action.equals("4")){
						fwd = 0;
						b = false;
					}
					//Go Left (A is pushed)
						// The booleans used here are to ensure that pushing
						// Left and right won't mess up the movement.
					if(action.equals("5") && l == false && r == false) {
						turn = 1;
						l = true;
					}else if(action.equals("6") && l == true && r == false){
						turn = 0;
						l = false;
					}
					//Go Right (D is pressed)
					if(action.equals("7") && r == false && l == false) {
						turn = -1;
						r = true;
					}else if(action.equals("8") && r == true && l == false){
						turn = 0;
						r = false;
					}

					
					actionPrev = action;
				}
				//Set motor power

				segway.PID( fwd, turn);
				
			}
			//Close connection
			serv.close();
			out.close();
		}catch(IOException ioe){
			System.out.println("Error in connection");
		}
	}
}
