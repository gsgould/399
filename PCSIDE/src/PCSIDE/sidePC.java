package PCSIDE;

import java.io.*;
import java.net.Socket;
import java.util.ArrayList;

import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import javax.swing.JFrame;

import javax.swing.*;
import javax.swing.JButton;

public class sidePC extends JFrame{
	
	// === Serial ID
	static final long serialVersionUID = 1;
	
	// === Network Variables
	// Change these for different default
	public static String ip = "10.0.1.1";
	public static int port = 1111;
	
	// === Global Variables
	public static ButtonHandler bh = new ButtonHandler();
	public static JLabel label, label2;
	public static JTextField ipF, portF;
	
	public static ArrayList<buttonSet> keys = new ArrayList<buttonSet>();
	
	public static Socket sock;
	public static DataOutputStream out;
	
	public static boolean connected = false;
	
	public static void main(String[] args) {
		
		// === Local variables
		JFrame frame = new JFrame("EV3 BT Controller"); 
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		frame.getContentPane().setLayout(new GridLayout(0,2));
		
		//=== Define Buttons
		// The syntax here is (Button Name, Key to listen to, 
		//command to send when pushed, command to send when released.)
		keys.add(new buttonSet("Forward", 'w', "1", "2"));
		keys.add(new buttonSet("Turn Left", 'a', "5", "6"));
		keys.add(new buttonSet("Backward", 's', "3", "4"));
		keys.add(new buttonSet("Turn Right", 'd', "7", "8"));
		keys.add(new buttonSet("Weapon Rotate", 't', "11", "12"));
		keys.add(new buttonSet("Weapon Rotate \nReverse", 'g', "13", "14"));
		keys.add(new buttonSet("Reset", 'r', "10", "0"));
		keys.add(new buttonSet("Disconnect", 'q', "9", "0"));
		
		//=== Add Listeners to all Buttons
		for (buttonSet k : keys) {
			k.getButton().addMouseListener(bh);
			k.getButton().addKeyListener(bh);
			frame.add(k.getButton());
		}

		
		// === Swing Implementation
		ipF = new JTextField(ip);
		portF = new JTextField(Integer.toString(port));
		
		label = new JLabel("Close me when done.",SwingConstants.CENTER);
		label2 = new JLabel("Connecting...",SwingConstants.CENTER);
		label.setPreferredSize(new Dimension(150, 50));
		
		frame.add(ipF);
		frame.add(portF);
		
		frame.getContentPane().add(label, BorderLayout.CENTER);
		frame.getContentPane().add(label2, BorderLayout.SOUTH);
		
		frame.setLocationRelativeTo(null); 
		frame.pack(); 
		frame.setVisible(true); 
		
		// === Try connecting until you connect successfully.
		while(connected==false)
			//Try connecting until successful
			try {
				ip = ipF.getText();
				port = Integer.parseInt(portF.getText());
				sock = new Socket(ip, port);
				out = new DataOutputStream(sock.getOutputStream());
				//out.writeUTF("Hello EV3!");
				label2.setText("Connected");
				connected = true;
			}catch(IOException ioe){
				System.out.println("Error in connection");
				label2.setText("Connection Error");
			
				
				label2.setText("Retrying");
			}

		
	}

	
	// Stuff derived from the example given
	public static class ButtonHandler implements MouseListener,KeyListener{
		
		//Mouse actions
		public void mouseClicked(MouseEvent arg0) {}

		public void mouseEntered(MouseEvent arg0) {}

		public void mouseExited(MouseEvent arg0) {}

		public void mousePressed(MouseEvent moe) {
			//Check all buttons for a mousePressed action.
			try {
	        	for (buttonSet b : keys) {
	        		if (moe.getSource() == b.getButton()){
	        			out.writeUTF(b.getPush());
	        		}
	        	}
	         }
	         catch (IOException ioe) {
	            System.out.println("Not yet connected!");
	         }
	        
	   }//End mousePressed

	   public void mouseReleased(MouseEvent moe) {
		   //Check all buttons for a mouseReleased action.
		   try {
		   		for (buttonSet b : keys) {
	        		if (moe.getSource() == b.getButton()){
	        			out.writeUTF(b.getRelease());
	        			if (b.getLetter() == 'q') {System.exit(0);}
	        		}
	        	}
	        }
	        catch (IOException ioe) {
	           System.out.println("Not yet connected!");
	        }
	      
	   }//End mouseReleased

		public void keyPressed(KeyEvent ke) {
			
		}//End keyPressed

		public void keyTyped(KeyEvent ke){
			//Check keyboard for keyTyped action.
			try {
				for (buttonSet b : keys) {
	        		if (ke.getKeyChar() == b.getLetter()){
	        			out.writeUTF(b.getPush());
	        		}
	        	} 
			}catch (IOException ioe) {
	            System.out.println("Not yet connected!");
	        }
		}//End keyTyped

		public void keyReleased(KeyEvent ke){
			//Check keyboard for keyReleased action.
			try {
				for (buttonSet b : keys) {
	        		if (ke.getKeyChar() == b.getLetter()){
	        			out.writeUTF(b.getRelease());
	        			if(b.getLetter() == 'q'){out.writeUTF("9");System.exit(0);}
	        		}	        		
	        	}
				
			}catch (IOException ioe) {
	            System.out.println("Not yet connected!");
	        }
		}//End keyReleased
		
		

	}//end ButtonHandler class
}



