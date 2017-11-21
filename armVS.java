import java.util.Arrays;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;
import lejos.utility.Matrix;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;




public class armVS {
    public static UnregulatedMotor firstLink = new UnregulatedMotor(MotorPort.A);
    public static UnregulatedMotor secondLink = new UnregulatedMotor(MotorPort.B);
        
    public static TrackerReader tracker;
    
    public static int gRatio1 = 7;
    public static int gRatio2 = 5; // ratio between the driver gear and follower gear of 2nd link
    
    public static double[] LINKS = new double[]{ 8.83, 10}; //8, 11.2}; // length of link1, link2 in cm
    public static int[] TACH = new int[2]; // current angles of the links
    public static int[] domain1 = new int[]{ 0, 180};
    public static int[] domain2 = new int[]{ -45, 135};
    		
 
    public static double[] getAngle(){
    	double[] outpt = new double[]{
    			TACH[0] / gRatio1,
    			-TACH[1] / gRatio2
    	};
    	return outpt;
    }

    public static void goToAngle(double angle) {
        
        
        firstLink.setPower(25); 
        firstLink.forward();
        double error = 1; 
        double newError = 1;
        double damper; 
        double currentAngle;
        int power = 50;
        double k = 1;
        int confirm = 1;
        

            
            currentAngle = firstLink.getTachoCount();
            error = angle - currentAngle;
            
            while (error != 0) {
                currentAngle = firstLink.getTachoCount();
                error = angle - currentAngle;
                power = (int)( error * k);
                firstLink.setPower(power); 
                System.out.println("Error: " + error);

            }
            
            System.out.println("Broke Loop");
            
            
        
        
        
    }
    
    
    public static double[] threeDec( double d[]){
    	int temp;
    	temp = (int) (d[0] * 1000);
    	d[0] = (double) temp / 1000;
    	temp = (int) (d[1] * 1000);
    	d[1] = (double) temp / 1000;
    	return d;
    	
    }
    public static void PID( int setAngle1, int setAngle2){
    	double kp = 1.5; // proportional
        double ki = 0; // integral
        double kd = 10; // derivative
        
        if( setAngle1 < domain1[0] || setAngle1 > domain1[1]){
        	System.out.println("invalid angles");
        	return;
        } else if( setAngle2 > -domain2[0] || setAngle2 < -domain2[1]){
        	System.out.println("invalid angles");
        	return;
        }
        
        // convert the set angles to account for gear ratio
        int setTheta1 = setAngle1 * gRatio1;
        int setTheta2 = -setAngle2 * gRatio2;
        

        
        double prev_error1 = 0;
        double prev_error2 = 0;
        double error1 = 100;
        double error2 = 100;
        double integral1 = 0;
        double integral2 = 0;
        double derivative1;
        double derivative2;
        double dt = 10;
        
        int output1;
        int output2;
        
        // variables to measure error
        int E1;
        int E2;
        
        
      
        
        
        while( true){
            TACH[0] = firstLink.getTachoCount();
            TACH[1] = secondLink.getTachoCount();
            
            E1 = Math.abs( TACH[0] - setTheta1);
            E2 = Math.abs( TACH[1] - setTheta2);
            
            
            if( 2 > E1 && 2 > E2) {
                break;
            }
            
            error1 = setTheta1- TACH[0];
            integral1 = integral1 + ( error1 * dt);
            derivative1 = ( error1 - prev_error1) / dt;
            output1 = (int) (kp * error1 + ki * integral1 + kd * derivative1);
            prev_error1 = error1;
            
            error2 = setTheta2 - TACH[1];
            integral2 = integral2 + ( error2 * dt);
            derivative2 = ( error2 - prev_error2) / dt;
            output2 = (int) (kp * error2 + ki * integral2 + kd * derivative2);
            prev_error2 = error2;
            
            
            
            
            firstLink.setPower( pow( output1));
            secondLink.setPower( pow( output2));
            firstLink.forward();
            secondLink.forward();
            
            Delay.msDelay( (int) dt);
            
            
        }
        firstLink.stop();
        secondLink.stop();
    }

    public static int[] invKin( double[] pos, int mode){
    	// mode 0: numerical, 1: analytical
    	
    	
    	double[] theta0 = new double[]{ 0, 45};
    	if( pos[0] > 0){
    		// position in first quadrant
    		theta0[0] = 25;
    	} else {
    		// position in second quadrant
    		theta0[0] = 80;
    	}
    	int[] th = kinematics.InvKin( pos, LINKS, theta0, mode);
    	
    	// check if th gets correct pos
    	// check that th1 th2 are in the domain
    	
    	double[] th_ = new double[]{ (double) th[0], (double) th[1]};
    	double[] calcPos = kinematics.fwdKinPos(th_, LINKS);
    	
    	
    	int temp;
    	
    	if( mode == 0){
    		// correct for numerical method
    		// check domain for link1
        	if( th_[0] < domain1[0]){ // if th1 less than domain add 180 
    			th_[0] = th_[0] + 360;
    			
    		} else if( th_[0] > domain1[1]){ // if th1 greater than domain minus 180
    			th_[0] = th_[0] - 360 ;
    		}
        	
        	// check domain for link2
        	
        	if( th_[1] < domain2[0]){
    			th_[1] = th_[1] + 360;
        	} else if ( th_[1] > domain2[1]){
        		th_[1] = th_[1] - 360;
        	}
        	
        	
    	} else {
    		// correct the analytical solution to the correct quadrant 
    		if( kinematics.euclidDist(pos, calcPos) > 1){ //if calculated position != to actual position
    			  
        		// check domains
        		if( th_[0] < domain1[0]){ // if th1 less than domain add 180 
        			th_[0] = th_[0] + 180;
        			
        		} else if( th_[0] > domain1[1]){ // if th1 greater than domain minus 180
        			th_[0] = th_[0] - 180;
        			
        		} else if (th_[0] == 0) { // last case is if the actual is 180 and the calculated is 0
        			th_[0] = th_[0] + 180;
        			
        		}
        		
        	}
    		
    	
    		
    		// if th2 is not in domain then switch to the other possible position
    		if( -th_[1] < domain2[0] || -th_[1] > domain2[1]){ 
    			th_[1] = -th_[1];
    			temp = 90 - Math.abs((int)th_[0]);
    			th_[0] = th_[0] + temp;
    				

    		} 
    	}
    	    	
    	
    	
    	
    	
		
		
		calcPos = kinematics.fwdKinPos(th_, LINKS);
    	
    	
    	
    	
    	
    	// print actual and calculated positions
    	int ap1 = (int) (pos[0] * 1000);
    	pos[0] = (double) ap1 / 1000;
    	int ap2 = (int) (pos[1] * 1000);
    	pos[1] = (double) ap2 / 1000;
    	
    	int cp1 = (int) (calcPos[0] * 1000);
    	calcPos[0] = (double) cp1 / 1000;
    	int cp2 = (int) (calcPos[1] * 1000);
    	calcPos[1] = (double) cp2 / 1000;
    	
    	//System.out.println("act " + pos[0] + " " + pos[1]);
    	//System.out.println("calc " + calcPos[0] + " " + calcPos[1]);
    	
    	th[0] = (int)th_[0];
    	th[1] = (int)th_[1];
    	return th;
    }
    
    
    public static void PID_angle( double setAngle){
        secondLink.resetTachoCount();
        
        
        //fileWrite fw = new fileWrite();

        double kp = 4; // proportional
        double ki = 0; // integral
        double kd = 100; // derivative 

        double currentAngle;
        double prev_error = 0;
        double error = 1;
        double integral = 0;
        double derivative;
        double dt = 10;
        int output;
        
        long startTime = System.currentTimeMillis();
        long currentTime;
       
        
        
        while( true){
            currentAngle = secondLink.getTachoCount();
            if( prev_error == 0 && error == 0){
                break;
            }
            error = setAngle - currentAngle;
            integral = integral + ( error * dt);
            derivative = ( error - prev_error) / dt;
            output = (int) (kp * error + ki * integral + kd * derivative);
            prev_error = error;
            
            currentTime = System.currentTimeMillis() - startTime;
            
            
            System.out.println( error);
            secondLink.setPower( pow( output));
            secondLink.forward();
            Delay.msDelay( (int) dt);
            //fw.append(Double.toString( error) + " " + Long.toString( currentTime), Double.toString(kp) + "_" + Double.toString(kd));
            
        }
        secondLink.stop();



    }
    
    public static int pow( int power){
        // keep the power in range [-100, 100]
        if( power > 100){
            return 100;
        } else if( power < -100){
            return -100;
        } else{
            return power;
        }
    }
  
    public static double[] nextPoint( double currentP[], double targetP[]){
    	double l = 20; // minimum distance
    	double dx = targetP[0] - currentP[0];
    	double dy = targetP[1] - currentP[1];
    	
    	double dist = kinematics.euclidDist(currentP, targetP);
    	if( dist > l){
    		// change length to l
    		dx = dx / dist * l;
    		dy = dy / dist * l;
    	} 
    	
    	
    	double outpt[] = new double[]{ currentP[0] + dx , currentP[1] + dy};
    	return outpt;
    }
    
    public static double[][] estimateJacobian(){
    	// orthogonal motions
    	// get initial thetas, positions
    	double theta[] =  getAngle();
    	double dtheta[] = new double[2];
    	double pos[] = new double[]{ tracker.x, tracker.y};
    	double dpos[] = new double[2];
    	double jacobian[][] = new double[2][2];
    	double dt = 5; // change in the theta
    	
    	// change th1 and get du, dv
    	dtheta[0] = theta[0] + dt;
    	dtheta[1] = theta[1];
    	
    	PID( (int)dtheta[0], (int)dtheta[1]);
    	dpos[0] = tracker.x - pos[0];
    	dpos[1] = tracker.y - pos[1];
    	
    	jacobian[0][0] = dpos[0];
    	jacobian[0][1] = dpos[1];
    	
    	// change th2 and get du, dv
    	dtheta[0] = theta[0];
    	dtheta[1] = theta[1] + dt;
    	
    	PID( (int)dtheta[0], (int)dtheta[1]);
    	dpos[0] = tracker.x - pos[0];
    	dpos[1] = tracker.y - pos[1];
    	
    	jacobian[1][0] = dpos[0];
    	jacobian[1][1] = dpos[1];
    	
    	PID( (int)pos[0], (int)pos[1]);
    	
    	return jacobian;
	
    }
    
    public static double[][] updateJacobian( double[][] J, double[] de, double[] dq){
    	/*
	     * Broyden
		 * Jk+1 = Jk + \alpha ( ( de - Jk * dq) dq') / ( dq' dq)
		 * 
		 *  e = [ u    q = [ th1     de = J * dq
	 	 *  	  v ]        th2 ]
		 *  
		 *  alpha = [0,1]
    	 */
    	double alpha = 1;
    	Matrix Jacob = new Matrix( J);
    	
    	// q and e are 1x2 and need to be 2x1
    	Matrix e_ = new Matrix( de, 1);
    	Matrix q_ = new Matrix( dq, 1);
    	Matrix e = e_.transpose();
    	Matrix q = q_.transpose();
    	
    	Matrix temp = e.minus( Jacob.times(q)).times(q.transpose());
    	double denom = dq[0] * dq[0] + dq[1] * dq[1];
    	temp = temp.times( alpha / denom );
    	
    	Jacob = Jacob.plus(temp);
    	
    	return Jacob.getArray();
    	
    	
    	
    			
    			
    }

    public static void VS(){

		// get initial positions
		double target[] = new double[]{ tracker.targetx, tracker.targety};
		double pos[] = new double[]{ tracker.x, tracker.y};
		double oldPos[] = new double[]{ tracker.x, tracker.y};
		
		double dPos[] = new double[2];
		double nxtP[] = new double[2];
		double theta[] = getAngle();
		double dTheta[] = new double[2];
		
		// initial jacobian
		double J[][] = estimateJacobian();
		double invJ[][] = new double[2][2]; 
		

		// loop until position is on target
		while( kinematics.euclidDist(target, pos) < 0.1 ){
			
			// update position
			pos[0] = tracker.x;
			pos[1] = tracker.y;
			
			nxtP = nextPoint( pos, target);
			dPos[0] = nxtP[0] - pos[0];
			dPos[1] = nxtP[1] - pos[1];
			
			invJ = kinematics.inv2x2Mat(J);
			//update thetas
			dTheta[0] = dPos[0] * invJ[0][0] + dPos[1] * invJ[0][1];
			dTheta[1] = dPos[0] * invJ[1][0] + dPos[1] * invJ[1][1];
			theta[0] = theta[0] + dTheta[0];
			theta[1] = theta[1] + dTheta[1];
			
			// move to new position
			PID( (int)theta[0], (int)theta[1]);
			
			// update jacobian
			J = updateJacobian( J, dPos, dTheta);
			
			
			
			
		}
		

		
		
	}
	
    
    
	/* algorithm
	 *  tx, ty = target position
	 *  u, v = current position
	 *  
	 *  calculate J_ 
	 *  
	 *  [ Du  = change in position
	 *    Dv ]
	 *  
	 *  update th
	 *  	thk+1 = thk + Dth
	 *  		  = thk + J_^-1[ Du
	 *  						 Dv ]
	 *  
	 * 
	 */
	
	
	/*
	 * [ x y]' = [ f1( th1, th2)
	 * 			   f2( th1, th2) ]
	 * 
	 * [ Dx  = [ df1/dth1 df1/dth2  [ Dth1
	 *   Dy ]    df2/dth1 df2/dth2 ]  Dth2 ]
	 *        
	 *               
	 * Visual servoing
	 * [ u   = f( th1, th2)
	 *   v ]              
	 *   
	 *   J_ = [ du/dth1 du/dth2
	 * 		   dv/dth1 dv/dth2 ]
	 * 
	 * [ Du   = J_ [ Dth1
	 *   Dv ]        Dth2 ]
	 *            
	 *              
	 * Broyden
	 * Jk+1 = Jk + \alpha ( ( de - Jk * dq) dq') / ( dq' dq)
	 * 
	 *  e = [ u    q = [ th1     de = J * dq
 	 *  	  v ]        th2 ]
	 *  
	 * 
	 */
    
    
    
    public static void testJUpdate(){
    	double J[][] = new double[][]{ 
    		{ 1, 0},
    		{ 0, 1}
    	};
    	
    	double dq[] = new double[]{ 1, 1};
    	double de[] = new double[]{ 0.1, 0.2};
    	
    	System.out.println( Arrays.toString(J[0]));
    	System.out.println( Arrays.toString(J[1]));
    	
    	J = updateJacobian( J, de, dq);
    	
    	System.out.println(" ");
    	System.out.println( Arrays.toString(J[0]));
    	System.out.println( Arrays.toString(J[1]));
    }
    
    public static void main( String[] args) {
		//tracker = new TrackerReader();
		//tracker.start();
		

        TACH[0] = firstLink.getTachoCount();
        TACH[1] = secondLink.getTachoCount();
        
        System.out.println("Lets Go!");
        
        //firstLink.resetTachoCount();
        //secondLink.resetTachoCount();
        
        testJUpdate();


        //double[] outpt = kinematics.fwdKinPos( getAngle(), LINKS);
        
        //int[] th = kinematics.InvKin(outpt, LINKS, getAngle(),1);
        
        //System.out.println(th[0]);
        //System.out.println(th[1]);
        
        //System.out.println("X: " + outpt[0]);
        //System.out.println("Y: " + outpt[1]);
        Delay.msDelay(4000);
   
    }

}