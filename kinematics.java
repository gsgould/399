import java.util.Arrays;


public class kinematics{

    public static double[][] jacob2D( double[] link, double[] theta){
        // link[] contains the length of the two links
        // theta[] contains the two angles
        double l1 = link[0];
        double l2 = link[1];
        double t1 = Math.toRadians( theta[0]);
        double t2 = Math.toRadians( theta[1]);

        /* jacobian
            [ df1(x)/ dx1  df1(x)/dx2
              df2(x)/ dx1  df2(x)/dx2 ]
        */
        double df1x1 = -l1 * Math.sin(t1) + -Math.sin(t1 + t2);
        double df1x2 = -l2 * Math.sin(t1 + t2);
        double df2x1 =  l1 * Math.cos(t1) + l2 * Math.cos( t1 + t2); 
        double df2x2 =  l2 * Math.cos(t1 + t2);

        double[][] jacob = new double[][]{
        	{ df1x1, df1x1},
        	{ df2x1, df2x2}
        };
        return jacob;
                                     
        

    }
    
    



    public static double[] fwdKinPos( double[] theta, double[] link){
    	
        double l1 = link[0]; 
        double l2 = link[1]; 
        double t1 = Math.toRadians( theta[0]);
        double t2 = Math.toRadians( theta[1]);

        double x = l1 * Math.cos( t1) + l2 * Math.cos( t1 + t2);
        double y = l1 * Math.sin( t1) + l2 * Math.sin( t1 + t2);
        
        double xy[] = new double[]{
        		x, y
        };
        
        return xy;

    }
    
    public static double[][] newtonJacob(  double[] theta, double[] link,  double alpha){
    	// estimate the jacobian
    	/*
    	double th11[] = new double[]{ theta[0] + alpha, theta[1]};
    	double th12[] = new double[]{ theta[0] - alpha, theta[1]};
    	double th21[] = new double[]{ theta[0], theta[1] + alpha};
    	double th22[] = new double[]{ theta[0], theta[1] - alpha};
    	
    	double P11[] = fwdKinPos( th11, link);
    	double P12[] = fwdKinPos( th12, link);
    	double P21[] = fwdKinPos( th21, link);
    	double P22[] = fwdKinPos( th22, link);
    	
    	double J11 = (P11[0] - P12[0]) / ( 2 * alpha);
    	double J12 = (P11[1] - P12[1]) / ( 2 * alpha);
    	double J21 = (P21[0] - P22[0]) / ( 2 * alpha);
    	double J22 = (P21[1] - P22[1]) / ( 2 * alpha);
    	
    	double J[][] = new double[][]{
    		{ J11, J21},
    		{ J12, J22}
    	};
    	*/
    	/*
    	 * J = [ dxth1 dxth2
    	 *  	 dyth1 dyth2]
    	 *  
    	 *  x = l1 cos(th1) + l2 cos( th1 + th2)
    	 *  y = l2 sin(th1) + l2 sin( th1 + th2)
    	 *  
    	 *  dxth1 = -l1 sin( th1) - l2 sin( th1 + th2)
    	 *  dyth1 =  l1 cos( th1) - l2 cos( th1 + th2)
    	 *  dxth2 = -l2 sin( th1 + th2)
    	 *  dyth2 =  l2 cos( th1 + th2) 
    	 * 
    	 */
    	double th1 = theta[0];
    	double th2 = theta[1];
    	double l1 = link[0];
    	double l2 = link[1];
    	
    	double dxth1 = -l1 * Math.sin( th1) - l2 * Math.sin( th1 + th2);
    	double dyth1 =  l1 * Math.cos( th1) - l2 * Math.cos( th1 + th2);
    	double dxth2 = -l2 * Math.sin( th1 + th2);
    	double dyth2 =  l2 * Math.cos( th1 + th2);
    	
    	double J[][] = new double[][]{
    		{ dxth1, dxth2},
    		{ dyth1, dyth2}
    	};
    	return J;
    	
    	
    	
    	
    }
    
    public static double[][] inv2x2Mat( double[][] m){
    	 /*
    	  * m = [ a b
    	  *       c d]
    	  * 
    	  * determinate = 1 / ( ad - bc)
    	  * 
    	  * inv m = det * [ d -b
    	  * 			   -c  a]
    	  */
    	
    	double a = m[0][0];
    	double b = m[0][1];
    	double c = m[1][0];
    	double d = m[1][1];
    	
    	double det = 1 /( a * d - b * c);
    	
    	double outpt[][] = new double[][]{
    		{ det * d, det * -b}, 
    		{ det * -c, det * a}
    	};
    	
    	
    	return outpt;
    	
    }
    
    
    

    public static int[] InvKin( double[] pos, double[] link, double[] theta0, int m){
        // pos[] x,y position of the desired point 
        // link[] length of the two links
        // theta0[] initial guess of the thetas of the two links
    	int outpt[];
    	
    	if( m == 0){ // newton numerical 
	        double alpha = 0.00001;
	        double err = 1;
	        double w[] = new double[]{ pos[0], pos[1]}; // input position
	        double dPos[] = new double[2]; // difference between current and actual position
	        double cPos[] = new double[2]; // current position using th
	        double J[][] = new double[2][2]; // jacobian
	        double invJ[][] = new double[2][2]; // inverse jacobian
	        double dTh[] = new double[2]; // change in theta
	        
	        
	        double th[] = theta0;
	        cPos = fwdKinPos( th, link);
	        // iterate until th is the actual theta
	        int j = 0;
	        do {
	        	
	        	// calculate the difference between actual and current position
	        	dPos[0] = w[0] - cPos[0];
	        	dPos[1] = w[1] - cPos[1];
	        	
	        	// calculate Jacobian
	        	J = newtonJacob( th, link, alpha);
	        	invJ = inv2x2Mat( J);
	        	
	        	// dth = invJ * dPos
	        	dTh[0] = invJ[0][0] * dPos[0] + invJ[0][1] * dPos[1];
	        	dTh[1] = invJ[1][0] * dPos[0] + invJ[1][1] * dPos[1];
	        	//dTh[0] = J[0][0] / -dPos[0] + J[0][1] / -dPos[1];
	        	//dTh[1] = J[1][0] / -dPos[0] + J[1][1] / -dPos[1];
	        	
	        	// update th
	        	th[0] = (th[0] + dTh[0]);
	        	th[1] = (th[1] + dTh[1]);

	        	cPos = fwdKinPos( th, link);
	        	j++;
	        	if( j > 3000) {
	        		System.out.println("err " + euclidDist( cPos, w));
	        		break;
	        	}
	        } while(  euclidDist( cPos, w) > err );
	        
	        
	        outpt = new int[]{ 
	        		(int) ( th[0]) % 360, 
	        		(int) ( th[1]) % 360
	        };
	        
    	} else { // analytical
    		double x = pos[0];
    		double y = pos[1];
    		double l1 = link[0];
    		double l2 = link[1];
    		
    		double th2 = Math.acos( ( x * x + y * y - l1 * l1 - l2 * l2) / ( 2 * l1 * l2));
    		//double th1 = Math.asin( ( l2 * Math.asin( th2) / Math.sqrt( x * x + y * y)) + Math.atan2(y, x));
    		double th1 = Math.atan( y / x) - Math.atan( ( l2 * Math.sin(th2)) / ( l1 + l2 * Math.cos(th2)));
    		
    		outpt = new int[]{ 
	        		(int) Math.toDegrees( th1), 
	        		(int) Math.toDegrees( th2)
	        };
    		
    	}
    	
        return outpt;




    }

    public static double euclidDist( double[] p1, double[] p2){
        // return the euclidiean distance between two points
        double x1 = p1[0];
        double x2 = p2[0];
        double y1 = p1[1];
        double y2 = p2[1];

        double x = ( x1 - x2) * ( x1 - x2);
        double y = ( y1 - y2) * ( y1 - y2);

        return Math.sqrt( x + y);
    }

    public static double lawOfCos( double sA, double sB, double sC){
    	// using the law of cosines return the angle corresponding to point a
    	// the inputs are the length of the side opposite to the angle
    	
    	System.out.println("a " + sA);
    	System.out.println("b " + sB);
    	System.out.println("c " + sC);
    	
    	double cosA = ( sB * sB + sC * sC - sA * sA) / ( 2 * sC * sB);
    	
    	System.out.println(cosA);
    	return Math.toDegrees( ( Math.acos( cosA)));
    	
    }
}