import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class segway {
    public static UnregulatedMotor right = new UnregulatedMotor(MotorPort.A);
    public static UnregulatedMotor left = new UnregulatedMotor(MotorPort.D);
    public static EV3GyroSensor gyro = new EV3GyroSensor( SensorPort.S1);
    public static final SampleProvider sp = gyro.getAngleMode();

    
    
    public static void testGyro(){
    	int value = 0;
    	
    	for( int i = 0; i < 50; i++){
    		float sample[] = new float[ sp.sampleSize()];
    		sp.fetchSample(sample, 0);
    		value = (int) sample[0];
    		
    		System.out.println("angle: " + value);
    		Delay.msDelay(500);
    		
    	}
    		
    	
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
    
    public static int getGyro(){
    	float sample[] = new float[ sp.sampleSize()];
		sp.fetchSample(sample, 0);
		return (int) sample[0];
    }
    
    public static void PID(){
    	double kp = 30; // proportional
        double kd = 100; // derivative
        double ki = 0.05; // integral

        

        
        double prev_error = 0;
        double error = 100;
        double integral = 0;
        double derivative;
        double dt = 5;
        
        int output;
     
        double time = 0;

        
        System.out.println("calibrating");
        //gyro.reset();
        //Delay.msDelay(3000);
        
        
        while( true){
               
            error = getGyro(); 
            System.out.println( error);
            
            integral = integral + ( error * dt);
            derivative = ( error - prev_error) / dt;
            output = (int) (kp * error + ki * integral + kd * derivative);
            prev_error = error;
            
            time = time + dt;
            
            left.setPower( pow( output));
            right.setPower( pow( output));
            left.forward();
            right.forward();
            
            Delay.msDelay( (int) dt);
            
            if( time > 10000) break;
        }
        right.stop();
        left.stop();
    }
    
    public static void main( String[] args) {
    	System.out.println("lets go!");
    	
    	
    	PID();
    }
}
