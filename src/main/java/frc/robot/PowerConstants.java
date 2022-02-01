package frc.robot;

public enum PowerConstants{
    
    // shooter
    ShooterShotsAuton(17000),
    ShooterShotsButton(18000),
    // ShooterShotsButtonLong(18500),
    ShooterShotsButtonLong(19000),
    ShooterReverse(5000), // 100% power
    ShooterJohann(13000),
    Shooter5419Shots(17000),

    // drive base
    DriveBaseLeftStraight(.6),
    DriveBaseRightTurn(.3),
    DriveBaseLeftTurn(.2),
    DriveBaseRightStraight(.4),
    DriveBaseLoadingStation(.2),
    DriveBaseMotionMagickP(.1),
    DriveBaseMotionMagickD(0),
    DriveBaseSetpoint(5),
    TurnWithEncoderSetPoint(19.0),
    // TurnWithEncoderSetPoint(37.8),
    TurnWithEncoderkP(.023),
    TurnWithEncoderkI(0.000009),
    TurnWithEncoderkD(.115),
    TurnWithEncoderTolerance(0.06),

    // revolver
    RevolverShotsButton(1),
    RevolverJohann(.6),
    RevolverToTape(-.45),
    RevolverWithIntake(.4),
    RevolverButtonBoard(.6),
    Revolver5419Shots(1),

    // intake
    IntakeJohannGround(-.9),
    IntakeJohannPlayerStation(.9),

    // climber
    ClimberJohann(.5),
    ClimberOperatorSlow(.5),
    ClimberOperatorFast(1),
    ClimberOperator(.9),

    // loader
    LoaderShotsButton(.7),
    LoaderJohann(1),

    // hood
    HoodPower(.2),
    HoodTime(1000),

    // gyro
    GyrokP(0.085),
    GyrokI(0),
    GyrokD(0),
    GyroRotation(1),

    ;

    public final double val; 
    PowerConstants(double val){
        this.val = val;
    }
    
}