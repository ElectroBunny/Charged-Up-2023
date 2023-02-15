// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Utilities;

// import java.nio.Buffer;

// import java.nio.ByteBuffer;

// import java.nio.ByteOrder;



// import edu.wpi.first.networktables.NetworkTableEntry;

// import edu.wpi.first.wpilibj.I2C;

// import edu.wpi.first.util.sendable.Sendable;

// import edu.wpi.first.wpilibj.SensorBase;

// import edu.wpi.first.wpilibj.interfaces.Accelerometer;

// import edu.wpi.first.wpilibj.interfaces.Gyro;

// import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;



// public class MPU6050 extends SensorBase implements Sendable, Accelerometer, Gyro {



// 	/*
// 	 * Check MPU6050 Manual for register use, default values and ranges
// 	 * https://store.invensense.com/Datasheets/invensense/RM-MPU-6000A.pdf
// 	 */

// 	public static class REG {

// 		// Config Registers

// 		public static final int SMPRT_DIV = 0x19;

// 		public static final int CONFIG = 0x1A;

// 		public static final int GYRO_CONFIG = 0x1B;

// 		public static final int ACCEL_CONFIG = 0x1C;

// 		public static final int FIFO_EN = 0x23;



// 		// Last Accelerometer data registers

// 		public static final int ACCEL_XOUT_H = 0x3B;

// 		public static final int ACCEL_XOUT_L = 0x3C;

// 		public static final int ACCEL_YOUT_H = 0x3D;

// 		public static final int ACCEL_YOUT_L = 0x3E;

// 		public static final int ACCEL_ZOUT_H = 0x3F;

// 		public static final int ACCEL_ZOUT_L = 0x40;



// 		// Last Temperature Date

// 		public static final int TEMP_OUT_H = 0x41;

// 		public static final int TEMP_OUT_L = 0x42;



// 		// Last Gyroscope data

// 		public static final int GYRO_XOUT_H = 0x43;

// 		public static final int GYRO_XOUT_L = 0x44;

// 		public static final int GYRO_YOUT_H = 0x45;

// 		public static final int GYRO_YOUT_L = 0x46;

// 		public static final int GYRO_ZOUT_H = 0x47;

// 		public static final int GYRO_ZOUT_L = 0x48;



// 		// Control Registers

// 		public static final int SIGNAL_PATH_RESET = 0x68;

// 		public static final int USER_CTRL = 0x6A;

// 		public static final int PWR_MGMT_1 = 0x6B;

// 		public static final int PWR_MGTM_2 = 0x6C;



// 		// FIFO Buffer Registers

// 		public static final int FIFO_COUNT_H = 0x72;

// 		public static final int FIFO_COUNT_L = 0x73;

// 		public static final int FIFO_R_W = 0x74;



// 	}



// 	private static final int FIFO_ENABLE = 1 << 6;

// 	private static final int GYRO_FIFO_EN = 0b01110000;



// 	public static final int MPU6050_ADDRESS = 0b1101000;

// 	public static final I2C.Port DEFAULT_I2C_PORT = I2C.Port.kOnboard;

// 	private I2C sensor;

// 	private double accelPerBit = 2 / 0xffff, gyroPerBit = 250 / 0xffff;

// 	private double secondsPerSample = 1 / 1000;

// 	private double xGyro = 0, yGyro = 0, zGyro = 0;



// 	/**
// 	 * Construct MPU6050 using defaults connected to the onboard port
// 	 */

// 	public MPU6050() {

// 		this(DEFAULT_I2C_PORT, 9, false);

// 	}



// 	/**
// 	 * Construct MPU6050 using defaults connected to the onboard port
// 	 * 
// 	 * @param selectionBit
// 	 *            The last bit of the address of the device
// 	 */

// 	public MPU6050(boolean selectionBit) {

// 		this(DEFAULT_I2C_PORT, 9, selectionBit);



// 	}



// 	/**
// 	 * Construct MPU6050 using defaults on a specified port
// 	 * 
// 	 * @param i2cPort
// 	 *            The I2C Port that the device is connected to
// 	 * @param selectionBit
// 	 *            The last bit of the address of the device
// 	 */

// 	public MPU6050(I2C.Port i2cPort, boolean selectionBit) {

// 		this(DEFAULT_I2C_PORT, 9, selectionBit);



// 	}



// 	/**
// 	 * Construct MPU6050 connected to the onboard port
// 	 * 
// 	 * @param sampleRateDiv
// 	 *            The divider for the 1000 kH sample rate
// 	 * @param selectionBit
// 	 *            The last bit of the address of the device
// 	 */

// 	public MPU6050(int sampleRateDiv, boolean selectionBit) {

// 		this(DEFAULT_I2C_PORT, sampleRateDiv, selectionBit);



// 	}



// 	/**
// 	 * Constructs MPU6050 on a specified port
// 	 * 
// 	 * @param i2cPort
// 	 *            The I2C Port that the device is connected to
// 	 * @param sampleRateDiv
// 	 *            The divider for the 1000 kH sample rate
// 	 * @param selectionBit
// 	 *            The last bit of the address of the device
// 	 */

// 	public MPU6050(I2C.Port i2cPort, int sampleRateDiv, boolean selectionBit) {

// 		sensor = new I2C(i2cPort, MPU6050_ADDRESS | (selectionBit ? 1 : 0));



// 		setSampleRateDiv(sampleRateDiv);

// 		sensor.write(REG.CONFIG, 2); // Set EXT_SYNC to OFF and Digital Low Pass Filter to 94Hz for Accel and

// 												// 98 for Gyro

// 		setRange(Range.k4G, GyroRange.k500deg);

// 		sensor.write(REG.FIFO_EN, GYRO_FIFO_EN); // Enable FIFO buffer for Gyro

// 		sensor.write(REG.USER_CTRL, FIFO_ENABLE); // Enable FIFO buffer and disable I2C master

// 		reset();

// 		sensor.write(REG.PWR_MGMT_1, 1); // Use X Gyro as clock reference and wake up



// 	}



// 	@Override

// 	public void calibrate() {

// 		// TODO Auto-generated method stub



// 	}



// 	@Override

// 	public void reset() {

// 		sensor.write(REG.USER_CTRL, FIFO_ENABLE | 0b101);

// 		xGyro = 0;

// 		yGyro = 0;

// 		zGyro = 0;

// 	}



// 	@Override

// 	public double getAngle() {

// 		return zGyro;

// 	}



// 	/**
// 	 * Get the gyro rotation in degrees
// 	 * 
// 	 * @param axis
// 	 *            Axis of rotation
// 	 * @return Angle in degrees
// 	 */

// 	public double getAngle(Axis axis) {

// 		switch (axis) {

// 		case X:

// 			return xGyro;

// 		case Y:

// 			return yGyro;

// 		case Z:

// 			return zGyro;

// 		}

// 		return zGyro;

// 	}



// 	@Override

// 	public double getRate() {

// 		return getGyroVal(getData(Sensors.GyroZ), 0);

// 	}



// 	/**
// 	 * Get Rotation Rate for specific axis of the gyro
// 	 * 
// 	 * @param axis
// 	 *            Axis of the gyro
// 	 * @return Rate of rotation in deg/s
// 	 */

// 	public double getRate(Axis axis) {

// 		ByteBuffer buffer = getData(REG.GYRO_XOUT_H + axis.val, 2);

// 		return getGyroVal(buffer, 0);

// 	}



// 	/**
// 	 * Update gyroscope Angles
// 	 */

// 	public void updateAngles() {

// 		getGyroFIFO();

// 	}



// 	@Override

// 	public void setRange(Range range) {

// 		int val = 0;

// 		switch (range) {

// 		case k2G:

// 			val = 0;

// 			accelPerBit = 2.0 / 0xffff;

// 			break;

// 		case k4G:

// 			val = 1;

// 			accelPerBit = 4.0 / 0xffff;

// 			break;

// 		case k8G:

// 			val = 2;

// 			accelPerBit = 8.0 / 0xffff;

// 			break;

// 		case k16G:

// 			val = 3;

// 			accelPerBit = 16.0 / 0xffff;

// 		}

// 		sensor.write(REG.ACCEL_CONFIG, val << 3);

// 	}



// 	/**
// 	 * Set the measurement range of the Gyroscope
// 	 * 
// 	 * @param gRange
// 	 *            The range selection
// 	 */

// 	public void setRange(GyroRange gRange) {

// 		gyroPerBit = ((double) gRange.val) / 0xffff;

// 		sensor.write(REG.GYRO_CONFIG, gRange.selector << 3);

// 	}



// 	/**
// 	 * Set the measurement of the Accelerometer and the Gyroscope
// 	 * 
// 	 * @param aRange
// 	 *            Accelerometer range selection
// 	 * @param gRange
// 	 *            Gyroscope range selection
// 	 */

// 	public void setRange(Range aRange, GyroRange gRange) {

// 		setRange(aRange);

// 		setRange(gRange);

// 	}



// 	/**
// 	 * Set the rate division
// 	 * 
// 	 * <p>
// 	 * 1000 kH / (divider + 1)
// 	 * 
// 	 * @param divider
// 	 *            divides 1000kH sample rate. The maximum value is 0xff
// 	 */

// 	public void setSampleRateDiv(int divider) {

// 		if (divider <= 0xff) {

// 			sensor.write(REG.SMPRT_DIV, divider);

// 		}

// 	}



// 	@Override

// 	public double getX() {

// 		return getAccelVal(getData(Sensors.AccelX), 0);

// 	}



// 	@Override

// 	public double getY() {

// 		return getAccelVal(getData(Sensors.AccelY), 0);

// 	}



// 	@Override

// 	public double getZ() {

// 		return getAccelVal(getData(Sensors.AccelZ), 0);

// 	}



// 	/**
// 	 * Get Current Acceleration in G's from a specific Axis
// 	 * 
// 	 * @param axis
// 	 *            Axis of the Accelerometer
// 	 * @return Current Acceleration in G's
// 	 */

// 	public double getAccel(Axis axis) {

// 		ByteBuffer buffer = getData(REG.ACCEL_XOUT_H + axis.val, 2);

// 		return getAccelVal(buffer, 0);

// 	}



// 	/**
// 	 * Get Current Accelerometer Data
// 	 * 
// 	 * @return Accelerometer data
// 	 */

// 	public AccelData getAccel() {

// 		AccelData data = new AccelData();

// 		ByteBuffer buffer = getData(Sensors.Accel);

// 		data.XAxis = getAccelVal(buffer, 0);

// 		data.YAxis = getAccelVal(buffer, 2);

// 		data.ZAxis = getAccelVal(buffer, 4);

// 		return data;

// 	}



// 	/**
// 	 * Get current Die temperature
// 	 * 
// 	 * @return
// 	 */

// 	public Double getTemp() {

// 		ByteBuffer buffer = getData(Sensors.Temp);

// 		return tempToDegC(buffer.getShort());

// 	}



// 	/**
// 	 * Get Current gyroscope Data
// 	 * 
// 	 * @return Gyroscope Data
// 	 */

// 	public GyroData getGyro() {

// 		GyroData data = new GyroData();

// 		ByteBuffer buffer = getData(Sensors.Gyro);

// 		data.XAxis = getGyroVal(buffer, 0);

// 		data.YAxis = getGyroVal(buffer, 2);

// 		data.ZAxis = getGyroVal(buffer, 4);

// 		return data;

// 	}



// 	/**
// 	 * Get All Sensor Data from the device
// 	 * 
// 	 * @return All Sensor Data
// 	 */

// 	public AllData getAll() {

// 		AllData data = new AllData();

// 		ByteBuffer buffer = getData(Sensors.All);

// 		data.Accel = new AccelData();

// 		data.Gyro = new GyroData();

// 		data.Accel.XAxis = getAccelVal(buffer, 0);

// 		data.Accel.YAxis = getAccelVal(buffer, 2);

// 		data.Accel.ZAxis = getAccelVal(buffer, 4);

// 		data.Temp = tempToDegC(buffer.getShort(6));

// 		data.Gyro.XAxis = getGyroVal(buffer, 8);

// 		data.Gyro.YAxis = getGyroVal(buffer, 10);

// 		data.Gyro.ZAxis = getGyroVal(buffer, 12);

// 		return data;

// 	}



// 	private double tempToDegC(double temp) {

// 		return ((double) temp / 340) + 36.53;

// 	}



// 	private double getGyroVal(ByteBuffer buffer, int start) {

// 		return buffer.getShort(start) * gyroPerBit;

// 	}



// 	private double getAccelVal(ByteBuffer buffer, int start) {

// 		return buffer.getShort(start) * accelPerBit;

// 	}



// 	private ByteBuffer getData(Sensors s) {

// 		return getData(s.start, s.bytes);

// 	}



// 	private ByteBuffer getData(int address, int bytes) {

// 		ByteBuffer buffer = ByteBuffer.allocate(bytes);

// 		sensor.read(address, bytes, buffer);

// 		return buffer;

// 	}



// 	private void getGyroFIFO() {

// 		int len = getFIFOLength();

// 		if (len == 0)

// 			return;



// 		ByteBuffer buffer = ByteBuffer.allocate(len);

// 		sensor.read(REG.FIFO_R_W, len, buffer);

// 		for (int i = 0; i <= len - 6; i += 6) {

// 			xGyro += getGyroVal(buffer, i) * secondsPerSample;

// 			yGyro += getGyroVal(buffer, i + 2) * secondsPerSample;

// 			zGyro += getGyroVal(buffer, i + 4) * secondsPerSample;

// 		}

// 	}



// 	private int getFIFOLength() {

// 		ByteBuffer buffer = ByteBuffer.allocate(2);

// 		sensor.read(REG.FIFO_COUNT_H, 2, buffer);

// 		return (((int) buffer.get(0)) << 8) + buffer.get(1);

// 	}



// 	public class AccelData {

// 		public double XAxis, YAxis, ZAxis;

// 	}



// 	public class GyroData {

// 		public double XAxis, YAxis, ZAxis;

// 	}



// 	public class AllData {

// 		public AccelData Accel;

// 		public GyroData Gyro;

// 		public double Temp;

// 	}



// 	@Override

// 	public void initSendable(SendableBuilder builder) {

// 		builder.setSmartDashboardType("6AxisAccelGyro");

// 		NetworkTableEntry accelX = builder.getEntry("accelX");

// 		NetworkTableEntry accelY = builder.getEntry("accelY");

// 		NetworkTableEntry accelZ = builder.getEntry("accelZ");



// 		NetworkTableEntry temp = builder.getEntry("temp");



// 		NetworkTableEntry gyroX = builder.getEntry("gyroX");

// 		NetworkTableEntry gyroY = builder.getEntry("gyroY");

// 		NetworkTableEntry gyroZ = builder.getEntry("gyroZ");



// 		NetworkTableEntry angleX = builder.getEntry("angleX");

// 		NetworkTableEntry angleY = builder.getEntry("angleY");

// 		NetworkTableEntry angleZ = builder.getEntry("angleZ");



// 		builder.setUpdateTable(() -> {

// 			getGyroFIFO();

// 			AllData data = getAll();

// 			accelX.setDouble(data.Accel.XAxis);

// 			accelY.setDouble(data.Accel.YAxis);

// 			accelZ.setDouble(data.Accel.ZAxis);



// 			temp.setDouble(data.Temp);



// 			gyroX.setDouble(data.Gyro.XAxis);

// 			gyroY.setDouble(data.Gyro.YAxis);

// 			gyroZ.setDouble(data.Gyro.ZAxis);



// 			angleX.setDouble(xGyro);

// 			angleY.setDouble(yGyro);

// 			angleZ.setDouble(zGyro);

// 		});



// 	}



// 	private enum Sensors {

// 		Accel(REG.ACCEL_XOUT_H, 6), AccelX(REG.ACCEL_XOUT_H, 2), AccelY(REG.ACCEL_YOUT_H, 2), AccelZ(REG.ACCEL_ZOUT_H,

// 				2), Temp(REG.TEMP_OUT_H, 2), Gyro(REG.GYRO_XOUT_H, 6), GyroX(REG.GYRO_XOUT_H,

// 						2), GyroY(REG.GYRO_YOUT_H, 2), GyroZ(REG.GYRO_ZOUT_H, 2), All(REG.ACCEL_XOUT_H, 14);



// 		int bytes, start;



// 		Sensors(int start, int bytes) {

// 			this.start = start;

// 			this.bytes = bytes;

// 		}

// 	}



// 	public enum GyroRange {

// 		k250deg(0, 250), k500deg(1, 500), k1000deg(2, 1000), k2000deg(3, 2000);



// 		int selector, val;



// 		GyroRange(int selector, int val) {

// 			this.selector = selector;

// 			this.val = val;

// 		}

// 	}



// 	public enum Axis {

// 		X(0), Y(2), Z(4);



// 		int val;



// 		Axis(int val) {

// 			this.val = val;
// 		}
// 	}
// }
