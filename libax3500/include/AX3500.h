/*
 * Copyright (C) 2011-2112 Garrett Brown <gbruin@ucla.edu>
 *
 * This Program is free software; you can redistribute it and/or modify it
 * under the terms of the Modified BSD License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *    3. Neither the name of the organization nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * This Program is distributed AS IS in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 * Date: November 12, 2011
 * This program was written with shit I learned in an awesome multithreading tutorial
 * http://www.paulbridger.com/multithreading_tutorial/
 *
 */

#ifndef AX3500_H_
#define AX3500_H_

#include <vector>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/tuple/tuple.hpp>

/* Allow serial port traffic (i/o) to be displayed on std::cout */
//#define DEBUG_SERIAL_IO
#define DEBUG_SERIAL_ERROR

#if defined(DEBUG_SERIAL_IO)
#include <iostream> // for debug cout's
#endif

/* Reference of possible MCU memory addresses */
#define AX3500_FLASH_INPUT_CONTROL_MODE    0x00
#define AX3500_FLASH_MOTOR_CONTROL_MODE    0x01
#define AX3500_FLASH_AMPS_LIMIT            0x02
#define AX3500_FLASH_ACCELERATION          0x03
#define AX3500_FLASH_INPUT_SWITCH_FUNCTION 0x04
// reserved                                0x05
#define AX3500_FLASH_JOYSTICK_DEADBAND     0x06
#define AX3500_FLASH_ANALOG_DEADBAND       0x06
// reserved? Might be 0x09, need to check  0x07 // problem, see conflicting pages 148 and 152 of the manual
#define AX3500_FLASH_EXPONENTIATION_CH1    0x08
#define AX3500_FLASH_EXPONENTIATION_CH2    0x09
#define AX3500_FLASH_LEFT_RIGHT_ADJUST     0x0A
#define AX3500_FLASH_ENCODER1_TIME_BASE    0x0B
#define AX3500_FLASH_ENCODER2_TIME_BASE    0x0C
// reserved                                0x0D
#define AX3500_FLASH_DISTANCE_DIVIDER      0x0E
#define AX3500_FLASH_PID_GAIN_PROP         0x0F
#define AX3500_FLASH_PID_GAIN_INTEGRAL     0x10
#define AX3500_FLASH_PID_GAIN_DIFF         0x11
#define AX3500_FLASH_JOYSTICK1_CENTER_MS   0x12
#define AX3500_FLASH_JOYSTICK1_CENTER_LS   0x13
#define AX3500_FLASH_JOYSTICK2_CENTER_MS   0x14
#define AX3500_FLASH_JOYSTICK2_CENTER_LS   0x15
#define AX3500_FLASH_JOYSTICK1_MIN_MS      0x16
#define AX3500_FLASH_JOYSTICK1_MIN_LS      0x17
#define AX3500_FLASH_JOYSTICK2_MIN_MS      0x18
#define AX3500_FLASH_JOYSTICK2_MIN_LS      0x19
#define AX3500_FLASH_JOYSTICK1_MAX_MS      0x1A
#define AX3500_FLASH_JOYSTICK1_MAX_LS      0x1B
#define AX3500_FLASH_JOYSTICK2_MAX_MS      0x1C
#define AX3500_FLASH_JOYSTICK2_MAX_LS      0x1D
#define AX3500_FLASH_AMPS_CALIBRATE_PARAM1 0xF0
#define AX3500_FLASH_AMPS_CALIBRATE_PARAM2 0xF1

#define AX3500_RAM_CHANNEL1_OPERATING_MODE 0x80
#define AX3500_RAM_CHANNEL2_OPERATING_MODE 0x81
#define AX3500_RAM_PID_GAIN_PROP1          0x82
#define AX3500_RAM_PID_GAIN_INTEGRAL1      0x83
#define AX3500_RAM_PID_GAIN_DIFF1          0x84
#define AX3500_RAM_PID_GAIN_PROP2          0x85
#define AX3500_RAM_PID_GAIN_INTEGRAL2      0x86
#define AX3500_RAM_PID_GAIN_DIFF2          0x87
#define AX3500_RAM_PWM_FREQUENCY           0x88
#define AX3500_RAM_CONTROLLER_STATUS       0x89
#define AX3500_RAM_CONTROLLER_MODEL        0x8A
#define AX3500_RAM_CURRENT_AMPS_LIMIT1     0x8B
#define AX3500_RAM_CURRENT_AMPS_LIMIT2     0x8C


class AX3500
{
public:
	/* Motors can be independent or associated. Check AX3500_FLASH_MOTOR_CONTROL_MODE */
	enum Channel
	{
		CHANNEL_1, CHANNEL_LINEAR   = CHANNEL_1,
		CHANNEL_2, CHANNEL_STEERING = CHANNEL_2
	};
	enum Encoder
	{
		ENCODER_1    = 0x1,
		ENCODER_2    = 0x2,
		ENCODER_BOTH = ENCODER_1 | ENCODER_2 // 0x3
	};
	enum EncoderCounterMode
	{
		ABSOLUTE,
		RELATIVE
	};

	AX3500();
	~AX3500();

	/*
	 * Open the port and connect to the AX3500 motor controller. The AX3500
	 * might not be in the correct mode to accept commands over the serial port,
	 * so this function is responsible for enabling the correct modes. For a
	 * breakdown of what this entails, read through the comments in the function.
	 *
	 * Examples:
	 * Linux: Open("/dev/ttyUSB0");
	 * Win32: Open("COM3");
	 *
	 * The safety cutoff mode available through the second command (default off)
	 * will cut power to the motors if no activity is performed for 1 second.
	 * For this to work, hardware support must be enabled (memory location
	 * AX3500_FLASH_INPUT_CONTROL_MODE must not be 0x01). If the hardware is
	 * misconfigured, you can fix this by running (once):
	 *
	 * WriteMemory(AX3500_FLASH_INPUT_CONTROL_MODE, 0x02);
	 * Reset();
	 */
	bool Open(std::string devname, bool safetyCutoff = false);

	/*
	 * Disconnect from the AX3500 and close the serial port.
	 */
	void Close();

	/*
	 * Returns true if a connection to a serial device exists. If the
	 * connection is in an intermediate stage (e.g. being opened or closed
	 * from another thread) then this function will block until the open or
	 * close attempt finishes.
	 */
	bool IsOpen() const;

	/*
	 * Send a speed of position value from -128 to 127 in the forward or reverse
	 * direction for a given channel. In mixed mode, channel 1 value sets the
	 * common forward and reverse value for both motors, while channel 2 sets the
	 * difference between motor 1 and motor 2 as required for steering. In all
	 * other modes, channel 1 commands motor 1 and channel 2 commands motor 2.
	 *
	 * Commands: !Ann, !Bnn, !ann, !bnn
	 */
	void SetSpeed(Channel channel, char speed);

	/*
	 * Turn on or off the digital output line on the 15-pin connector. See "AX3500’s
	 * Inputs and Outputs" on page 56 of the AX3500 user manual for details on how
	 * to identify and wire these signals.
	 *
	 * Commands: !C and !c
	 */
	void ToggleAccessoryOutput(bool enable);

	/*
	 * Reset the motor controller. In addition to sending the reset command,
	 * this will reestablish connection and verify that the motor controller
	 * is in the proper mode.
	 *
	 * Command: %rrrrrr
	 */
	void Reset();

	/*
	 * Set one or both counters to zero.
	 *
	 * A second mode exists, where the counters may be set to an arbitrary 32-bit
	 * number by loading parameters into the encoder MCU's memory using
	 * WriteEncoderMemory(). However, this mode is not supported by this function.
	 *
	 * Command: !Q2
	 */
	void ResetEncoder(Encoder encoder);

	/*
	 * Send a null character to the motor controller to keep it awake. This is
	 * only required if safety cutoff mode is enabled, and only if no other
	 * command has been sent within within the last second.
	 *
	 * Command: \0 (null character)
	 */
	void TickleWatchdogTimer();

	/*
	 * This query will cause the controller to return the actual amount of power
	 * that is being applied to the motors at that time. The number is a hexadecimal
	 * number ranging from 0 to +127. In most cases, this value is directly related
	 * to the command value.
	 *
	 * Note: The applied power value that is read back from the controller can be
	 * different than the command values for any of the following reasons: current
	 * limitation is active, motors operate at reduced speed after overheat
	 * detection, or mixed mode is currently active. No forward or reverse direction
	 * information is returned by this query. This query is most useful for providing
	 * feedback to a microcontroller commanding the controller.
	 *
	 * Command: ?V
	 */
	void QueryMotorPower(unsigned char &Motor1_relative, unsigned char &Motor2_relative);

	/*
	 * This query will cause the controller to return the actual number of Amps
	 * flowing from the battery to power each motor.
	 *
	 * The Amps measurement has an approximately 10% precision. Its main purpose
	 * is to provide feedback to the controller’s current limitation circuitry.
	 *
	 * Important Notice: The current flowing in the motor can be higher than the
	 * battery flowing out of the battery. See "Battery Current vs. Motor Current"
	 * on page 45.
	 *
	 * Value returned will be between 0 and 127.5 amps (in steps of 0.5 amps).
	 *
	 * Command: ?A
	 */
	void QueryMotorCurrent(float &Motor1_amps, float &Motor2_amps);

	/*
	 * This query will cause the controller to return the values of the signals
	 * present at its two analog inputs. If the controller is used in close-loop
	 * speed mode with analog feedback, the values represent the actual speed
	 * measured by the tachometer. When used in position mode, the values represent
	 * the actual motor position measured by a potentiometer. In all other modes,
	 * the values represent the measured voltage (0 to 5V) applied to the analog
	 * inputs.
	 *
	 * Command: ?P
	 */
	void QueryAnalogInputs12(float &Input1_volts, float &Input2_volts);

	/*
	 * See QueryAnalogInputs12.
	 *
	 * Note: On controllers prior to RevB, querying Analog inputs 3 and 4 will
	 * return a meaningless number.
	 *
	 * Command: ?R
	 */
	void QueryAnalogInputs34(float &Input3_volts, float &Input4_volts);

	/*
	 * This query will cause the controller to return the temperature measured
	 * by internal thermistors located at each heatsink side of the controller.
	 * NTC thermistors are non-linear devices, so a conversion using interpolation
	 * over a lookup table is performed.
	 *
	 * If the resulting temperature lies outside the bounds of the lookup table,
	 * then a default value of 150 degrees Celsius will be returned.
	 *
	 * Command: ?M
	 */
	void QueryTemperature(int &Thermistor1_celsius, int &Thermistor2_celsius);

	/*
	 * Retrieve temperature measurements and average them to report a single
	 * temperature value.
	 */
	void QueryAverageTemperature(int &Temperature_celsius);

	/*
	 * This query will cause the controller to return values based on two internally
	 * measured voltages: the first is the Main Battery voltage present at the thick
	 * red and black wires. The second is the internal 12V supply needed for the
	 * controller’s microcomputer and MOSFET drivers.
	 *
	 * Command: ?E
	 */
	void QueryBatteryVoltages(float &MainBattery_volts, float &PowerControl_volts);

	/*
	 * This query will cause the controller to return the state of the controller’s
	 * two accessory inputs (inputs E and F) and the state of the Emergency
	 * Stop/Inverted input. See “Connecting Sensors and Actuators to Input/Outputs”
	 * on page 55 for information on how to wire and use these signals.
	 *
	 * Command: ?I
	 */
	void QueryDigitalInputs(bool &InputE, bool &InputF, bool &EmergencySwitch_pulled_up);

	/*
	 * It is possible to use RS232 commands to examine and change the
	 * controller’s parameters stored in Flash. The complete list of parameters
	 * accessible using these commands is listed in “Automatic Switching from
	 * RS232 to RC Mode” on page 169. Note that many parameters will not take
	 * effect until the controller is reset.
	 *
	 * No changes will be made when attempting to read or write a parameter that
	 * does not exist or when attempting to store a parameter with an invalid
	 * value.
	 *
	 * Additionally, it is possible to change several of the controller’s
	 * operating modes, on-the-fly during normal operation. Unlike the
	 * Configuration Parameters that are stored in Flash (see above), the
	 * Operating Parameters are stored in RAM and can be changed indefinitely.
	 * After reset, the Operating Parameters are loaded with the values stored
	 * in the Configuration Parameter flash. They are then changed using RS232
	 * commands.
	 *
	 * As a general rule, flash addresses are 0x00 - 0x1D and 0xF0 - 0xF1.
	 * RAM addresses are 0x80 - 0x8C. Not all addresses are writable.
	 *
	 * See page 148 for flash configuration parameters and page 155 for RAM
	 * operating parameters.
	 *
	 * Command: ^nn
	 */
	void ReadMemory(char address, char &value);

	/*
	 * Counterpart to ReadMemory(). Because flash can only be written a finite
	 * number of times, WriteMemory() performs a read first to evaluate whether
	 * a write is necessary. Most parameters will require a Reset() after being
	 * written. Due to this, WriteMemory() is always synchronous (unlike other
	 * commands) so that the subsequent Reset() will occur *after* the memory
	 * is written.
	 *
	 * Command: ^nn mm
	 */
	void WriteMemory(char address, char value);

	/*
	 * Read the value of the Encoder counter(s). The number is a signed 32 bit
	 * number that may range from -2,147,836,648 to +2,147,836,647.
	 *
	 * Counters’ values can be read as Absolute or Relative. An Absolute counter
	 * read will return the full counter value after every read query. In a Relative
	 * counter read, the counter value is immediately cleared immediately after
	 * being read so that the next read query returns the new number of counts since
	 * the last time the counter was read. Additionally, the Encoder module can also
	 * return the sum for both counters. This is useful for measuring the average
	 * traveled distance by the right and left wheels of a robotic vehicle.
	 *
	 * Command: ?Qn
	 */
	void ReadEncoder(Encoder encoder, EncoderCounterMode mode, int &i);

	/*
	 * This query will cause the controller to return the speed computed by the
	 * Encoder module. The values range from -127 to +127. The -127 value represents
	 * the maximum RPM in the reverse direction. +127 represents the maximum RPM in
	 * the forward direction. The relation of this relative number and the actual,
	 * absolute RPM value depends on the encoder’s resolution and a user programmable
	 * Time Base. See "Using the Encoder to Measure Speed" on page 84 of the user
	 * manual for a detailed discussion.
	 *
	 * Command: ?Z
	 */
	void ReadSpeed(char &Encoder1_relative, char &Encoder2_relative);

	/*
	 * This query is will cause the controller to return either the speed or the
	 * distance computed by the Encoder module, depending on the operating mode
	 * that is selected. This command is similar to ReadSpeed(), except that it is
	 * read from a different location inside the controller and is a filtered value
	 * that smoothened abrupt changes.
	 *
	 * Command: ?K
	 */
	void ReadFilteredSpeed(char &Encoder1_relative, char &Encoder2_relative);

	/*
	 * This query will cause the controller to return the status of the four optional
	 * Encoder limit switches. Note that for this function to work, limit switches
	 * must be connected to the encoder module using the special wiring diagram show
	 * in "Wiring Optional Limit Switches" on page 82 of the user manual. If no limit
	 * switches are present, this query will return the logic levels of each of the
	 * encoder’s quadrature outputs, which in most cases is not relevant information.
	 *
	 * Command: ?W
	 */
	void ReadQuadrature(bool &Switch1, bool &Switch2, bool &Switch3, bool &Switch4);

	/*
	 * These commands make it possible to examine and change parameters that are
	 * stored in the Encoder’s module MCU RAM. While this command provides
	 * unrestricted access to up to 256 memory locations, a small number of these
	 * locations should never be read or altered.
	 *
	 * See page 163 in the user manual.
	 *
	 * Command: *nn
	 */
	void ReadEncoderMemory(char address, char &value);

	/*
	 * Counterpart to ReadEncoderMemory(). Because flash can only be written a
	 * finite number of times, WriteMemory() performs a read to evaluate
	 * whether a write is necessary. Unlike other commands, memory writes are
	 * not asynchronous.
	 *
	 * Command: *nn mm
	 */
	void WriteEncoderMemory(char address, char value);

private:
	void io_run();
	void watchdog_run();

	/*
	 * Allow me to demystify this item. When a command is ready to be sent to
	 * the IO queue, it is wrapped in a whole slew of objects and packed with
	 * some other stuff. In order to achieve synchronous writes, the commanding
	 * function is put to sleep until the write is finished and the output
	 * recorded. The boost::condition, passed to the IO queue, allows the IO
	 * thread to wake the commanding function when complete.
	 *
	 * The shared_array is used to pass results back to the commanding function.
	 * If this is null, results are still read but are not passed back.
	 *
	 * The final int is the number of response lines to expect (in addition to
	 * the echoed command). This value must be correct! It can be obtained from
	 * the AX3500 user manual.
	 */
	typedef boost::tuple< boost::shared_ptr<std::string>, /* command */
	                      boost::shared_ptr<boost::condition>, /* wait condition */
	                      boost::shared_array<char>, /* result(s), this can be null */
	                      int /* count of expected responses */
	                  > io_queue_t;

	/*
	 * Inserts the command in the command queue. Command is placed after other
	 * commands and before any queries. If the same command is present in the
	 * queue it will be deleted.
	 */
	void AddCommand(const io_queue_t &command, std::vector<io_queue_t> &queue) const;

	// Member variables!

	std::string              m_deviceName; // Only used for Reset() <TODO: and GetDeviceName()>
	boost::asio::io_service  m_io; // The I/O service talks to the serial device
	boost::asio::serial_port m_port;
	bool                     m_bSafetyCutoffOption; // Only stored for the Reset() command
	
	/*
	 * Serial port access is only monitored by a mutex in the Open() and Close()
	 * functions. Outside of these functions, only io_run() touches the serial
	 * port, and io_run() is killed before Open()/Close() acquires the mutex.
	 */
	mutable boost::mutex     open_mutex; // This mutex is mutable so that IsOpen() const may use it

	/*
	 * Because io_run() only exists once (creation upon a successful Open()),
	 * it doesn't require its own serial port mutex. Instead, serial port access
	 * is synchronized to queue of IO requests, io_queue; io_queue is monitored
	 * using io_mutex.
	 */
	boost::thread            io_thread;
	std::vector<io_queue_t>  io_queue; // see documentation above for io_queue_t
	mutable boost::mutex     io_mutex;
	boost::condition         io_condition;

	boost::thread            watchdog_thread;
	boost::condition         watchdog_condition;
	bool                     m_bRunning; // are the IO and watchdog threads running
	
	// Helper functions

	/*
	 * Temperature Conversion Code (page 68)
	 *
	 * The analog value that is reported by the motor controller will range from 0
	 * (warmest) to 255 (coldest). Because of the non-linear characteristics of NTC
	 * thermistors, the conversion from measured value to temperature must be done
	 * using the correction curve in the user manual.
	 *
	 * If the lookup fails because c is too high (above 250ish) or too low (below
	 * 9ish) then this will default to returning 150 degrees C.
	 */
	int toCelsius(unsigned char c);

	/*
	 * In order to create a more efficient data stream on the controller’s serial
	 * port, a simple compression technique is implemented. The scheme eliminates
	 * all of the counter’s most significant bits if they are at 0 (for a positive
	 * count number) or F (for a negative count number).
	 *
	 * For example, if the counter value is Hex 00000015, the value 15 will be
	 * returned after a counter query.
	 *
	 * For negative numbers, a count value of -5 (which is FFFFFFFB in hex), the
	 * response to the query will be B.
	 *
	 * To distinguish between positive and negative numbers, the Encoder module
	 * will insert a 0 ahead of any number string starting with a digit value
	 * higher than 7 (i.e. 8 to F) to signify that the number is positive. For
	 * negative numbers, the Encoder module will insert an F ahead of any number
	 * string starting with a digit value lower than 8 (i.e. 0 to 7). The table
	 * below shows examples of this scheme as applied to various counter values.
	 *
	 *       Decimal   32-bit Hex  Controller Output
	 *            +5     00000005             5
	 *          +250     000000FA           0FA
	 *            -6     FFFFFFFA             A
	 *          -235     FFFFFF15           F15
	 *   +91,011,186     056CB872       56CB872
	 *    -7,986,091     FF862455        862455
	 *
	 * See page 168 in the user manual.
	 */
	unsigned int fromHex32(const char *src) const;

	inline unsigned char fromHex(const char *src) const
	{
		unsigned char value;
		value = (std::isdigit(src[0]) ? (src[0] - '0') : (std::toupper(src[0]) - 'A' + 10)) << 4;
		value += std::isdigit(src[1]) ? (src[1] - '0') : (std::toupper(src[1]) - 'A' + 10);
		return value;
	}

	inline char* toHex(unsigned char value, char *dest) const
	{
		dest[0] = value / 16;
		dest[0] += (dest[0] > 9 ? 'A' - 10 : '0');
		dest[1] = value % 16;
		dest[1] += (dest[1] > 9 ? 'A' - 10 : '0');
		return dest;
	}

	inline bool isCommand(const char *src) const
	{
		// Commands to write memory look like "^MM NN\r" (thus src[3] is a space instead of \r)
		return src[0] == '!' || src[0] == '%' || ((src[0] == '^' || src[0] == '*') && src[3] == ' ');
	}

	inline bool isQuery(const char *src) const
	{
		return !isCommand(src);
	}

	// Debugging commands
	inline void debug_out(std::string out_str)
	{
#if defined(DEBUG_SERIAL_IO)
		std::cout << "<- " << out_str;
#endif
	}

	inline void debug_in(std::string in_str)
	{
#if defined(DEBUG_SERIAL_IO)
		std::cout << "-> " << in_str << '\n';
#endif
	}

	inline void debug_error(std::string err_str)
	{
#if defined(DEBUG_SERIAL_ERROR)
		std::cout << err_str << '\n';
#endif
	}
};


#endif /* AX3500_H_ */
