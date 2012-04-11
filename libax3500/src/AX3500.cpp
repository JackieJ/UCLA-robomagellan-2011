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

// RoboteQ AX3500 motor controller multi-threaded interface

#include "AX3500.h"
#include <boost/bind.hpp>
#include <iostream>

#define DELIMITER_CHAR '\r'
#define DELIMITER "\r"

#define COMMAND_RESET             "%rrrrrr" DELIMITER
#define COMMAND_ENTER_SERIAL_MODE DELIMITER DELIMITER DELIMITER DELIMITER DELIMITER \
                                  DELIMITER DELIMITER DELIMITER DELIMITER DELIMITER // x 10

AX3500::AX3500() : m_io(), m_port(m_io), m_bSafetyCutoffOption(false), m_bRunning(false)
{
}

AX3500::~AX3500()
{
	Close();
}

bool AX3500::Open(std::string device, bool safetyCutoff /* = false */)
{
	Close();

	// Enter critical section
	boost::mutex::scoped_lock lock(open_mutex);

	m_port.open(device.c_str());
	if (!m_port.is_open())
	{
		debug_error("Failed to open serial port: " + device);
		return false;
	}

	// Serial communication settings, see page 138 of the AX3500 manual.
	// Flow control is off because the controller is always ready to receive
	// data and can send data at any time.
	m_port.set_option(boost::asio::serial_port_base::baud_rate(9600));
	m_port.set_option(boost::asio::serial_port_base::character_size(7));
	m_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	m_port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even));
	m_port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

	// Now, the AX3500 may be in one of several states:
	//  * Off
	//  * On
	//  * On, in RC/Analog mode (basically it ignores us like a trailer trash mom)
	// How do we find out which state? By sending it a reset command (page 146)
	// and listening for what it spews back.
	//
	// "This command allows the controller to be reset in the same manner as if
	//  the reset button were pressed. This command should be used in exceptional
	//  conditions only or after changing the controller’s parameters in Flash
	//  memory so that they can take effect."
	write(m_port, boost::asio::buffer(COMMAND_RESET, sizeof(COMMAND_RESET) - 1));
	debug_out(COMMAND_RESET);

	// Ready to start accepting input from the AX3500
	// Reading using boost is a multi-buffer process
	boost::asio::streambuf input; // input streambuf
	std::istream is(&input);      // stream to convert streambuf to std::string
	std::string line;             // final resting place of our input line

	// Capture echoed reset command
	read_until(m_port, input, DELIMITER_CHAR);
	std::getline(is, line, DELIMITER_CHAR);
	debug_in(line);

	// Sometimes after Reset() it will print OK instead of echoing COMMAND_RESET,
	// so I guess this means we can skip a bunch of stuff
	if (line.compare("OK") != 0)
	{
		// During a short time at power up, the Encoder’s MCU will send data to
		// the main serial port. (If input control mode is RC/analog, these lines
		// will be garbled numbers and symbols.)
		//  * Power up prompt from main MCU
		//  * Hardware code of main board
		//  * Power up prompt from encoder MCU
		//  * Hardware code of encoder module
		std::string startup_prompt; // startup prompt (multiple lines)

		for (int i = 0; i < 4; i++)
		{
			// This *might* block until an AX3500 is powered up on the serial port
			read_until(m_port, input, DELIMITER_CHAR);
			std::getline(is, line, DELIMITER_CHAR); // Convert streambuf to std::string
			debug_in(line);
			startup_prompt += line + '\n';
		}

		// If the controller is configured in R/C or Analog mode, it will not be
		// able to accept and recognize RS232 commands immediately. However, the
		// controller will be “listening” to the serial port and will enter the
		// serial mode after it has received 10 continuous “Enter” (Carriage
		// Return) characters. At that point, the controller will output an “OK”
		// message, indicating that it has entered the RS232 mode and that it is
		// now ready to accept commands.
		read_until(m_port, input, DELIMITER_CHAR);
		std::getline(is, line, DELIMITER_CHAR);
		debug_in(line);
	}

	if (line.compare("OK") != 0)
	{
		write(m_port, boost::asio::buffer(COMMAND_ENTER_SERIAL_MODE, sizeof(COMMAND_ENTER_SERIAL_MODE) - 1));
		debug_out("\\r\\r\\r\\r\\r\\r\\r\\r\\r\\r\n");

		read_until(m_port, input, DELIMITER_CHAR);
		std::getline(is, line, DELIMITER_CHAR);
		debug_in(line);

		// Ignore any characters that have accumulated before the "OK"
		if (line.substr(line.length() - 2, 2).compare("OK") != 0)
		{
			// bail out
			debug_error("Failed to open serial port, confused by device " + device);
			write(m_port, boost::asio::buffer(COMMAND_RESET, sizeof(COMMAND_RESET) - 1));
			debug_out(COMMAND_RESET);
			m_port.close();
			return false;
		}
	}

	// Rev up the IO thread
	m_bRunning = true;
	boost::thread t(boost::bind(&AX3500::io_run, this));
	io_thread.swap(t);

	// Avoid race conditions by letting the IO thread fully initialize
	usleep(100000); // 0.1s

	m_bSafetyCutoffOption = safetyCutoff;

	// Get the input control mode and watchdog state
	//   Value  Mode
	//       0    R/C Radio mode (default)
	//       1    RS232, no watchdog
	//       2    RS232, with watchdog
	//       3    Analog mode
	char value;
	ReadMemory(AX3500_FLASH_INPUT_CONTROL_MODE, value);
	switch (value)
	{
	case 0:
	case 3:
		// Start up in RS232 mode next time
		WriteMemory(AX3500_FLASH_INPUT_CONTROL_MODE, 0x02);
		break;
	case 1:
		if (!safetyCutoff)
		{
			// No hardware watchdog to tickle, no need for watchdog thread to run
			safetyCutoff = true;
		}
		else
		{
			// Hardware support is missing. Enable it and reset the board
			WriteMemory(AX3500_FLASH_INPUT_CONTROL_MODE, 0x02);

			write(m_port, boost::asio::buffer(COMMAND_RESET, sizeof(COMMAND_RESET) - 1));
			debug_out(COMMAND_RESET);

			line = "";
			for (int i = 0; line.compare("OK") != 0; ++i)
			{
				if (i == 5) // after 5 lines of not-OK (remember, startup could be 4 lines)
				{
					m_port.close();
					return false;
				}
				read_until(m_port, input, DELIMITER_CHAR);
				std::getline(is, line, DELIMITER_CHAR);
				debug_in(line);
			}
		}
		break;
	case 2:
	default:
		break;
	}

	// Disable the AX3500's safety cutoff by starting the watchdog thread
	if (!safetyCutoff)
	{
		boost::thread t2(boost::bind(&AX3500::watchdog_run, this));
		watchdog_thread.swap(t2);
	}

	m_deviceName = device;

	return true;
}

/*
 * Disconnect from the AX3500 and close the serial port.
 *
 * Disclosure: A race condition might exist if thread B performs a query after
 * the IO thread is kill and before Close() exits. More analysis is needed...
 */
void AX3500::Close()
{
	// Kill the IO thread if running
	m_bRunning = false;
	io_condition.notify_one();
	io_thread.join();

	// Kill the watchdog thread too
	watchdog_condition.notify_one();
	watchdog_thread.join();

	// Enter critical section
	boost::mutex::scoped_lock lock(open_mutex);

	m_deviceName = "";
	if (!m_port.is_open())
		return; // Already closed, nothing left to do

	// Reset the board
	write(m_port, boost::asio::buffer(COMMAND_RESET, sizeof(COMMAND_RESET) - 1));
	debug_out(COMMAND_RESET);
	usleep(500 * 1000); // wait for the board to reset
	m_port.close();
}

bool AX3500::IsOpen() const
{
	boost::mutex::scoped_lock lock(open_mutex);

	return m_port.is_open();
}

void AX3500::io_run()
{
	// Reading using boost is a multi-buffer process
	boost::asio::streambuf input; // input streambuf
	std::istream is(&input);      // stream to convert streambuf to std::string
	std::string line;             // final resting place of our input line

	// Event loop
	while (true)
	{
		// Critical section we enter here is different from Open() and Close().
		// This function is still "excluded" because Open() calls Close() which
		// blocks until this function exits. I assume this program could be
		// written using a single mutex, but that's not the way I designed it.
		boost::mutex::scoped_lock io_lock(io_mutex);

		// On entry, release mutex and suspend this thread. On return, reacquire mutex
		while (io_queue.empty() && m_bRunning)
			io_condition.wait(io_lock); // only wait on a locked mutex

		// If we were awaken to exit, then clean up shop and die a quiet death
		if (!m_bRunning)
		{
			io_queue.clear();
			// Note, this is the only exit point of the function
			// Automatically releases the scoped lock upon deconstruction
			return;
		}

		// Pop the front command
		io_queue_t command = io_queue.front();
		io_queue.erase(io_queue.begin());

		// Don't need the lock for the rest of the while loop, let it expire
		io_lock.unlock();

		std::string cmd_copy(command.get<0>()->c_str());

		// Check if watchdog reset. No need to check response (there isn't one)
		if (cmd_copy.length() == 0)
		{
			write(m_port, boost::asio::buffer("", 1));
			debug_out("\\0\n");
			command.get<1>()->notify_one();
			continue;
		}

		// Send the command to the motor controller and read the echoed response
		write(m_port, boost::asio::buffer(cmd_copy));
		debug_out(cmd_copy);

		read_until(m_port, input, DELIMITER_CHAR);
		std::getline(is, line, DELIMITER_CHAR);
		debug_in(line);

		// Sometimes the controller will echo a '\r' and the command will be on
		// the following line. And check for a spurious OK just 'cause.
		if (!line.length() || line.compare("OK") == 0)
		{
			read_until(m_port, input, DELIMITER_CHAR);
			std::getline(is, line, DELIMITER_CHAR);
			debug_in(line);
		}

		// Strip delimiter (not needed for future references to cmd_copy)
		cmd_copy = cmd_copy.substr(0, cmd_copy.length() - 1);

		// Verify that the command is echoed back to us. Strip accumulated W's from line
		while (line[0] == 'W')
			line = line.substr(1, line.length() - 1);
		if (cmd_copy.compare(line) != 0)
			debug_error("Error: expecting " + cmd_copy + ", received " + line);

		// Read one line for each requested response. If a command lied about how
		// many responses it should get, this will block FOREVER. God help us all.
		char *response = command.get<2>().get();
		int count = command.get<3>();
		for (int i = 0; i < count; i++)
		{
			read_until(m_port, input, DELIMITER_CHAR);
			std::getline(is, line, DELIMITER_CHAR);
			debug_in(line);

			// Check for a spurious OK
			if (line.compare("OK") == 0)
			{
				read_until(m_port, input, DELIMITER_CHAR);
				std::getline(is, line, DELIMITER_CHAR);
				debug_in(line);
			}

			// Some commands, like !A00, reply with a plus or minus. If it's a
			// minus, don't store the response because '-' (0x2D) might be
			// interpreted as a result
			if (line[0] == '-')
			{
				//response[i] = '-'; // commented out
//				debug_error("Command failed: " << cmd_copy);
				break;
			}

			if (!response)
				continue;

			// ?q query (Read Encoder Counter query) is unique in that it returns
			// a 32-bit integer (NNNNNNNN). Handle this separately by using four
			// response characters, one for each byte (NOTE: little endian output!)
			if (isQuery(cmd_copy.c_str()) && (cmd_copy[1] == 'q' || cmd_copy[1] == 'Q'))
			{
				unsigned int num = fromHex32(line.c_str());
				for (i = 0; i < count; ++i)
					response[i] = (num >> (8*i)) % 256;
				break;
			}
			// Special cases
			if (line[0] == '+')
				response[i] = '+';
			else // response is two hex characters NN
				response[i] = fromHex(line.c_str());
		}

		// Response array is all filled out. Notify the calling function
		command.get<1>()->notify_one();
	}
}

void AX3500::watchdog_run()
{
	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);

	while (m_bRunning)
	{
		boost::system_time timeout = boost::get_system_time() + boost::posix_time::milliseconds(900);
		watchdog_condition.timed_wait(private_lock, timeout);
		if (m_bRunning)
			TickleWatchdogTimer();
	}
}

void AX3500::AddCommand(const io_queue_t &command, std::vector<io_queue_t> &queue) const
{
	std::string cmd(*command.get<0>());

	// Iterator it servers two purposes. First, we use it to search for commands
	// that attempt to overwrite an existing command on the queue. If a command
	// is found, it is removed and replaced with the new command. Second, we use
	// it to find the best spot to place our new command (being after all other
	// commands and before all other queries).
	std::vector<io_queue_t>::iterator it;
	for (it = queue.begin(); it != queue.end(); it++)
	{
		const char* cmd2 = it->get<0>()->c_str();
		if (isCommand(cmd2) && ((cmd[2] == '^' || cmd[2] == '*')        // If it's a memory write command
		        ? fromHex(cmd2 + 1) == fromHex(cmd.c_str() + 1)         // compare addresses
		            : (std::toupper(cmd[1]) == std::toupper(cmd2[1])))) // otherwise, compare letters
		{
			// Erase and move on to the next command
			it = queue.erase(it);
			if (it == queue.end())
				break;
			cmd2 = it->get<0>()->c_str();
		}
		if (isQuery(cmd2))
		{
			// If we find a query, insert our command right in front of it
			queue.insert(it, command);
			break;
		}
	}
	// If there were no queries in the queue, add our command at the end
	if (it == io_queue.end())
		queue.push_back(command);
}

void AX3500::SetSpeed(AX3500::Channel channel, char speed)
{
	// Build the command
	char cmd[5];
	cmd[0] = '!';
	cmd[1] = (channel == CHANNEL_1 ? 'A' : 'B');
	if (speed >= 0)
	{
		toHex(speed, cmd + 2);
	}
	else
	{
		// make the command lower case to indicate reverse
		cmd[1] = std::tolower(cmd[1]);
		toHex(-speed, cmd + 2);
	}
	cmd[4] = DELIMITER_CHAR;

	// Make the tuple
	boost::shared_ptr<std::string>      cmd_ptr(new std::string(cmd, sizeof(cmd)));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition); // unused
	boost::shared_array<char>           response(NULL); // don't care about the response

	io_queue_t command(cmd_ptr, query_condition, response, 1); // expect a '+' confirmation

	boost::mutex::scoped_lock lock(io_mutex);

	AddCommand(command, io_queue);
	io_condition.notify_one();
}

void AX3500::ToggleAccessoryOutput(bool enable)
{
	// Build the command
	char cmd[3];
	cmd[0] = '!';
	cmd[1] = (enable ? 'C' : 'c');
	cmd[2] = DELIMITER_CHAR;

	// Make the tuple
	boost::shared_ptr<std::string>      cmd_ptr(new std::string(cmd, sizeof(cmd)));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition); // unused
	boost::shared_array<char>           response(NULL); // don't care about the response

	io_queue_t command(cmd_ptr, query_condition, response, 1); // expect a '+' confirmation

	boost::mutex::scoped_lock lock(io_mutex);

	AddCommand(command, io_queue);
	io_condition.notify_one();
}

void AX3500::Reset()
{
	Open(m_deviceName, m_bSafetyCutoffOption); // Nice and easy
}

void AX3500::ResetEncoder(Encoder encoder)
{
	// Build the command
	char cmd[4];
	cmd[0] = '!';
	cmd[1] = 'Q';
	switch(encoder)
	{
	case ENCODER_1:
		cmd[2] = '0';
		break;
	case ENCODER_2:
		cmd[2] = '1';
		break;
	case ENCODER_BOTH:
		cmd[2] = '2';
		break;
	}
	cmd[3] = DELIMITER_CHAR;

	// Make the tuple
	boost::shared_ptr<std::string>      cmd_ptr(new std::string(cmd, sizeof(cmd)));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition); // unused
	boost::shared_array<char>           response(NULL); // don't care about the response

	io_queue_t command(cmd_ptr, query_condition, response, 1); // expect a '+' confirmation

	boost::mutex::scoped_lock lock(io_mutex);

	AddCommand(command, io_queue);
	io_condition.notify_one();
}

void AX3500::TickleWatchdogTimer()
{
	// Make the tuple
	boost::shared_ptr<std::string>      cmd_ptr(new std::string(""));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition); // unused
	boost::shared_array<char>           response(NULL);

	io_queue_t command(cmd_ptr, query_condition, response, 0); // no response

	boost::mutex::scoped_lock lock(io_mutex);

	// If a command already exists on the queue, we don't need to do anything
	if (io_queue.empty())
		io_queue.push_back(command);
	io_condition.notify_one();
}

void AX3500::QueryMotorPower(unsigned char &Motor1_relative, unsigned char &Motor2_relative)
{
	boost::shared_ptr<std::string>      cmd(new std::string("?V" DELIMITER));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[2]);

	io_queue_t command(cmd, query_condition, response, 2);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	// Wait on a private mutex to avoid deadlock
	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Get the response
	Motor1_relative = response[0];
	Motor2_relative = response[1];
}

void AX3500::QueryMotorCurrent(float &Motor1_amps, float &Motor2_amps)
{
	boost::shared_ptr<std::string>      cmd(new std::string("?A" DELIMITER));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[2]);

	io_queue_t command(cmd, query_condition, response, 2);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Get the response
	Motor1_amps = (unsigned char)response[0] / 2.0f;
	Motor2_amps = (unsigned char)response[1] / 2.0f;
}

void AX3500::QueryAnalogInputs12(float &Input1_volts, float &Input2_volts)
{
	boost::shared_ptr<std::string>      cmd(new std::string("?P" DELIMITER));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[2]);

	io_queue_t command(cmd, query_condition, response, 2);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Convert to volts (page 145)
	Input1_volts = ((float)response[0] + 0x80) * 5.0f / 255;
	Input2_volts = ((float)response[1] + 0x80) * 5.0f / 255;
}

void AX3500::QueryAnalogInputs34(float &Input3_volts, float &Input4_volts)
{
	boost::shared_ptr<std::string>      cmd(new std::string("?R" DELIMITER));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[2]);

	io_queue_t command(cmd, query_condition, response, 2);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Convert to volts
	Input3_volts = ((float)response[0] + 0x80) * 5.0f / 255;
	Input4_volts = ((float)response[1] + 0x80) * 5.0f / 255;
}

void AX3500::QueryTemperature(int &Thermistor1_celsius, int &Thermistor2_celsius)
{
	boost::shared_ptr<std::string>      cmd(new std::string("?M" DELIMITER));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[2]);

	io_queue_t command(cmd, query_condition, response, 2);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Adjust for nonlinear nature of thermistors
	Thermistor1_celsius = toCelsius(response[0]);
	Thermistor2_celsius = toCelsius(response[1]);
}

void AX3500::QueryAverageTemperature(int &Temperature_celsius)
{
	int T1, T2;
	QueryTemperature(T1, T2);

	// If a thermistor reported 150, it probably failed, so use the other one
	if (T1 == 150 && T2 != 150)
		Temperature_celsius = T2;
	else if (T2 == 150)
		Temperature_celsius = T1;
	else
		Temperature_celsius = (T1 + T2) / 2;
}

inline int AX3500::toCelsius(unsigned char c)
{
	// Interpolation table
	int TempTable[39] = {248, 246, 243, 240, 235, 230, 224, 217, 208, 199, 188, 177,
	165, 153, 140, 128, 116, 104, 93, 83, 74, 65, 58, 51, 45, 40, 35, 31, 27, 24, 21,
	19, 17, 15, 13, 12, 11, 9, 8};

	int LoTemp, lobound, hibound, temp, i;

	i = 38;
	while (TempTable[i] < c && i > 0)
		i--;

	if (i < 0)
		i = 0;
	if (i == 38)
		return 150;
	else
	{
		LoTemp = i * 5 - 40;
		lobound = TempTable[i];
		hibound = TempTable[i+1];
		temp = LoTemp + (5 * ((c - lobound)*100/ (hibound - lobound)))/100;
		return temp;
	}
}

void AX3500::QueryBatteryVoltages(float &MainBattery_volts, float &PowerControl_volts)
{
	boost::shared_ptr<std::string>      cmd(new std::string("?E" DELIMITER));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[2]);

	io_queue_t command(cmd, query_condition, response, 2);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Convert to volts (page 67)
	MainBattery_volts = 55.0f * ((unsigned char)response[0]) / 256;
	PowerControl_volts = 28.5f * ((unsigned char)response[1]) / 256;
}

void AX3500::QueryDigitalInputs(bool &InputE, bool &InputF, bool &EmergencySwitch_pulled_up)
{
	boost::shared_ptr<std::string>      cmd(new std::string("?I" DELIMITER));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[3]);

	io_queue_t command(cmd, query_condition, response, 3);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Get the response
	InputE = (response[0] == 1);
	InputF = (response[1] == 1);
	EmergencySwitch_pulled_up = (response[2] == 1);
}

void AX3500::ReadMemory(char address, char &value)
{
	// Build the query
	char query[4];
	query[0] = '^';
	toHex(address, query + 1);
	query[3] = DELIMITER_CHAR;
	boost::shared_ptr<std::string>      cmd(new std::string(query, sizeof(query)));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[2]); // first char is value, second is +

	io_queue_t command(cmd, query_condition, response, 2);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Get the value, ignore the confirmation (+/-)
	value = response[0];
}

void AX3500::WriteMemory(char address, char value)
{
	// Flash can only be written to a finite number of times
	char oldValue = 0xFF; // just in case
	ReadMemory(address, oldValue);
	if (oldValue == value)
		return;

	// Build the query
	char cmd[7];
	cmd[0] = '^';
	toHex(address, cmd + 1);
	cmd[3] = ' ';
	toHex(value, cmd + 4);
	cmd[6] = DELIMITER_CHAR;
	boost::shared_ptr<std::string>      cmd_ptr(new std::string(cmd, sizeof(cmd)));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(NULL);

	io_queue_t command(cmd_ptr, query_condition, response, 1); // expect a + confirmation

	{
		boost::mutex::scoped_lock lock(io_mutex);
		AddCommand(command, io_queue);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);
}

void AX3500::ReadEncoder(Encoder encoder, EncoderCounterMode mode, int &Encoder_ticks)
{
	// Build the query
	char query[4];
	query[0] = '?';
	query[1] = 'Q';
	switch (mode)
	{
	case ABSOLUTE:
		switch (encoder)
		{
		case ENCODER_1:
			query[2] = '0';
			break;
		case ENCODER_2:
			query[2] = '1';
			break;
		case ENCODER_BOTH:
			query[2] = '2';
			break;
		}
		break;
	case RELATIVE:
		switch (encoder)
		{
		case ENCODER_1:
			query[2] = '4'; // not a typo
			break;
		case ENCODER_2:
			query[2] = '5';
			break;
		case ENCODER_BOTH:
			query[2] = '6';
			break;
		}
		break;
	}
	query[3] = DELIMITER_CHAR;

	// Make the tuple
	boost::shared_ptr<std::string>      cmd_ptr(new std::string(query, sizeof(query)));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[4]);

	io_queue_t command(cmd_ptr, query_condition, response, 4); // expect a single 32-bit integer as 4 bytes

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Response is a 32-bit number in little endian format
	Encoder_ticks = ((unsigned char)response[0] << (8*0)) +
	                ((unsigned char)response[1] << (8*1)) +
	                ((unsigned char)response[2] << (8*2)) +
	                ((unsigned char)response[3] << (8*3));
}

void AX3500::ReadSpeed(char &Encoder1_relative, char &Encoder2_relative)
{
	boost::shared_ptr<std::string>      cmd(new std::string("?Z" DELIMITER));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[2]);

	io_queue_t command(cmd, query_condition, response, 2);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Get the response
	Encoder1_relative = response[0];
	Encoder2_relative = response[1];
}

void AX3500::ReadFilteredSpeed(char &Encoder1_relative, char &Encoder2_relative)
{
	boost::shared_ptr<std::string>      cmd(new std::string("?K" DELIMITER));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[2]);

	io_queue_t command(cmd, query_condition, response, 2);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Get the response
	Encoder1_relative = response[0];
	Encoder2_relative = response[1];
}

void AX3500::ReadQuadrature(bool &Switch1, bool &Switch2, bool &Switch3, bool &Switch4)
{
	boost::shared_ptr<std::string>      cmd(new std::string("?W" DELIMITER));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[1]);

	io_queue_t command(cmd, query_condition, response, 1);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Get the response
	Switch1 = ((response[0] >> 0) % 2 == 1);
	Switch2 = ((response[0] >> 1) % 2 == 1);
	Switch3 = ((response[0] >> 2) % 2 == 1);
	Switch4 = ((response[0] >> 3) % 2 == 1);
}

void AX3500::ReadEncoderMemory(char address, char &value)
{
	// Build the query
	char query[4];
	query[0] = '*';
	toHex(address, query + 1);
	query[3] = DELIMITER_CHAR;
	boost::shared_ptr<std::string>      cmd(new std::string(query, sizeof(query)));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(new char[2]); // first char for value, second for +

	io_queue_t command(cmd, query_condition, response, 2);

	{
		boost::mutex::scoped_lock lock(io_mutex);

		// Submit the query and wait for a response
		io_queue.push_back(command);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);

	// Get the value, ignore the confirmation (+/-)
	value = response[0];
}

void AX3500::WriteEncoderMemory(char address, char value)
{
	// Flash can only be written to a finite number of times. Only write if
	// necessary. A side effect is this command is now semi-synchronous.
	char oldValue = 0xFF; // just in case
	ReadEncoderMemory(address, oldValue);
	if (oldValue == value)
		return;

	// Build the query (*NN MM)
	char cmd[7];
	cmd[0] = '*';
	toHex(address, cmd + 1);
	cmd[3] = ' ';
	toHex(value, cmd + 4);
	cmd[6] = DELIMITER_CHAR;
	boost::shared_ptr<std::string>      cmd_ptr(new std::string(cmd, sizeof(cmd)));
	boost::shared_ptr<boost::condition> query_condition(new boost::condition);
	boost::shared_array<char>           response(NULL);

	io_queue_t command(cmd_ptr, query_condition, response, 1); // expect a + confirmation

	{
		boost::mutex::scoped_lock lock(io_mutex);
		AddCommand(command, io_queue);
		io_condition.notify_one();
	}

	boost::mutex private_mutex;
	boost::mutex::scoped_lock private_lock(private_mutex);
	query_condition->wait(private_lock);
}

unsigned int AX3500::fromHex32(const char *src) const
{
	// First, expand the number into 8 hex characters
	std::string num(src);
	char pad = (num[0] <= '7' ? '0' : 'F');
	while (num.length() < 8)
		num.insert(0, 1, pad);

	return (fromHex(num.c_str()+0) << (8*3)) +
	       (fromHex(num.c_str()+2) << (8*2)) +
	       (fromHex(num.c_str()+4) << (8*1)) +
	       (fromHex(num.c_str()+6) << (8*0));
}
