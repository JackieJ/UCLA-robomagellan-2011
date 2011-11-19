// main.cpp

#include "AX3500.h"
#include <iostream>

using std::cout;


#ifndef _WIN32
	const char *PORT = "/dev/ttyUSB0";
#else
	const char *PORT = "COM3";
#endif


suseconds_t elapsed(const timeval &then, const timeval &now)
{
	return now.tv_usec - then.tv_usec + (now.tv_sec - then.tv_sec) * 1000000;
}

void testQueries(AX3500 &ax3500)
{
	unsigned char c1, c2;
	char c3, c4;
	float f1, f2;
	int i1, i2;
	bool b1, b2, b3, b4;
	timeval t1, t2, t3;
	gettimeofday(&t1, NULL);

	c1 = c2 = 0xff;
	ax3500.QueryMotorPower(c1, c2);
	gettimeofday(&t2, NULL);
	cout << "Motor power: [" << (int)c1 << ", " << (int)c2 << "] in " << elapsed(t1, t2) << " microseconds \n\n";

	f1 = f2 = -1;
	ax3500.QueryMotorCurrent(f1, f2);
	cout << "Motor current: [" << f1 << ", " << f2 << "]\n\n";

	f1 = f2 = -1;
	ax3500.QueryAnalogInputs12(f1, f2);
	cout << "Analog inputs 1, 2: [" << f1 << ", " << f2 << "]\n\n";

	f1 = f2 = -1;
	ax3500.QueryAnalogInputs34(f1, f2);
	cout << "Analog inputs 3, 4: [" << f1 << ", " << f2 << "]\n\n";

	i1 = i2 = -1;
	ax3500.QueryTemperature(i1, i2);
	cout << "Temperature: [" << i1 << ", " << i2 << "]\n\n";

	f1 = f2 = -1;
	ax3500.QueryBatteryVoltages(f1, f2);
	cout << "Battery voltages: [" << f1 << ", " << f2 << "]\n\n";

	b1 = b2 = b3 = false;
	ax3500.QueryDigitalInputs(b1, b2, b3);
	cout << "Digital inputs: [" << (int)b1 << ", " << (int)b2 << ", " << (int)b3 << "]\n\n";

	i1 = -1;
	ax3500.ReadEncoder(AX3500::ENCODER_BOTH, AX3500::ABSOLUTE, i1);
	cout << "Encoder counter: " << i1 << "\n\n";

	c3 = c4 = -1;
	ax3500.ReadSpeed(c3, c4);
	cout << "Encoder speeds: [" << (int)c3 << ", " << (int)c4 << "]\n\n";

	c3 = c4 = -1;
	ax3500.ReadFilteredSpeed(c3, c4);
	cout << "Encoder speeds (filtered): [" << (int)c3 << ", " << (int)c4 << "]\n\n";

	b1 = b2 = b3 = b4 = false;
	ax3500.ReadQuadrature(b1, b2, b3, b4);
	cout << "Encoder quadrature: [" << (int)b1 << ", " << (int)b2 << ", " << (int)b3 << ", " << (int)b4 << "]\n\n";

	gettimeofday(&t3, NULL);
	cout << "11 queries in " << elapsed(t1, t3) << " microseconds (" << elapsed(t1, t3)/11 << " per)\n\n";
}

void testEncoderQueries(AX3500 &ax3500)
{
	char c1, c2;
	int i;
	bool b1, b2, b3, b4;

	i = -1;
	ax3500.ReadEncoder(AX3500::ENCODER_BOTH, AX3500::ABSOLUTE, i);
	cout << "Encoder counter: " << i << "\n\n";

	c1 = c2 = -1;
	ax3500.ReadSpeed(c1, c2);
	cout << "Encoder speeds: [" << (int)c1 << ", " << (int)c2 << "]\n\n";

	c1 = c2 = -1;
	ax3500.ReadFilteredSpeed(c1, c2);
	cout << "Encoder speeds (filtered): [" << (int)c1 << ", " << (int)c2 << "]\n\n";

	b1 = b2 = b3 = b4 = false;
	ax3500.ReadQuadrature(b1, b2, b3, b4);
	cout << "Encoder quadrature: [" << (int)b1 << ", " << (int)b2 << ", " << (int)b3 << ", " << (int)b4 << "]\n\n";
}

void dumpMemory(AX3500 &ax3500)
{
	timeval then, now;
	cout << "Beginning memory dump\n";
	gettimeofday(&then, NULL);

	char value = 0xFF;
	for (int i = 0; i <= 0x1D; i++)
		ax3500.ReadMemory(i, value);
	for (int i = 0xF0; i <= 0xF1; i++)
		ax3500.ReadMemory(i, value);
	for (int i = 0x80; i <= 0x8C; i++)
		ax3500.ReadEncoderMemory(i, value);

	gettimeofday(&now, NULL);
	cout << "Memory dump took " << elapsed(then, now) << " microseconds\n\n";
}

int main(int argc, char **argv)
{
	AX3500 ax3500;
	timeval now, then;
	gettimeofday(&then, NULL);

	cout << "MAIN: Opening port\n";
	if (!ax3500.Open(PORT))
	{
		cout << "MAIN: Open failed\n";
		return 0;
	}
	gettimeofday(&now, NULL);
	cout << "MAIN: Open succeeded in " << elapsed(then, now) << " microseconds\n\n";

	//testQueries(ax3500);
	//dumpMemory(ax3500);

	/*
	// Sample commands
	ax3500.ResetWatchdogTimer();
	ax3500.SetSpeed(AX3500::CHANNEL_1, 20);
	ax3500.WriteMemory(0x01, 0x12);
	ax3500.WriteMemory(0xC0, 0x90);
	ax3500.SetSpeed(AX3500::CHANNEL_1, -1);
	ax3500.ResetEncoder(AX3500::ENCODER_BOTH);
	ax3500.WriteMemory(0x01, 0x15);
	ax3500.ToggleAccessoryOutput(false);
	*/


	// If only one wheel spins, the motor is in linear/steering mode
	ax3500.SetSpeed(AX3500::CHANNEL_LINEAR, 5);
	ax3500.SetSpeed(AX3500::CHANNEL_STEERING, 5);
	sleep(8);

	cout << "MAIN: Closing port\n";
	ax3500.Close();
	cout << "MAIN: Port is closed\n";
	return 0;
}
