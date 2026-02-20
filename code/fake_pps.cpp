#include <chrono>
#include <gpios.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <stdio.h>

#include <SerialPort.h>
#include <SerialStream.h>
#include <hardware_config/mandeye.h>
#include <gpiod.h>
namespace NMEA
{
const unsigned int BufferLen = 128;

//! Helper structure to get time
struct timestamp
{
	uint8_t hours;
	uint8_t mins;
	uint8_t secs;
	uint8_t day;
	uint8_t month;
	uint8_t year;
};

std::string produceNMEA(const NMEA::timestamp& ts)
{
	char buffer[BufferLen];
	char payload[BufferLen];
	snprintf(
		payload,
		NMEA::BufferLen,
		"GPRMC,%02d%02d%02d.00,A,5109.0262308,N,11401.8407342,W,0.004,133.4,%02d%02d%02d,0.0,E,D",
		ts.hours,
		ts.mins,
		ts.secs,
		ts.day,
		ts.month,
		ts.year);
	size_t len = strnlen(payload, NMEA::BufferLen);
	// compute NMEA checksum on buffer
	uint8_t NMEAChecksumComputed = 0;

	size_t i = 0;
	for(i = 0; i < len; i++)
	{
		NMEAChecksumComputed ^= payload[i];
	}
	// attach cheksum
	snprintf(buffer, NMEA::BufferLen, "$%s*%02X\n", payload, NMEAChecksumComputed);
	return std::string(buffer);
}

NMEA::timestamp GetTimestampFromSec(time_t secsElapsed)
{
	std::tm* timeInfo = gmtime(&secsElapsed);
	NMEA::timestamp ts;
	ts.hours = timeInfo->tm_hour;
	ts.mins = timeInfo->tm_min;
	ts.secs = timeInfo->tm_sec;
	ts.day = timeInfo->tm_mday;
	ts.month = timeInfo->tm_mon + 1;
	ts.year = timeInfo->tm_year - 100;
	return ts;
}
} // namespace NMEA

std::atomic<bool> stop{false};
void oneSecondThread()
{
	// setup serial port
	std::vector<std::unique_ptr<LibSerial::SerialPort>> serialPorts;
	std::vector<gpiod_line*> syncOutsLines;

	const auto portsNames = hardware::GetLidarSyncPorts();
	for(const auto& portName : portsNames)
	{
		std::cout << "opening port " << portName << std::endl;
		std::unique_ptr<LibSerial::SerialPort> serialPort = std::make_unique<LibSerial::SerialPort>();
		serialPort->Open(portName, std::ios_base::out);
		serialPort->SetBaudRate(LibSerial::BaudRate::BAUD_9600);
		serialPorts.emplace_back(std::move(serialPort));
	}
	const auto ouputs = hardware::GetLidarSyncLEDs();
	const auto& chipPath = mandeye::GetGPIOChip();
	std::cout << "Opening GPIO chip " << chipPath << std::endl;

	gpiod_chip *chip = gpiod_chip_open(chipPath);
	if (chip == nullptr)
	{
		std::cerr << "Error: Unable to open GPIO chip." << std::endl;
		std::abort();
	}

	for (const auto& led : ouputs)
	{
		auto pin = hardware::GetLED(led);
		auto line = gpiod_chip_get_line(chip, pin);
		if (line == nullptr)
		{
			std::cerr << "Error: Unable to open GPIO line." << std::endl;
			gpiod_chip_close(chip);
			std::abort();
		}
		// inizializza il pin a LOW (0) subito
		int ret = gpiod_line_request_output(line, "mandeye_fake_pps", 0);
		if (ret < 0)
		{
			std::cerr << "Error: Unable to request GPIO line." << std::endl;
			gpiod_chip_close(chip);
			std::abort();
		}
		syncOutsLines.emplace_back(line);
	}
	assert(serialPorts.size() == syncOutsLines.size()); // fix: era syncOuts

	// setup pps timing
	constexpr uint64_t Rate = 1000; // 1 secondo in ms
	const auto now = std::chrono::system_clock::now();
	uint64_t millisFromEpoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
	millisFromEpoch += Rate;

	// arrotonda al prossimo secondo esatto
	millisFromEpoch = (millisFromEpoch / Rate) * Rate;
	auto waKeUpTime = std::chrono::system_clock::time_point(std::chrono::milliseconds(millisFromEpoch));

	while(!stop)
	{
		// aspetta il secondo esatto
		std::this_thread::sleep_until(waKeUpTime);
		millisFromEpoch += Rate;
		waKeUpTime = std::chrono::system_clock::time_point(std::chrono::milliseconds(millisFromEpoch));

		const uint64_t secs = millisFromEpoch / 1000;
		NMEA::timestamp ts = NMEA::GetTimestampFromSec(secs);

		// --- segnale PPS conforme alle specifiche Livox Mid-360 ---
		// specifica: HIGH tra 20ms e 200ms, periodo 1000ms
		// sequenza: LOW 900ms → HIGH 100ms → LOW (fino al prossimo secondo)

		// 1. impulso HIGH per 100ms (il fronte di salita è il riferimento PPS)
		for (auto& syncOut : syncOutsLines)
		{
			gpiod_line_set_value(syncOut, 1);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// 2. torna LOW subito dopo l'impulso
		for (auto& syncOut : syncOutsLines)
		{
			gpiod_line_set_value(syncOut, 0);
		}

		// 3. invia la stringa NMEA durante il periodo LOW
		//    il LiDAR legge l'ora dalla stringa NMEA che segue il fronte PPS
		const std::string nmeaMessage = NMEA::produceNMEA(ts);
		for (auto& serialPort : serialPorts)
		{
			serialPort->Write(nmeaMessage);
		}

		// 4. rimane LOW fino al prossimo secondo (sleep_until gestisce il resto)
		std::this_thread::sleep_until(waKeUpTime);
	}

	// cleanup
	for (auto& syncOut : syncOutsLines)
	{
		gpiod_line_set_value(syncOut, 0); // assicura LOW all'uscita
		gpiod_line_release(syncOut);
	}
	gpiod_chip_close(chip);
}

int main(int arc, char* argv[])
{
	std::cout << "fake pps" << std::endl;

	std::thread t1(oneSecondThread);
	while(!stop)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	t1.join();
	return 0;
}
