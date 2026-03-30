#include <RTOS.h>
#define __DEBUG__ true
#define CurrentBoardID 0x00

// arduino core ============================================================
#include <Arduino.h>
void setup();
void loop();

// Project Board Info ======================================================
// Read Readme.md for more details about the board and pinout
#define MAX_BOARDS //FIXME : define the maximum number of boards in the system

constexpr uint8_t MAC_ADDR [MAX_BOARDS][6] =
{

};


// UART Communication ======================================================
#include "driver/uart.h"
#define MAX_SERIAL_LENGTH 64
#define MAX_PENDING_MESSAGES 10
#define UART_RX_BUFFER_SIZE (MAX_SERIAL_LENGTH * MAX_PENDING_MESSAGES)  // 640
#define UART_TX_BUFFER_SIZE (MAX_SERIAL_LENGTH * 4)                     // 256
#define UART_EVENT_QUEUE_SIZE 20

#define VEX UART_NUM_1
#define UART1_RX_PIN //FIXME : define the pin number for UART1 RX
#define UART1_TX_PIN //FIXME : define the pin number for UART1 TX

#define VoiceRecog UART_NUM_2
#define UART2_RX_PIN //FIXME : define the pin number for UART2 RX
#define UART2_TX_PIN //FIXME : define the pin number for UART2 TX

#define UART_LISTNER_CORE 0
#define UART_BAUD_RATE 115200
#define UART_BLOCK_TICKS pdMS_TO_TICKS(portMAX_DELAY)

#if __DEBUG__
	#undef UART_BLOCK_TICKS
	#define UART_BLOCK_TICKS pdMS_TO_TICKS(1000)
#endif

QueueHandle_t vexV5SerialQueue;
QueueHandle_t voiceSerialQueue;
volatile TickType_t lastVexSerialTick = 0;
volatile TickType_t lastVoiceSerialTick = 0;

void uartInit();
void vexUartListener(void *param);
void vexUartUpdate(void);
void voiceUartListener(void *param);
void onSerialRecieved(const uart_port_t uart_num, const char* cmdText);
void uartPrintf(const uart_port_t uart_num, const char *format, ...);

// ESPNOW Communication ==================================================



// QMC5883P Magnetometer =================================================
#include "Adafruit_QMC5883P.h"
#include "Wire.h"

constexpr float GausThreshold = 0.1F; // FIXME : define the threshold for detecting significant changes in magnetic field
#define I2CSDA // FIXME : define the pin number for I2CSDA
#define I2CSCL // FIXME : define the pin number for I2CSCL
#define I2C_FREQ 400000U

#define COMPASS_LISTNER_CORE 1
#define COMPASS_UPDATE_INTERVAL_MS 100

TwoWire I2C1 = TwoWire(0);
Adafruit_QMC5883P compass;
volatile float azimuth = -9999.0F; // Invalid initial value to indicate uninitialized state

void compassInit();
void compassDataUpdate(void *param);

// RC522 RFID Reader =====================================================
#include <SPI.h>
#include <MFRC522.h>

#define SPI_SCL_PIN // FIXME : define the pin number for SPI SCL
#define SPI_MOSI_PIN // FIXME : define the pin number for SPI MOSI
#define SPI_MISO_PIN // FIXME : define the pin number for SPI MISO

#define Front_RST_PIN // FIXME : define the pin number for Front RFID RST
#define Front_SS_PIN  // FIXME : define the pin number for Front RFID SS (SDA)
#define Center_RST_PIN  // FIXME : define the pin number for Center RFID RST
#define Center_SS_PIN   // FIXME : define the pin number for Center RFID SS (SDA)

MFRC522 frontRFID(Front_SS_PIN, Front_RST_PIN);
MFRC522 centerRFID(Center_SS_PIN, Center_RST_PIN);

void mfrcInit();
void mfrcDataListener(void *param);

// =======================================================================
void setup()
{
	// Initialize Serial==================================================
	#if __DEBUG__
		Serial.begin(115200);
		while(!Serial) delay(50);
	#endif

	// Initialize UART Communication======================================
	uartInit();

	// Initialize Compass=================================================
	compassInit();

	// Initialize RFID Reader=============================================
	mfrcInit();
}

void loop()
{

}

// UART Communication ======================================================
void uartInit()
{
	uart_config_t uartConfig = 
	{
		.baud_rate = UART_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(VEX, &uartConfig);
	uart_param_config(VoiceRecog, &uartConfig);
	uart_set_pin(VEX, UART1_TX_PIN, UART1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_set_pin(VoiceRecog, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(VEX, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, UART_EVENT_QUEUE_SIZE, &vexV5SerialQueue, 0);
	uart_driver_install(VoiceRecog, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, UART_EVENT_QUEUE_SIZE, &voiceSerialQueue, 0);

	delay(500);

	xTaskCreatePinnedToCore
	(
		vexUartListener,
		"VEX UART Listener",
		4096,
		NULL,
		1,
		NULL,
		UART_LISTNER_CORE
	);
	xTaskCreatePinnedToCore
	(
		voiceUartListener,
		"Voice UART Listener",
		4096,
		NULL,
		1,
		NULL,
		UART_LISTNER_CORE
	);
}

void vexUartListener(void *param)
{
	uart_event_t event;
	uint8_t data[MAX_SERIAL_LENGTH];
	while(true)
	{
		if(xQueueReceive(vexV5SerialQueue, (void *)&event, UART_BLOCK_TICKS))
		{
			#if __DEBUG__
				Serial.println("VEX UART Event Received");
			#endif

			switch(event.type)
			{
				case UART_DATA:
					size_t readLen = min(event.size, sizeof(data) - 1);
					int len = uart_read_bytes(VEX, data, readLen, UART_BLOCK_TICKS);
					if(len > 0)
					{
						data[len] = '\0';
						onSerialRecieved(VEX, (char *)data);
					}
					break;
				default:
				#if __DEBUG__
					Serial.printf("Unknown VEX UART Event Type : %d\n", event.type);
				#endif
			}
		}
		#if __DEBUG__
			else
			{
				Serial.println("VEX UART Event Timeout");
			}
		#endif
	}
}

void voiceUartListener(void *param)
{
	uart_event_t event;
	uint8_t data[MAX_SERIAL_LENGTH];
	while(true)
	{
		if(xQueueReceive(voiceSerialQueue, (void *)&event, UART_BLOCK_TICKS))
		{
			#if __DEBUG__
				Serial.println("Voice UART Event Received");
			#endif

			switch(event.type)
			{
				case UART_DATA:
					size_t readLen = min(event.size, sizeof(data) - 1);
					int len = uart_read_bytes(VoiceRecog, data, readLen, UART_BLOCK_TICKS);
					if(len > 0)
					{
						data[len] = '\0';
						onSerialRecieved(VoiceRecog, (char *)data);
					}
					break;
				default:
				#if __DEBUG__
					Serial.printf("Unknown Voice UART Event Type : %d\n", event.type);
				#endif
			}
		}
		#if __DEBUG__
			else
			{
				Serial.println("Voice UART Event Timeout");
			}
		#endif
	}
}

void onSerialRecieved(const uart_port_t uart_num, const char *cmdText)
{
	#if __DEBUG__
		Serial.printf("Data received on UART%d: %s\n", uart_num, cmdText);
	#endif

}

void uartPrintf(const uart_port_t uart_num, const char *format, ...)
{
    char buffer[250];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    uart_write_bytes(uart_num, buffer, strlen(buffer));
}

// QMC5883P Magnetometer ===================================================

void compassInit()
{
	I2C1.begin(I2CSDA, I2CSCL, I2C_FREQ);

	if(!compass.begin(QMC5883P_DEFAULT_ADDR, &I2C1))
	{
		#if __DEBUG__
			Serial.println("Failed to initialize QMC5883P compass!");
		#endif
		return;
	}
	
	compass.setMode(QMC5883P_MODE_NORMAL);

	xTaskCreatePinnedToCore
	(
		compassDataUpdate,
		"Compass Data Update",
		4096,
		NULL,
		1,
		NULL,
		COMPASS_LISTNER_CORE
	);

}

void compassDataUpdate(void *param)
{
	TickType_t lastUpdateTick = 0;

	while(true)
	{
		if(!compass.isDataReady())
		{
			#if __DEBUG__
				Serial.println("Compass data not ready, skipping update.");
			#endif
		}
		else
		{
			int16_t ix, iy, iz;
			float fx, fy, fz;
			float gx, gy, gz;
			compass.getRawMagnetic(&ix, &iy, &iz);
			fx = (float)ix;
			fy = (float)iy;
			fz = (float)iz;
			compass.getGaussField(&gx, &gy, &gz);

			float magG = sqrtf(gx * gx + gy * gy + gz * gz);
			if(magG > GausThreshold)
			{
				azimuth = atan2f(gy, gx) * 180.0 / M_PI;
			}

			#if __DEBUG__
				Serial.printf("Compass Update - Raw: (%d, %d, %d), Gauss: (%.2f, %.2f, %.2f), Mag: %.2f G\n", ix, iy, iz, gx, gy, gz, magG);
			#endif
		}
		xTaskDelayUntil(&lastUpdateTick, pdMS_TO_TICKS(COMPASS_UPDATE_INTERVAL_MS));
	}
}

// MFRC522 RFID Reader =====================================================

void mfrcInit()
{

}

void mfrcDataListener(void *param)
{

}