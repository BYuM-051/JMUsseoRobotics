#include <RTOS.h>
#define __DEBUG__ true
#define CurrentBoardID 0x00
#define CurrentBoard 0 // FIXME : define the current board index for ESP-NOW communication, should be set according to the actual board ID in MAC_ADDR array

// arduino core ============================================================
#include <Arduino.h>
void setup();
void loop();

// Project Board Info ======================================================
// Read Readme.md for more details about the board and pinout

/*
R 	3v3			
K 	GND		
W 	SCK			A0
G 	MISO		A1
B 	MOSI		A2
Br 	Front SDA	A3
Y 	Front RST	A4
Pu 	Center SDA	A5
O 	Center RST	A6

*/

#define MAX_BOARDS 4 

constexpr uint8_t MAC_ADDR [MAX_BOARDS][6] = 
{
	{0x20, 0x6E, 0xF1, 0x33, 0x4D, 0xD0}, // ROBOT BOARD 1

	{0x3C, 0x84, 0x27, 0xC3, 0xED, 0xF0}, // AI Board 1
	{0x48, 0xCA, 0x43, 0x2F, 0x77, 0x58}, // AI Board 2
	{0x3C, 0x84, 0x27, 0xC4, 0x3F, 0xF4} // AI Board 3
	// Add more boards as needed
};

// UART Communication ======================================================
#include "driver/uart.h"
#define MAX_SERIAL_LENGTH 64
#define MAX_PENDING_MESSAGES 10
#define UART_RX_BUFFER_SIZE (MAX_SERIAL_LENGTH * MAX_PENDING_MESSAGES)  // 640
#define UART_TX_BUFFER_SIZE (MAX_SERIAL_LENGTH * 4)                     // 256
#define UART_EVENT_QUEUE_SIZE 20

#define VEX UART_NUM_1
#define UART1_RX_PIN D4
#define UART1_TX_PIN D5

#define VoiceRecog UART_NUM_2
#define UART2_RX_PIN D9
#define UART2_TX_PIN D12

#define UART_LISTNER_CORE 0
#define UART_BAUD_RATE 115200
#define UART_BLOCK_TICKS pdMS_TO_TICKS(portMAX_DELAY)
#define VEX_UPDATE_INTERVAL_MS 50	// NOTE : 20hz update rate for VEX UART communication, can be adjusted based on performance needs
#define VEX_RFID_ALIGN_INTERVAL_MS 10 // NOTE : 100hz update rate for robot alignment after RFID tag detection, can be adjusted based on performance needs and robot's turning precision

#if __DEBUG__
	#undef UART_BLOCK_TICKS
	#define UART_BLOCK_TICKS pdMS_TO_TICKS(1000)
#endif

QueueHandle_t vexV5SerialQueue;
QueueHandle_t voiceSerialQueue;
volatile TickType_t lastVexSerialTick = 0;
volatile TickType_t lastVoiceSerialTick = 0;

volatile byte leftMotorOutput = 0;
volatile byte rightMotorOutput = 0;

typedef enum
{
	ROBOT_STOP = 0,
	ROBOT_MOVE_WITH_COMPASS,
	ROBOT_RFID_APPROACHING,
	ROBOT_RFID_ALIGNING
} vexAction_t;

volatile vexAction_t currentVexAction = ROBOT_STOP;

void uartInit();
void vexUartListener(void *param);
void vexUartUpdate(void);
void voiceUartListener(void *param);
void onSerialRecieved(const uart_port_t uart_num, const char* cmdText);
void uartPrintf(const uart_port_t uart_num, const char *format, ...);

// ESPNOW Communication ==================================================
#include <WiFi.h>
#include <esp_now.h>

#define ESP_NOW_CHANNEL 2U
#define ESP_NOW_LISTENER_CORE 0

typedef struct 
{
	uint8_t macAddr[6];
	uint8_t data[250];
	int len;
} espNowPacket_t;

QueueHandle_t espNowRecvQueue;

void espNowInit(void);
void espNowDataRecvCallback(const uint8_t *mac_addr, const uint8_t *data, int len);
void espNowDataSendCallback(const uint8_t *mac_addr, esp_now_send_status_t status);
void espNowListnener(void *param);
void onESPNowDataReceived(char *cmdText);
void espNowPrintf(const uint8_t *mac_addr, const char *format, ...);

// QMC5883P Magnetometer =================================================
#include "Adafruit_QMC5883P.h"
#include "Wire.h"

constexpr float GausThreshold = 0.1F; // FIXME : define the threshold for detecting magnet stick
#define I2CSDA D3
#define I2CSCL D2
#define I2C_FREQ 400000U

#define COMPASS_LISTNER_CORE 1
#define COMPASS_UPDATE_INTERVAL_MS 20	// NOTE : 50hz update rate, can be adjusted based on performance needs

TwoWire I2C1 = TwoWire(0);
Adafruit_QMC5883P compass;
volatile float azimuth = -9999.0F; // Invalid initial value to indicate uninitialized state

void compassInit();
void compassDataPolling(void *param);

// RC522 RFID Reader =====================================================
#include <SPI.h>
#include <MFRC522.h>

#define SPI_SCK_PIN A0
#define SPI_MOSI_PIN A2
#define SPI_MISO_PIN A1

#define Front_RST_PIN A4
#define Front_SS_PIN  A3
#define Center_RST_PIN A6
#define Center_SS_PIN  A5

#define MFRC_LISTNER_CORE 1
#define MFRC_UPDATE_INTERVAL_MS 100 // NOTE : 10hz update rate, can be adjusted based on performance needs
#define RFIDAlignDeadband 1.5F // FIXME : define the acceptable angle error for robot alignment after RFID tag detection, can be adjusted based on performance needs and robot's turning precision

MFRC522 frontRFID(Front_SS_PIN, Front_RST_PIN);
MFRC522 centerRFID(Center_SS_PIN, Center_RST_PIN);

typedef struct 
{
	byte uid[10];
	byte uidLength;
	float angle;
} RFIDData;


void mfrcInit();
void mfrcDataPolling(void *param);
void mfrcRobotAction(void *param);
void mfrcRobotAction(const RFIDData *data);

constexpr RFIDData myRFIDData[10] = //FIXME : define the UID for each RFID tag and the associated angle for robot actions
{
	{
		{0xDE, 0xAD, 0xBE, 0xEF},
		4,
		0.0F
	},
	{
		{0xBA, 0xAD, 0xF0, 0x0D},
		4,
		90.0F
	},
	{
		{0xFE, 0xED, 0xFA, 0xCE},
		4,
		180.0F
	}
};

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

	// Initialize ESP-NOW Communication===================================
	espNowInit();
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
				{
					size_t readLen = min(event.size, sizeof(data) - 1);
					int len = uart_read_bytes(VEX, data, readLen, UART_BLOCK_TICKS);
					if(len > 0)
					{
						data[len] = '\0';
						onSerialRecieved(VEX, (char *)data);
					}
					break;
				}
				case UART_BUFFER_FULL:
				{
					#if __DEBUG__
						Serial.println("VEX UART Buffer Full");
					#endif
					break;
				}
				default:
				{
				#if __DEBUG__
					Serial.printf("Unknown VEX UART Event Type : %d\n", event.type);
				#endif
					break;
			
				}
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

void vexUartUpdate(void)
{
	TickType_t currentTick = xTaskGetTickCount();
	digitalRead(1);
	switch(currentVexAction)
	{
		case ROBOT_STOP :
		{
			leftMotorOutput = 0;
			rightMotorOutput = 0;
			uartPrintf(VEX, "motor %d %d", leftMotorOutput, rightMotorOutput);
			break;
		}
		case ROBOT_MOVE_WITH_COMPASS :
		{
			//TODO : Implement motor control logic to move robot based on compass data and target angle
			break;
		}
		case ROBOT_RFID_APPROACHING :
		{
			leftMotorOutput = 10; // FIXME : set appropriate motor output for approaching action
			rightMotorOutput = 10; // FIXME : set appropriate motor output for approaching action
			uartPrintf(VEX, "motor %d %d", leftMotorOutput, rightMotorOutput);
			break;
		}
		case ROBOT_RFID_ALIGNING :
		{
			/* ================================================================================================
			this state used to skip uart update when robot is aligning to target angle after RFID tag detection
			check the mfrcRobotAction function. the robot control logic is there.
			================================================================================================ */
			break;
		}
	}

	vTaskDelayUntil(&currentTick, pdMS_TO_TICKS(VEX_UPDATE_INTERVAL_MS));
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
				{
					size_t readLen = min(event.size, sizeof(data) - 1);
					int len = uart_read_bytes(VoiceRecog, data, readLen, UART_BLOCK_TICKS);
					if(len > 0)
					{
						data[len] = '\0';
						onSerialRecieved(VoiceRecog, (char *)data);
					}
					break;
				}
				default:
				{
					#if __DEBUG__
						Serial.printf("Unknown Voice UART Event Type : %d\n", event.type);
					#endif
				}
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
		compassDataPolling,
		"Compass Data Polling",
		4096,
		NULL,
		1,
		NULL,
		COMPASS_LISTNER_CORE
	);

}

void compassDataPolling(void *param)
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
	SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
	frontRFID.PCD_Init();
	centerRFID.PCD_Init();

	xTaskCreatePinnedToCore
	(
		mfrcDataPolling,
		"MFRC Data Polling",
		4096,
		NULL,
		1,
		NULL,
		UART_LISTNER_CORE
	);
}

void mfrcDataPolling(void *param)
{
	TickType_t lastUpdateTick = 0;
	while(true)
	{
		if(frontRFID.PICC_IsNewCardPresent() && frontRFID.PICC_ReadCardSerial())
		{	
			#if __DEBUG__
				Serial.print("Front RFID Card Detected - UID: ");
				for(byte i = 0; i < frontRFID.uid.size; i++)
				{
					Serial.printf("%02X ", frontRFID.uid.uidByte[i]);
				}
				Serial.println();
			#endif
			currentVexAction = ROBOT_RFID_APPROACHING;
		}
		if(centerRFID.PICC_IsNewCardPresent() && centerRFID.PICC_ReadCardSerial())
		{
			currentVexAction = ROBOT_STOP; // Stop the robot before processing the detected RFID tag for alignment

			RFIDData detectedData;
			memcpy(detectedData.uid, centerRFID.uid.uidByte, centerRFID.uid.size);
			detectedData.uidLength = centerRFID.uid.size;
			detectedData.angle = 0.0F;
			#if __DEBUG__
				Serial.print("Center RFID Card Detected - UID: ");
				for(byte i = 0; i < centerRFID.uid.size; i++)
				{
					Serial.printf("%02X ", centerRFID.uid.uidByte[i]);
				}
				Serial.println();
			#endif
			mfrcRobotAction(&detectedData);
		}

		xTaskDelayUntil(&lastUpdateTick, pdMS_TO_TICKS(MFRC_UPDATE_INTERVAL_MS));
	}
}

void mfrcRobotAction(const RFIDData *data)
{
	byte uid[10];
	bool matchFound = false;
	memcpy(uid, data->uid, data->uidLength);
	for(const auto &entry : myRFIDData)
	{
		if(entry.uidLength == data->uidLength && memcmp(entry.uid, uid, data->uidLength) == 0)
		{
			float targetAngle = entry.angle;
			matchFound = true;
			#if __DEBUG__
				Serial.printf("Matched RFID Tag - UID: ");
				for(byte i = 0; i < entry.uidLength; i++)
				{
					Serial.printf("%02X ", entry.uid[i]);
				}
				Serial.printf(", Target Angle: %.2f\n", targetAngle);
			#endif
			currentVexAction = ROBOT_RFID_APPROACHING;

			//TODO : Implement logic to set target angle for robot alignment based on matched RFID tag data
			xTaskCreate
			(
				mfrcRobotAction, 
				"rfidAlignTask", 
				4096,
				NULL,
				1,
				NULL
			);
			
		}

		if(!matchFound)
		{
			#if __DEBUG__
				Serial.println("No matching RFID tag found in predefined data.");
			#endif
		}
	}
}

void mfrcRobotAction(void *param)
{
	TickType_t currentTick = xTaskGetTickCount();

	const float targetAngle = ((RFIDData *) param)->angle;

	while(azimuth == -9999.0F) vTaskDelayUntil(&currentTick, pdMS_TO_TICKS(50));

	while(abs(azimuth - targetAngle) > RFIDAlignDeadband)
	{
		//TODO : Implement control logic to adjust motor outputs based on current azimuth and target angle for alignment
		vTaskDelayUntil(&currentTick, pdMS_TO_TICKS(VEX_RFID_ALIGN_INTERVAL_MS));
	}
	vTaskDelete(NULL);
}

// ESPNOW Communication ====================================================
void espNowInit(void)
{
    WiFi.mode(WIFI_STA);
    if(esp_now_init() != ESP_OK)
    {
        #ifdef __DEBUG__
            Serial.println("ESP-NOW Initialization Failed");
        #endif
        return;
    }

    for(int i = 0 ; i < MAX_BOARDS ; i++)
    {
        esp_now_peer_info_t peerInfo = {};
        peerInfo.channel = ESP_NOW_CHANNEL;
        peerInfo.encrypt = false;
        if(i == CurrentBoard)	//FIXME : refactor with CurrentBoardID
        {
            peerInfo.peer_addr[0] = 0x02; // Locally Administered dummy Address
        }
        else
        {
            memcpy(peerInfo.peer_addr, MAC_ADDR[i], 6);
        }
        if(esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            #ifdef __DEBUG__
                Serial.println("Failed to add ESP-NOW peer");
            #endif
        }
        else
        {
            #ifdef __DEBUG__
            Serial.print("Added ESP-NOW peer: ");
            Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", peerInfo.peer_addr[0], peerInfo.peer_addr[1], peerInfo.peer_addr[2], peerInfo.peer_addr[3], peerInfo.peer_addr[4], peerInfo.peer_addr[5]);
            #endif
        }
}

    esp_now_register_recv_cb(espNowDataRecvCallback);
    esp_now_register_send_cb(espNowDataSendCallback);
    espNowRecvQueue = xQueueCreate(10, sizeof(espNowPacket_t));
    xTaskCreatePinnedToCore
    (
        espNowListnener,
        "espNowListener",
        4096,
        NULL,
        2,
        NULL,
        ESP_NOW_LISTENER_CORE
    );
}

void espNowDataRecvCallback(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espNowPacket_t buffer;
    memcpy(buffer.macAddr, mac_addr, 6);
    memcpy(buffer.data, data, len);
    buffer.len = len;
    xQueueSendFromISR(espNowRecvQueue, &buffer, NULL);
}

void espNowListnener(void *param)
{
    espNowPacket_t packet;
    while(true)
    {
        if(xQueueReceive(espNowRecvQueue, &packet, portMAX_DELAY))
        {
            #ifdef __DEBUG__
                Serial.println("ESP-NOW Data Received");
            #endif
            boolean isKnownSender = false;
            for(int i = 0 ; i < MAX_BOARDS ; i++)
            {
                if(memcmp(packet.macAddr, MAC_ADDR[i], 6) == 0)
                {
                    isKnownSender = true;
                    break;
                }
            }

            if(!isKnownSender)
            {
                #ifdef __DEBUG__
                    Serial.println("Unknown ESP-NOW Sender, Ignoring Packet");
                #endif
                continue;
            }
            else
            {
                if(packet.len > 0) 
                {
                    packet.data[packet.len] = '\0';
                    onESPNowDataReceived((char *)packet.data);
                }
                #ifdef __DEBUG__
                    else
                    {
                        Serial.println("Received Empty ESP-NOW Packet");
                    }
                #endif
            }
        }
    }
}

void onESPNowDataReceived(char *cmdText)
{
    if(strcmp(cmdText, "PONG") == 0)
    {
        // Ping Pong Command, Do nothing
    }
    else if(strcmp(cmdText, "") == 0)
    {

    }
    else
    {
        #ifdef __DEBUG__
            Serial.printf("Unknown ESP-NOW Command Received : %s\n", cmdText);
        #endif
    }
}

void espNowDataSendCallback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    #ifdef __DEBUG__
        Serial.print("ESP-NOW Data Send Callback, Status: ");
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failure");
    #endif
}

void espNowPrintf(const uint8_t *mac_addr, const char *format, ...)
{
    char buffer[250];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    esp_now_send(mac_addr, (uint8_t *)buffer, strlen(buffer));
}
