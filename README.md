Project FORCE System

Address Info :
ESP32	MAC Address
R0		
R1
R2
B0
B1
B2



W0 REST API URL : 

- EVAC Guide Robot [R0, R1, R2]

    R0 Actuating Robotics Module Powered by VEX V5
    - > Connect with R1 via UART over RS485
    - > Connect with R2 via UART over RS485
    - > V5 Smart Motor 11W * 2EA
    - > Human UI via VEX V5 touch screen
    Smart Port Info
    0 : 

    R1 Robot Control Module Powered by ESP32 S3 [Arduino Nano ESP32]
    - > Connect with W0 via HTTP over WiFi
    - > Connect with R0 via UART over TTL
    - > Node Sensing via Front RC522 RFID Reader
    - > Node Reading and rotating via Center RC522 RFID Reader
    - > Segment Tracking via QMC5883L 3-axis Magnetometer 
    GPIO Port Info
    0 : 

    R2 Robot Voice Assist Module Powered by ESP32 S3 [Seeed XIAO S3 Sense]
    - > Connect with R0 via UART over TTL
    - > Speech Recognition with ESP-SR via embedded mic on Seeed XIAO S3 Sense
    - > Text to speech with google TTS streaming via ESP32 wifi
    - > Speaker output with MAX98357A amplifier via I2S

- Building Monitoring System [B0, Bn]

    B0 Central Building Status Processor Powered by ESP32 S3 [Arduino Nano ESP32]
    - > Connect with W0 via HTTP over WiFi 
    - > Connect with Bn via ESP-NOW over WiFi
    
    Bn Peripheral Building Special Node Sensor by ESP32 S3 [Arduino Nano ESP32]
    - > Connect with B0 via ESP-NOW over WiFi
    - > Gas Sensor
    - > mmWave Human Tracking Sensor
    - > DHT-11

    Node Block Embedded NTAG215 RFID Card-Key
    - > has azimuth of branches to go 

    Segment Block Embedded Magnetic Guide Rail

- Building Information Mirroring System [W0]

    W0 Building Node and Segment Table Powered by Firebase
    - > 


## Mermaid DFD

### EVAC Guide Robot

```mermaid
flowchart LR
    U[User]
    F[(W0 Firebase)]
    G[Google TTS]

    RFID1[Front RC522 RFID Reader]
    RFID2[Center RC522 RFID Reader]
    MAG[QMC5883L Magnetometer]
    UI[VEX V5 Touch Screen]

    R1[R1\nRobot Control Module]
    R0[R0\nActuating Robotics Module]
    R2[R2\nVoice Assist Module]

    U -->|touch / local interaction| UI
    UI -->|UI input| R0
    R0 -->|screen feedback / alerts| UI

    RFID1 -->|front node sensing| R1
    RFID2 -->|center node reading| R1
    MAG -->|segment tracking data| R1

    F -->|route table / building node data| R1

    U -->|voice input| R2
    R2 -->|recognized speech / intent| R1
    R1 -->|response text| R2
    R2 -->|TTS request| G
    G -->|audio stream| R2
    R2 -->|voice guidance audio| U

    R1 -->|movement / rotation / stop command| R0
    R0 -->|actuator status / execution result| R1
```

### Building Monitoring System

```mermaid
flowchart LR
    F[(W0 Firebase)]
    B0[B0\nCentral Building Status Processor]
    BN1[Bn #1]
    BN2[Bn #2]
    BNX[Bn #n]

    GAS[Gas Sensor]
    MMW[mmWave Human Tracking]
    DHT[DHT-11]

    GAS --> BN1
    MMW --> BN1
    DHT --> BN1

    GAS --> BN2
    MMW --> BN2
    DHT --> BN2

    BN1 -->|sensor packet via ESP-NOW| B0
    BN2 -->|sensor packet via ESP-NOW| B0
    BNX -->|sensor packet via ESP-NOW| B0

    B0 -->|aggregated building status| F

```