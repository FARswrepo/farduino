#include <I2Cdev.h>
#include "Wire.h"
#include "MPU9250.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP280.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SPI.h>
#include "SdFat.h"
#include "RF24.h"

//all times in microseconds
#define MIN_ACCELERATION_TIME 400000
#define ONE_G 2000
#define ONE_SECOND 1000000
#define MIN_FLIGHT_TIME 7000000
#define MAX_FLIGHT_TIME 12000000
#define MOTOR_BURNOUT_TIME 4000000
#define PEAK_DISCRIMINATION_TIME 800000
#define MAX_SAMPLE_COUNT 100000
#define MIN_PRESSURE_DROP -1.0
#define PYRO0 PB5
#define PYRO1 PB4
#define PYRO2 PB3
#define PYRO3 PA15


#define OCTAVE_OFFSET 0
#define isdigit(n) (n >= '0' && n <= '9')

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low  = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69


typedef struct{
  double mean_a[3];
  double mean_w[3];
  double mean_B[3];
  
  double sigma_a[3];
  double sigma_w[3];
  double sigma_B[3];  
} inertial_measurement_t;


char clock_string_buffer[12];
char my_name[16];
char my_line[64];

unsigned int sample_count = 0;
unsigned int file_count = 0;

unsigned long last_micros = 0;
unsigned long base_seconds = 0;
unsigned long base_fraction = 0;
unsigned long launch_timestamp;
unsigned long peak_timestamp;

unsigned long timestamp_lookback[16];
unsigned long loop_count;

double sigma_pressure = 0.0;
double pressure_0 = 0.0;
double pressure_limit = 0.0;

double sum_p = 0.0;
double sum_p2 = 0.0;
int pressure_samples = 0;

double sigma_omega[3] = {0.0, 0.0, 0.0};
double omega_0[3] = {0.0, 0.0, 0.0};


int consecutive_count = 0;

double peak_pressure;
double peak_altitude;
double max_acc;
double base_time = 0.0;

double temperature;
double pressure;
double altitude;
double raw_acc[3];
double raw_omega[3];
double raw_B[3];
double acc;
double omega;


const int tonePin = PA2;

int notes[] = { 0,
262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,
1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976,
2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951
};

char *simpsons_song = "The Simpsons:d=4,o=5,b=160:c.6,e6,f#6,8a6,g.6,e6,c6,8a,8f#,8f#,8f#,2g,8p,8p,8f#,8f#,8f#,8g,a#.,8c6,8c6,8c6,c6";
char *indiana_song = "Indiana:d=4,o=5,b=250:e,8p,8f,8g,8p,1c6,8p.,d,8p,8e,1f,p.,g,8p,8a,8b,8p,1f6,p,a,8p,8b,2c6,2d6,2e6,e,8p,8f,8g,8p,1c6,p,d6,8p,8e6,1f.6,g,8p,8g,e.6,8p,d6,8p,8g,e.6,8p,d6,8p,8g,f.6,8p,e6,8p,8d6,2c6";
char *take_song = "TakeOnMe:d=4,o=4,b=160:8f#5,8f#5,8f#5,8d5,8p,8b,8p,8e5,8p,8e5,8p,8e5,8g#5,8g#5,8a5,8b5,8a5,8a5,8a5,8e5,8p,8d5,8p,8f#5,8p,8f#5,8p,8f#5,8e5,8e5,8f#5,8e5,8f#5,8f#5,8f#5,8d5,8p,8b,8p,8e5,8p,8e5,8p,8e5,8g#5,8g#5,8a5,8b5,8a5,8a5,8a5,8e5,8p,8d5,8p,8f#5,8p,8f#5,8p,8f#5,8e5,8e5";
char *entertainer_song = "Entertainer:d=4,o=5,b=140:8d,8d#,8e,c6,8e,c6,8e,2c.6,8c6,8d6,8d#6,8e6,8c6,8d6,e6,8b,d6,2c6,p,8d,8d#,8e,c6,8e,c6,8e,2c.6,8p,8a,8g,8f#,8a,8c6,e6,8d6,8c6,8a,2d6";
//char *song = "Muppets:d=4,o=5,b=250:c6,c6,a,b,8a,b,g,p,c6,c6,a,8b,8a,8p,g.,p,e,e,g,f,8e,f,8c6,8c,8d,e,8e,8e,8p,8e,g,2p,c6,c6,a,b,8a,b,g,p,c6,c6,a,8b,a,g.,p,e,e,g,f,8e,f,8c6,8c,8d,e,8e,d,8d,c";
char *xfiles_song = "Xfiles:d=4,o=5,b=125:e,b,a,b,d6,2b.,1p,e,b,a,b,e6,2b.,1p,g6,f#6,e6,d6,e6,2b.,1p,g6,f#6,e6,d6,f#6,2b.,1p,e,b,a,b,d6,2b.,1p,e,b,a,b,e6,2b.,1p,e6,2b.";
//char *song = "Looney:d=4,o=5,b=140:32p,c6,8f6,8e6,8d6,8c6,a.,8c6,8f6,8e6,8d6,8d#6,e.6,8e6,8e6,8c6,8d6,8c6,8e6,8c6,8d6,8a,8c6,8g,8a#,8a,8f";
//char *song = "20thCenFox:d=16,o=5,b=140:b,8p,b,b,2b,p,c6,32p,b,32p,c6,32p,b,32p,c6,32p,b,8p,b,b,b,32p,b,32p,b,32p,b,32p,b,32p,b,32p,b,32p,g#,32p,a,32p,b,8p,b,b,2b,4p,8e,8g#,8b,1c#6,8f#,8a,8c#6,1e6,8a,8c#6,8e6,1e6,8b,8g#,8a,2b";
//char *song = "Bond:d=4,o=5,b=80:32p,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d#6,16d#6,16c#6,32d#6,32d#6,16d#6,8d#6,16c#6,16c#6,16c#6,16c#6,32e6,32e6,16e6,8e6,16d#6,16d6,16c#6,16c#7,c.7,16g#6,16f#6,g#.6";
//char *song = "MASH:d=8,o=5,b=140:4a,4g,f#,g,p,f#,p,g,p,f#,p,2e.,p,f#,e,4f#,e,f#,p,e,p,4d.,p,f#,4e,d,e,p,d,p,e,p,d,p,2c#.,p,d,c#,4d,c#,d,p,e,p,4f#,p,a,p,4b,a,b,p,a,p,b,p,2a.,4p,a,b,a,4b,a,b,p,2a.,a,4f#,a,b,p,d6,p,4e.6,d6,b,p,a,p,2b";
//char *song = "StarWars:d=4,o=5,b=45:32p,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#.6,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#6";
//char *song = "GoodBad:d=4,o=5,b=56:32p,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,d#,32a#,32d#6,32a#,32d#6,8a#.,16f#.,16g#.,c#6,32a#,32d#6,32a#,32d#6,8a#.,16f#.,32f.,32d#.,c#,32a#,32d#6,32a#,32d#6,8a#.,16g#.,d#";
char *topgun_song = "TopGun:d=4,o=4,b=31:32p,16c#,16g#,16g#,32f#,32f,32f#,32f,16d#,16d#,32c#,32d#,16f,32d#,32f,16f#,32f,32c#,16f,d#,16c#,16g#,16g#,32f#,32f,32f#,32f,16d#,16d#,32c#,32d#,16f,32d#,32f,16f#,32f,32c#,g#";
//char *song = "A-Team:d=8,o=5,b=125:4d#6,a#,2d#6,16p,g#,4a#,4d#.,p,16g,16a#,d#6,a#,f6,2d#6,16p,c#.6,16c6,16a#,g#.,2a#";
//char *song = "Flinstones:d=4,o=5,b=40:32p,16f6,16a#,16a#6,32g6,16f6,16a#.,16f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c6,d6,16f6,16a#.,16a#6,32g6,16f6,16a#.,32f6,32f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c6,a#,16a6,16d.6,16a#6,32a6,32a6,32g6,32f#6,32a6,8g6,16g6,16c.6,32a6,32a6,32g6,32g6,32f6,32e6,32g6,8f6,16f6,16a#.,16a#6,32g6,16f6,16a#.,16f6,32d#6,32d6,32d6,32d#6,32f6,16a#,16c.6,32d6,32d#6,32f6,16a#,16c.6,32d6,32d#6,32f6,16a#6,16c7,8a#.6";
char *jeopardy_song = "Jeopardy:d=4,o=6,b=125:c,f,c,f5,c,f,2c,c,f,c,f,a.,8g,8f,8e,8d,8c#,c,f,c,f5,c,f,2c,f.,8d,c,a#5,a5,g5,f5,p,d#,g#,d#,g#5,d#,g#,2d#,d#,g#,d#,g#,c.7,8a#,8g#,8g,8f,8e,d#,g#,d#,g#5,d#,g#,2d#,g#.,8f,d#,c#,c,p,a#5,p,g#.5,d#,g#";
//char *song = "Gadget:d=16,o=5,b=50:32d#,32f,32f#,32g#,a#,f#,a,f,g#,f#,32d#,32f,32f#,32g#,a#,d#6,4d6,32d#,32f,32f#,32g#,a#,f#,a,f,g#,f#,8d#";
//char *song = "Smurfs:d=32,o=5,b=200:4c#6,16p,4f#6,p,16c#6,p,8d#6,p,8b,p,4g#,16p,4c#6,p,16a#,p,8f#,p,8a#,p,4g#,4p,g#,p,a#,p,b,p,c6,p,4c#6,16p,4f#6,p,16c#6,p,8d#6,p,8b,p,4g#,16p,4c#6,p,16a#,p,8b,p,8f,p,4f#";
//char *song = "MahnaMahna:d=16,o=6,b=125:c#,c.,b5,8a#.5,8f.,4g#,a#,g.,4d#,8p,c#,c.,b5,8a#.5,8f.,g#.,8a#.,4g,8p,c#,c.,b5,8a#.5,8f.,4g#,f,g.,8d#.,f,g.,8d#.,f,8g,8d#.,f,8g,d#,8c,a#5,8d#.,8d#.,4d#,8d#.";
//char *song = "LeisureSuit:d=16,o=6,b=56:f.5,f#.5,g.5,g#5,32a#5,f5,g#.5,a#.5,32f5,g#5,32a#5,g#5,8c#.,a#5,32c#,a5,a#.5,c#.,32a5,a#5,32c#,d#,8e,c#.,f.,f.,f.,f.,f,32e,d#,8d,a#.5,e,32f,e,32f,c#,d#.,c#";
char *impossible_song = "MissionImp:d=16,o=6,b=95:32d,32d#,32d,32d#,32d,32d#,32d,32d#,32d,32d,32d#,32e,32f,32f#,32g,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,g,8p,g,8p,a#,p,c7,p,g,8p,g,8p,f,p,f#,p,a#,g,2d,32p,a#,g,2c#,32p,a#,g,2c,a#5,8c,2p,32p,a#5,g5,2f#,32p,a#5,g5,2f,32p,a#5,g5,2e,d#,8d";

byte flight_address[6] = "FAR01";
byte ground_address[6] = "FAR02";


unsigned char GPS_checksum = 0;
byte unsigned GPS_pointer = 0;
bool receiving_GPS;
char GPS_sentence[128];

//the pressure sensor is necessary
Adafruit_BMP280 met;

MPU9250 imu_mpu9250;
bool mpu9250_present = false;

Adafruit_BNO055 imu_bno055 = Adafruit_BNO055(-1, 0x28);
bool bno055_present = false;

RF24 radio_nrf24(PA8,PA4);
bool nrf24l01_present = false;


//SdFat constants and variables
const uint8_t chipSelect = PB12;
SPIClass spi2(2);
SdFat sd(&spi2);
bool SD_present = false;


SdFile dataFile;
bool file_exists;

typedef enum {
  state_IDLE,
  state_COASTING,
  state_PEAK_REACHED,
  state_FALLING,
  state_DROGUE_OPENED,
  state_LANDED
} rocket_state_t;

rocket_state_t current_state;

// SETUP routine

void setup() {

  double altitude;
  unsigned long imu_timestamp;
  unsigned long met_timestamp;
  double sigma_w;
  
  unsigned long mean_count;

  inertial_measurement_t imu_data;

  pinMode(PYRO0, OUTPUT);
  digitalWrite(PYRO0, LOW);
  pinMode(PYRO1, OUTPUT);
  digitalWrite(PYRO1, LOW);
  pinMode(PYRO2, OUTPUT);
  digitalWrite(PYRO2, LOW);  
  pinMode(PYRO3, OUTPUT);
  digitalWrite(PYRO3, LOW);

  // join I2C bus
  Wire.begin();

  //
  Serial.begin(115200);
  
  
  //serial port #1 set to 9600 8n1 for GPS data stream
  Serial1.begin(9600);

  delay(2000);
  Serial.println(F("FARduino Maple v0.2"));
  
  
  nrf24l01_present = radio_nrf24.begin();
  if (nrf24l01_present){
    Serial.println(F("Initializing nrf24L01+"));
    radio_nrf24.setPALevel(RF24_PA_LOW);
    radio_nrf24.openWritingPipe(ground_address);
    //radio_nrf24.openReadingPipe(1,flight_address);
    radio_nrf24.stopListening();
  }
  else{
    Serial.println(F("<!> nrf24L01+ not detected"));
  }
  
  //initialize I2C devices
  
  mpu9250_present = imu_mpu9250.testConnection();
  if (mpu9250_present){
    Serial.println(F("Initializing IMU MPU9250"));
    imu_mpu9250.initialize();
    //set gyroscope scale to +/-2000°/s
    imu_mpu9250.setFullScaleGyroRange(MPU9250_GYRO_FS_2000);
    //set acceleration scale to +/-16g
    imu_mpu9250.setFullScaleAccelRange(MPU9250_ACCEL_FS_16);
  }
  else{  
    Serial.println( "<!> MPU9250 not detected");
  }
  
  bno055_present = imu_bno055.begin();
  if (bno055_present){
	Serial.println("BNO055 initialized");
	imu_bno055.setExtCrystalUse(true);
  }	  
  else
  {
    Serial.println("<!> BNO055 not detected");
  }
  
  Serial.println(F("Initializing BMP280 pressure sensor at 0x76"));
  if (!met.begin(0x76)) {
    Serial.println(F("<!> BMP280 does not respond"));
    exit(-1);
  }

  // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    Serial.println("<!> no SD card found");
    SD_present = false;
    //exit(-1);
  }
  else {
    SD_present = true;
  }

  if (SD_present) {
    //search for next free file dataXXXX.txt
    do {
      sprintf(my_name, "data%04d.txt", file_count++);
      Serial.print("checking ");
      Serial.println(my_name);

      file_exists = !dataFile.open(my_name, O_CREAT | O_WRITE | O_EXCL);
      if (file_exists) {
        Serial.println("file exists.");
      }
      else {
        Serial.println("new file created");
      }
    }
    while (file_exists);
  }

  //start state machine in IDLE
  current_state = state_IDLE;

  String serial_line;
  
  do {
    mean_pressure(256, pressure_0, sigma_pressure);
  } while ((sigma_pressure<0.02)||(sigma_pressure>0.05));

  consecutive_count = 0;
  pressure_limit = -5*sigma_pressure;

  dtostrf(pressure_0, 4, 3, my_line);
  serial_line = my_line;
  
  Serial.print("p_0 = (");
  Serial.print(serial_line);
  dtostrf(sigma_pressure, 4, 3, my_line);
  serial_line = my_line;
  
  Serial.print("+/-");
  Serial.print(serial_line);
  Serial.println(")");
  
  play_rtttl(entertainer_song);  
  
  loop_count = 0;
}



//central LOOP

void loop() {

  char sentence[100];

  unsigned long imu_timestamp;
  unsigned long met_timestamp;
  unsigned long flight_time;
  unsigned long mean_count;

  double delta_pressure; 

  if (mpu9250_present){
    get_mpu9250_data(imu_timestamp, raw_acc[0], raw_acc[1], raw_acc[2], raw_omega[0], raw_omega[1], raw_omega[2], raw_B[0], raw_B[1], raw_B[2]);
    construct_IMU_sentence(imu_timestamp, raw_acc, raw_omega, raw_B, &sentence[0]);
    //Serial1.print(&sentence[0]);
    if (SD_present){
      dataFile.print(&sentence[0]);
    }
  }
  
  if (bno055_present){
	  get_bno055_data(imu_timestamp, raw_acc[0], raw_acc[1], raw_acc[2], raw_omega[0], raw_omega[1], raw_omega[2], raw_B[0], raw_B[1], raw_B[2]);
    construct_IMU_sentence(imu_timestamp, raw_acc, raw_omega, raw_B, &sentence[0]);
    Serial.print(&sentence[0]);
    if (SD_present){
      dataFile.print(&sentence[0]);
    }
  }	  
  
  if ((loop_count % 10)==0){
    get_MET_data(met_timestamp, temperature, pressure, altitude);
    construct_MET_sentence(met_timestamp, pressure, temperature, altitude, &sentence[0]);
    Serial.print(&sentence[0]);
    if (SD_present){
      dataFile.print(&sentence[0]); 
    }
  }
  
  bool gps_available = get_GPS_data();
  if (gps_available){
	Serial.print(&sentence[0]);  
    if (SD_present){
	  dataFile.print(&GPS_sentence[0]);
	}  
  }

  sample_count++;

  if (SD_present) {
    // Force data to SD and update the directory entry to avoid data loss.
    if (!dataFile.sync() || dataFile.getWriteError()) {
      sd.errorHalt(F("write error"));
    }

    if ((sample_count >= MAX_SAMPLE_COUNT) && (current_state == state_IDLE)) {
      dataFile.close();

      do {
        sprintf(my_name, "data%04d.txt", file_count++);
        Serial.print("creating file ");
        Serial.println(my_name);
        file_exists = !dataFile.open(my_name, O_CREAT | O_WRITE | O_EXCL);
        if (file_exists) {
          Serial.println("file exists.");
        }
        else {
          Serial.println("new file created");
        }
      }
      while (file_exists);

      sample_count = 0;
    }
  }

  acc = sqrt(raw_acc[0] * raw_acc[0] + raw_acc[1] * raw_acc[1] + raw_acc[2] * raw_acc[2]);
  omega = sqrt(raw_omega[0]*raw_omega[0] + raw_omega[1]*raw_omega[1] + raw_omega[2]*raw_omega[2]);
  bool state_changed = false;

  if (current_state != state_IDLE) {
    flight_time = calc_flight_time(launch_timestamp, met_timestamp);
  }
  
  delta_pressure = pressure - pressure_0;

  unsigned long state_timestamp;
  if ((loop_count%10)==0){
  switch (current_state) {

    case state_IDLE: {        

        //check if pressure below 5 sigma level
        if (delta_pressure < pressure_limit){
          //reset ground pressure calculation
          sum_p = 0.0;
          sum_p2 = 0.0;
          pressure_samples = 0;

          //store first timestamp as possible launch time  
          if (consecutive_count== 0){
            launch_timestamp = met_timestamp;
          } 
          consecutive_count++;

          //wait for at least 16 consecutive measurements and a minimum of 1mBar pressure drop
          if ((consecutive_count>16) && (delta_pressure<MIN_PRESSURE_DROP)){
            //store current time and switch to next state
            state_timestamp = met_timestamp;
            current_state = state_COASTING;
            //remember last pressure/altitude and time as peak parameters
            peak_pressure = pressure;
            peak_altitude = altitude;
            peak_timestamp = met_timestamp;
            state_changed = true;
          }
        }
        else{
          //calculate mean pressure at ground level
          consecutive_count = 0;
          sum_p += pressure;
          sum_p2 += pressure*pressure;
          pressure_samples += 1;

          if (pressure_samples == 256){
            double new_pressure_0 = sum_p/pressure_samples;
            double new_sigma_pressure = sqrt(sum_p2/pressure_samples - new_pressure_0*new_pressure_0);

            String serial_line;

            dtostrf(new_sigma_pressure, 4, 3, my_line);
            serial_line = my_line;
            Serial.print("delta p : ");
            Serial.println(serial_line);
            
            if ((new_sigma_pressure>0.01) && (new_sigma_pressure<0.03)){
              pressure_0 = new_pressure_0;
              sigma_pressure = new_sigma_pressure;
              pressure_limit = -5*sigma_pressure;    
  
              //dtostrf(pressure_0, 4, 3, my_line);
              //serial_line = my_line;
              //Serial.print("p_0 = (");
              //Serial.print(serial_line);
              //dtostrf(sigma_pressure, 4, 3, my_line);
              //serial_line = my_line;
              //Serial.print("+/-");
              //Serial.print(serial_line);
              //Serial.println(")");			  
            }
            sum_p = 0.0;
            sum_p2 = 0.0;
            pressure_samples = 0;
            
            
          }
        }
        
        break;
      }


    case state_COASTING: { 

      if (flight_time > MAX_FLIGHT_TIME){
        current_state = state_PEAK_REACHED;
        state_timestamp = peak_timestamp;
        state_changed = true;                      
      }  
           
      //search for peak with barometer
       
      if (pressure < peak_pressure) {
        peak_pressure = pressure;
        peak_altitude = altitude;
        peak_timestamp = met_timestamp;
      }
        
         
      if ((flight_time > MIN_FLIGHT_TIME)&&(pressure>peak_pressure)){
        
          float peak_delta = calc_flight_time(peak_timestamp, met_timestamp);
          
          if (peak_delta > PEAK_DISCRIMINATION_TIME) {
            current_state = state_PEAK_REACHED;
            state_timestamp = peak_timestamp;
            state_changed = true;         
            tone(PA2,1000,500);             
          }
      }
      
      break;
    }

    case state_PEAK_REACHED: {
      //activate two pyros
      digitalWrite(PYRO0, HIGH);
      digitalWrite(PYRO1, HIGH);        
      current_state = state_FALLING;
      state_timestamp = met_timestamp;
      state_changed = true;
      break;
    }

    case state_FALLING: {

          current_state = state_DROGUE_OPENED;
          state_timestamp = met_timestamp;
          state_changed = true;
        
        break;
      }

    case state_DROGUE_OPENED: {
        
        if (delta_pressure > pressure_limit) {
          current_state = state_LANDED;
          state_timestamp = met_timestamp;
          state_changed = true;
        }
        break;
      }

    case state_LANDED: {
        digitalWrite(PYRO0, LOW);
        digitalWrite(PYRO1, LOW);        
      
        //current_state = state_IDLE;
        state_timestamp = imu_timestamp;
        //state_changed = true;
        delay(3000);
        play_rtttl(indiana_song);
        break;
      }
      
  }

  if (state_changed) {
    construct_state_sentence(state_timestamp, acc, current_state, &sentence[0]);
    //Serial1.print(&sentence[0]);
    if (SD_present) {
      dataFile.print(&sentence[0]);
    }
  }
  }
  
  loop_count++;

}


unsigned long calc_flight_time(unsigned long start_time, unsigned long stop_time) {

  unsigned long flight_time;

  if (start_time <= stop_time) {
    flight_time = stop_time - start_time;
  }
  else {
    flight_time = (4294967295 - start_time) + stop_time;
  }

  return flight_time;
}


//writes time of day into given buffer, always 11 chars long
void time_of_day(unsigned long timestamp, char *destination) {

  int current_hour;
  int current_minute;
  int current_second;

  //unsigned long delta_microsecond;
  //float delta_second;
  //float current_time = base_time + (double)timestamp/1000000.0;

  unsigned long timestamp_seconds = timestamp/1000000;
  unsigned long timestamp_fraction = timestamp%1000000;

  unsigned long current_fraction = base_fraction + timestamp_fraction;
  current_second = base_seconds + timestamp_seconds + current_fraction/1000000;
  current_fraction = (current_fraction%1000000)/100;
  
    
  current_hour =  current_second / 3600;
  current_second = current_second % 3600;
  current_minute = current_second / 60;
  current_second %= 60;

  sprintf(&destination[0], "%02i", current_hour);
  sprintf(&destination[2], "%02i", current_minute);
  sprintf(&destination[4], "%02i", current_second);
  destination[6] = '.';
  sprintf(&destination[7], "%04i", current_fraction);
  destination[11] = 0;
  
}



void construct_IMU_sentence (unsigned long timestamp, double my_acc[3], double my_gyro[3], double my_magn[3], char* sentence_buffer) {

  char *buffer_start;                          //pointer to timestamp string
  
  unsigned char xor_checksum;                 //simple XOR checksum
  int k, l;
  char *my_pointer;                            //pointer for checksum calculation

  buffer_start = sentence_buffer+8;            //remember beginning of sentence after first word
  my_pointer = sentence_buffer + 1;            //set pointer to first charcater after '$' for checksum calculation at the end of sentence construction
  sprintf(sentence_buffer, "$RQIMU0,");        //write sentence keyword to buffer
  sentence_buffer += 8;                        //adjust pointer to next free entry
  time_of_day(timestamp, sentence_buffer);        //construct string from timestamp
  sentence_buffer += 11;                       //adjust pointer to next free entry 
  
  *sentence_buffer++ = ',';                    //add separator and adjust pointer

  for (int i = 0; i < 3; i++) {                //loop through acceleration vector 
    dtostrf(my_acc[i], 6, 2, sentence_buffer);
    sentence_buffer += 6;
    *sentence_buffer++ = ',';
  }

  for (int i = 0; i < 3; i++) {                //loop through angular rate vector  
    dtostrf(my_gyro[i], 6, 2, sentence_buffer);
    sentence_buffer += 6;
    *sentence_buffer++ = ',';
  }

  for (int i = 0; i < 2; i++) {                //loop through magnetic field vector  
    dtostrf(my_magn[i], 6, 2, sentence_buffer);
    sentence_buffer += 6;
    *sentence_buffer++ = ',';
  }
  
  dtostrf(my_magn[2], 6, 2, sentence_buffer);
  sentence_buffer += 6;
  

  xor_checksum = 0;                            //calculate checksum  
  while (my_pointer != sentence_buffer) {
    xor_checksum ^= *my_pointer++;
  }
                                               //and append it to the sentence in hex format 
  sprintf(sentence_buffer, "*%02X", xor_checksum);
  sentence_buffer += 3;
  *sentence_buffer++ = 0x0d;                  //NMEA sentence termination: $0D $0A
  *sentence_buffer++ = 0x0a;
  *sentence_buffer = 0;                       //terminate string

  remove_spaces(buffer_start);
}


void remove_spaces(char* my_buffer){

  char* my_pointer;

    while (*my_buffer != 0){
    if (*my_buffer==0x20){
      my_pointer = my_buffer;
      while (*my_pointer!=0){
        *my_pointer = *(my_pointer+1);
        my_pointer++;    
      }
    }
    else{
      my_buffer++;
    }  
  }

}

void construct_MET_sentence(unsigned long timestamp, double p, double T, double h, char* sentence_buffer) {

  char *buffer_start;                          //pointer to timestamp string
  char *time_string;
  unsigned char xor_checksum;
  int k, l;
  char *checksum_pointer;


  buffer_start = sentence_buffer;
  checksum_pointer = sentence_buffer + 1;
  sprintf(sentence_buffer, "$RQMET0,");
  sentence_buffer += 8;
  time_of_day(timestamp, sentence_buffer);
  sentence_buffer += 11;
  *sentence_buffer++ = ',';
  dtostrf(p, 7, 3, sentence_buffer);
  sentence_buffer += 7;
  *sentence_buffer++ = ',';

  dtostrf(T, 5, 2, sentence_buffer);
  sentence_buffer += 5;
  *sentence_buffer++ = ',';

  dtostrf(h, 7, 2, sentence_buffer);
  sentence_buffer += 6;

  xor_checksum = 0;
  while (checksum_pointer != sentence_buffer) {
    xor_checksum ^= *checksum_pointer++;
  }

  sprintf(sentence_buffer, "*%02X", xor_checksum);
  sentence_buffer += 3;
  *sentence_buffer++ = 0x0d;
  *sentence_buffer++ = 0x0a;
  *sentence_buffer = 0;  //terminate string
  remove_spaces(buffer_start);
}


void construct_state_sentence(unsigned long timestamp, double& my_acc, rocket_state_t my_state, char* sentence_buffer) {

  char *time_string;
  unsigned char xor_checksum;
  int k, l;
  char *checksum_pointer;

  checksum_pointer = sentence_buffer + 1;
  sprintf(sentence_buffer, "$RQSTATE,");
  sentence_buffer += 9;
  time_of_day(timestamp, sentence_buffer);
  sentence_buffer += 11;
  
  *sentence_buffer++ = ',';

  switch (my_state) {

    case state_IDLE: {
        sprintf(sentence_buffer, "IDLE,");
        sentence_buffer += 5;
        dtostrf(pressure_0, 7, 3, sentence_buffer);
        sentence_buffer += 7;        
        break;
      }

    case state_COASTING: {
        sprintf(sentence_buffer, "COASTING,");
        sentence_buffer += 9;
        dtostrf(pressure, 7, 2, sentence_buffer);
        sentence_buffer += 7;
        *sentence_buffer++ = ',';
        dtostrf(pressure_0, 7, 2, sentence_buffer);
        sentence_buffer += 7;
        break;
      }

    case state_PEAK_REACHED: {
        sprintf(sentence_buffer, "PEAK_REACHED,");
        sentence_buffer += 13;
        dtostrf(peak_pressure, 7, 2, sentence_buffer);
        sentence_buffer += 7;
        *sentence_buffer++ = ',';
        dtostrf(peak_altitude, 7, 2, sentence_buffer);
        sentence_buffer += 7;
        break;
      }

    case state_FALLING: {
        sprintf(sentence_buffer, "FALLING");
        sentence_buffer += 7;
        break;
      }


    case state_DROGUE_OPENED: {
        sprintf(sentence_buffer, "DROGUE_OPENED");
        sentence_buffer += 13;
        break;
      }

    case state_LANDED: {
        sprintf(sentence_buffer, "LANDED");
        sentence_buffer += 6;
        break;
      }
  }

  xor_checksum = 0;
  while (checksum_pointer != sentence_buffer) {
    xor_checksum ^= *checksum_pointer++;
  }

  sprintf(sentence_buffer, "*%02X", xor_checksum);
  sentence_buffer += 3;
  *sentence_buffer++ = 0x0d;
  *sentence_buffer++ = 0x0a;
  *sentence_buffer = 0;  //terminate string
}


  bool get_GPS_data(void){

  char cipher = 0;
  char first_char;
  char second_char;
  unsigned char received_checksum;

  while (Serial1.available()){
    cipher = Serial1.read();

    if (cipher == '$'){
      GPS_checksum = 0;
      GPS_pointer = 0;
      receiving_GPS = true;
    }

    GPS_sentence[GPS_pointer] = cipher;
    if (GPS_pointer<128){
      GPS_pointer++;
    }

    if (cipher == '*'){
      receiving_GPS = false;
    }
    else if ((cipher != '$') && receiving_GPS){
      GPS_checksum ^= cipher;
    }
    else if (cipher == 0x0A){
      first_char = GPS_sentence[GPS_pointer-4];
      second_char = GPS_sentence[GPS_pointer-3];

      if (first_char>'9'){
        first_char = first_char - 'A' +10;
      }
      else{
        first_char = first_char - '0';
      }

      if (second_char>'9'){
        second_char = second_char - 'A' +10;
      }
      else{
        second_char = second_char - '0';
      }

      received_checksum = (first_char<<4) + second_char;

      
      Serial1.println(received_checksum,HEX);
      Serial1.println(GPS_checksum,HEX);

      if (GPS_checksum == received_checksum){
        GPS_sentence[GPS_pointer] = 0;
        return true;
      }
      else{
        return false;
      }

    }
  }
  return false;
}


unsigned long get_timestamp(){

  unsigned long current_micros = micros();
  if (current_micros<last_micros){
    base_seconds += 4294;
    base_fraction += 967295;
  }
  last_micros = current_micros;

  return current_micros;  
}


void get_mpu9250_data(unsigned long& timestamp, double& acc_x, double& acc_y, double& acc_z, double& omega_x, double& omega_y, double& omega_z, double& mag_x, double& mag_y, double& mag_z) {

  int16_t ax, ay, az;
  int16_t wx, wy, wz;
  int16_t Bx, By, Bz;

  timestamp = get_timestamp();
  imu_mpu9250.getMotion9(&ax, &ay, &az, &wx, &wy, &wz, &Bx, &By, &Bz);

  acc_x = (double)ax;
  acc_y = (double)ay;
  acc_z = (double)az;

  omega_x = (double)wx - omega_0[0];
  omega_y = (double)wy - omega_0[1];
  omega_z = (double)wz - omega_0[2];

  mag_x = (double)Bx;
  mag_y = (double)By;
  mag_z = (double)Bz;
}


void get_bno055_data(unsigned long& timestamp, double& acc_x, double& acc_y, double& acc_z, double& omega_x, double& omega_y, double& omega_z, double& mag_x, double& mag_y, double& mag_z) {

  sensors_event_t magneticData, angVelocityData, accelData;

  timestamp = get_timestamp();
  
  imu_bno055.getEvent(&magneticData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu_bno055.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_bno055.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  acc_x = accelData.acceleration.x;
  acc_y = accelData.acceleration.y;
  acc_z = accelData.acceleration.z;

  omega_x = angVelocityData.gyro.x;
  omega_y = angVelocityData.gyro.y;
  omega_z = angVelocityData.gyro.z;

  mag_x = magneticData.magnetic.x;
  mag_y = magneticData.magnetic.y;
  mag_z = magneticData.magnetic.z;
}





void get_MET_data(unsigned long& timestamp, double& temperature, double& pressure, double& altitude) {

  timestamp = get_timestamp();
  temperature = met.readTemperature();       
  pressure    = met.readPressure()/100.0;    
  altitude    = met.readAltitude(pressure_0);
}


void mean_pressure(int n, double& mean_p, double& sigma_p){

  double sum  = 0.0;
  double sum2 = 0.0;

  unsigned long timestamp;
  double temp;
  double p;
  double h;

  for (int k=0; k<n; k++){
    get_MET_data(timestamp, temp, p, h);
    
    sum  += p;
    sum2 += p*p;
  }

  mean_p  = sum/n;
  sigma_p = sqrt((sum2 - n*mean_p*mean_p)/(n-1));
  
}


void mean_inertial(int n, inertial_measurement_t& data){

  double sum_acc[3]  = {0.0, 0.0, 0.0};
  double sum_acc2[3] = {0.0, 0.0, 0.0};

  double sum_w[3]  = {0.0, 0.0, 0.0};
  double sum_w2[3] = {0.0, 0.0, 0.0};

  double sum_B[3]  = {0.0, 0.0, 0.0};
  double sum_B2[3] = {0.0, 0.0, 0.0};

  

  unsigned long timestamp;
  double acc[3];
  double omega[3];
  double B[3];

  for (int k=0; k<n; k++){
    get_mpu9250_data(timestamp, acc[0], acc[1], acc[2], omega[0], omega[1], omega[2], B[0], B[1], B[2]);

    for (int j=0; j<3; j++){
      sum_acc[j]  += acc[j];
      sum_acc2[j] += acc[j]*acc[j];

      sum_w[j]  += omega[j];
      sum_w2[j] += omega[j]*omega[j];
      
      sum_B[j]  += B[j];
      sum_B2[j] += B[j]*B[j];
    }  
  }


  for (int j=0; j<3; j++){
    data.mean_a[j]  = sum_acc[j]/n;
    data.sigma_a[j] = sqrt((sum_acc2[j] - n*data.mean_a[j]*data.mean_a[j])/(n-1));

    data.mean_w[j]  = sum_w[j]/n;
    data.sigma_w[j] = sqrt((sum_w2[j] - n*data.mean_w[j]*data.mean_w[j])/(n-1));

    data.mean_B[j]  = sum_B[j]/n;
    data.sigma_B[j] = sqrt((sum_B2[j] - n*data.mean_B[j]*data.mean_B[j])/(n-1));
  }
}


void play_rtttl(char *p)
{
  // Absolutely no error checking in here

  byte default_dur = 4;
  byte default_oct = 6;
  int bpm = 63;
  int num;
  long wholenote;
  long duration;
  byte note;
  byte scale;

  // format: d=N,o=N,b=NNN:
  // find the start (skip name, etc)

  while(*p != ':') p++;    // ignore name
  p++;                     // skip ':'

  // get default duration
  if(*p == 'd')
  {
    p++; p++;              // skip "d="
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
    if(num > 0) default_dur = num;
    p++;                   // skip comma
  }

  //Serial.print("ddur: "); Serial.println(default_dur, 10);

  // get default octave
  if(*p == 'o')
  {
    p++; p++;              // skip "o="
    num = *p++ - '0';
    if(num >= 3 && num <=7) default_oct = num;
    p++;                   // skip comma
  }

  //Serial.print("doct: "); Serial.println(default_oct, 10);

  // get BPM
  if(*p == 'b')
  {
    p++; p++;              // skip "b="
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
    bpm = num;
    p++;                   // skip colon
  }

  //Serial.print("bpm: "); Serial.println(bpm, 10);

  // BPM usually expresses the number of quarter notes per minute
  wholenote = (60 * 1000L / bpm) * 4;  // this is the time for whole note (in milliseconds)

  //Serial.print("wn: "); Serial.println(wholenote, 10);


  // now begin note loop
  while(*p)
  {
    // first, get note duration, if available
    num = 0;
    while(isdigit(*p))
    {
      num = (num * 10) + (*p++ - '0');
    }
   
    if(num) duration = wholenote / num;
    else duration = wholenote / default_dur;  // we will need to check if we are a dotted note after

    // now get the note
    note = 0;

    switch(*p)
    {
      case 'c':
        note = 1;
        break;
      case 'd':
        note = 3;
        break;
      case 'e':
        note = 5;
        break;
      case 'f':
        note = 6;
        break;
      case 'g':
        note = 8;
        break;
      case 'a':
        note = 10;
        break;
      case 'b':
        note = 12;
        break;
      case 'p':
      default:
        note = 0;
    }
    p++;

    // now, get optional '#' sharp
    if(*p == '#')
    {
      note++;
      p++;
    }

    // now, get optional '.' dotted note
    if(*p == '.')
    {
      duration += duration/2;
      p++;
    }
 
    // now, get scale
    if(isdigit(*p))
    {
      scale = *p - '0';
      p++;
    }
    else
    {
      scale = default_oct;
    }

    scale += OCTAVE_OFFSET;

    if(*p == ',')
      p++;       // skip comma for next note (or we may be at the end)

    // now play the note

    if(note)
    {
      //Serial.print("Playing: ");
      //Serial.print(scale, 10); Serial.print(' ');
      //Serial.print(note, 10); Serial.print(" (");
      //Serial.print(notes[(scale - 4) * 12 + note], 10);
      //Serial.print(") ");
      //Serial.println(duration, 10);
      tone(tonePin, notes[(scale - 4) * 12 + note]);
      delay(duration);
      noTone(tonePin);
    }
    else
    {
      //Serial.print("Pausing: ");
      //Serial.println(duration, 10);
      delay(duration);
    }
  }
}
  
