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

#include "rtttl_songs.h"
#include "farduino_constants.h"
#include "farduino_types.h"
#include "print_utilities.h"

#define isdigit(n) (n >= '0' && n <= '9')

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low  = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69





char clock_string_buffer[12];
char my_name[16];
char my_line[64];

unsigned int sample_count = 0;
unsigned int file_count = 0;



unsigned long launch_timestamp;
unsigned long peak_timestamp;
unsigned long last_timestamp;


unsigned long test_timestamp0;
unsigned long test_timestamp1;
unsigned long test_timestamp2;
unsigned long test_timestamp3;
unsigned long test_timestamp4;
unsigned long test_timestamp5;
unsigned long test_timestamp6;
unsigned long test_timestamp7;


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


//SD constants and variables
const uint8_t chipSelect = PB12;
SPIClass spi2(2);
SdFat sd(&spi2);
bool SD_present = false;
SdFile dataFile;
bool file_exists;



rocket_state_t current_state;

// SETUP routine

void setup() {

  double altitude;
  unsigned long imu_timestamp;
  unsigned long met_timestamp;
  double sigma_w;
  
  unsigned long mean_count;

  inertial_measurement_t imu_data;

  //set all pyro pins as outputs and inactive
  pinMode(PYRO0, OUTPUT);
  digitalWrite(PYRO0, LOW);
  pinMode(PYRO1, OUTPUT);
  digitalWrite(PYRO1, LOW);
  pinMode(PYRO2, OUTPUT);
  digitalWrite(PYRO2, LOW);  
  pinMode(PYRO3, OUTPUT);
  digitalWrite(PYRO3, LOW);

  //join I2C bus
  Wire.begin();
  Wire.setClock(400000);

  //open USB com port
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
  
  if (!met.begin(0x76)) {
    Serial.println(F("<!> BMP280 not detected"));
    exit(-1);
  }
  else{
    Serial.println(F("BMP280 initialized"));  
  }
  // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  SD_present = sd.begin(chipSelect, SPI_FULL_SPEED);
  if (!SD_present) {
    Serial.println("<!> no SD card found");
  }
  else {
    Serial.println("SD card initialized");
    //search for next free file dataXXXX.txt
    do {
      sprintf(my_name, "data%04d.txt", file_count++);
      Serial.print("checking ");
      Serial.println(my_name);

      file_exists = !dataFile.open(my_name, O_CREAT | O_WRITE | O_EXCL);
      if (!file_exists) {
        Serial.print("file ");
        Serial.print(my_name);
        Serial.println(" created");
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
  
  last_timestamp = get_timestamp();
}



//central LOOP

void loop() {

  char sentence[100];

  unsigned long imu_timestamp;
  unsigned long met_timestamp;
  unsigned long flight_time;
  unsigned long mean_count;
  
  unsigned long delta_time;

  double delta_pressure; 


  test_timestamp0 = get_timestamp();
  
  if (mpu9250_present){
    get_mpu9250_data(imu_timestamp, raw_acc[0], raw_acc[1], raw_acc[2], raw_omega[0], raw_omega[1], raw_omega[2], raw_B[0], raw_B[1], raw_B[2]);
    construct_IMU_sentence(imu_timestamp, raw_acc, raw_omega, raw_B, &sentence[0]);
    Serial1.print(&sentence[0]);
    if (SD_present){
      dataFile.print(&sentence[0]);
    }
  }
  
  test_timestamp1 = get_timestamp();
  
  if (bno055_present){
	  get_bno055_data(imu_timestamp, raw_acc[0], raw_acc[1], raw_acc[2], raw_omega[0], raw_omega[1], raw_omega[2], raw_B[0], raw_B[1], raw_B[2]);
    construct_IMU_sentence(imu_timestamp, raw_acc, raw_omega, raw_B, &sentence[0]);
    //Serial.print(&sentence[0]);
    if (SD_present){
      dataFile.print(&sentence[0]);
    }
  }	  
  
  test_timestamp2 = get_timestamp();
  
  if ((loop_count % 1)==0){
    get_MET_data(met_timestamp, temperature, pressure, altitude);
	delta_time = met_timestamp - last_timestamp;
	last_timestamp = met_timestamp;
	
    construct_MET_sentence(met_timestamp, pressure, temperature, altitude, &sentence[0]);
    Serial.print(&sentence[0]);
    if (SD_present){
      dataFile.print(&sentence[0]); 
    }
  }
  
  test_timestamp3 = get_timestamp();
  
  bool gps_available = get_GPS_data();
  if (gps_available){
	  //Serial.print(&sentence[0]);  
    if (SD_present){
	    dataFile.print(&GPS_sentence[0]);
	  }  
  }

  sample_count++;

  test_timestamp4 = get_timestamp();

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

  test_timestamp5 = get_timestamp();

  acc = sqrt(raw_acc[0] * raw_acc[0] + raw_acc[1] * raw_acc[1] + raw_acc[2] * raw_acc[2]);
  omega = sqrt(raw_omega[0]*raw_omega[0] + raw_omega[1]*raw_omega[1] + raw_omega[2]*raw_omega[2]);
  bool state_changed = false;

  if (current_state != state_IDLE) {
    flight_time = calc_flight_time(launch_timestamp, met_timestamp);
  }
  
  delta_pressure = pressure - pressure_0;

  test_timestamp6 = get_timestamp();

  unsigned long state_timestamp;
  if ((loop_count%1)==0){
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

            //dtostrf(new_sigma_pressure, 4, 3, my_line);
            //serial_line = my_line;
            //Serial.print("delta p : ");
            //Serial.println(serial_line);
            
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
	
      //check if maximum time to peak has elapsed
      if (flight_time > MAX_TIME_TO_PEAK){
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
          
	  //wait for a minimum flight time to filter out pressure fluctuations after launch  
      if ((flight_time > MIN_FLIGHT_TIME) && (pressure > peak_pressure)){
        
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
      
        state_timestamp = imu_timestamp;
        delay(3000);
        play_rtttl(indiana_song);
        break;
      }
      
  }

  if (state_changed) {
    construct_state_sentence(state_timestamp, pressure_0, pressure, current_state, &sentence[0]);
    //Serial1.print(&sentence[0]);
    if (SD_present) {
      dataFile.print(&sentence[0]);
    }
  }
  }
  
  test_timestamp7 = get_timestamp();
  
  
  delta_time = test_timestamp7 - test_timestamp0;
  if (delta_time>50000){
    delta_time = test_timestamp1 - test_timestamp0;
    time_of_day(delta_time, &my_line[0]);
	Serial.print(F("delta t_0 = "));
	Serial.println(&my_line[0]);
	
	delta_time = test_timestamp2 - test_timestamp1;
    time_of_day(delta_time, &my_line[0]);
	Serial.print(F("delta t_1 = "));
	Serial.println(&my_line[0]);
	
	delta_time = test_timestamp3 - test_timestamp2;
    time_of_day(delta_time, &my_line[0]);
	Serial.print(F("delta t_2 = "));
	Serial.println(&my_line[0]);
	
	delta_time = test_timestamp4 - test_timestamp3;
    time_of_day(delta_time, &my_line[0]);
	Serial.print(F("delta t_3 = "));
	Serial.println(&my_line[0]);
	
	delta_time = test_timestamp5 - test_timestamp4;
    time_of_day(delta_time, &my_line[0]);
	Serial.print(F("delta t_SD = "));
	Serial.println(&my_line[0]);
	
	delta_time = test_timestamp6 - test_timestamp5;
    time_of_day(delta_time, &my_line[0]);
	Serial.print(F("delta t_5 = "));
	Serial.println(&my_line[0]);
	
	delta_time = test_timestamp7 - test_timestamp6;
    time_of_day(delta_time, &my_line[0]);
	Serial.print(F("delta t_6 = "));
	Serial.println(&my_line[0]);

    Serial.println(F("\n\n"));
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
