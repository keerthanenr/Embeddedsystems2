#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//Mapping from sequential drive states to motor phase outputs
/*
 State   L1  L2  L3
 0       H   -   L
 1       -   H   L
 2       L   H   -
 3       L   -   H
 4       -   L   H
 5       H   L   -
 6       -   -   -
 7       -   -   -
 */
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L2L(L2Lpin);
PwmOut L3L(L3Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3H(L3Hpin);

RawSerial pc(SERIAL_TX, SERIAL_RX);

SHA256 coin;

Thread out;
Thread decode;
//Thread duty_cycle;
Thread bitcoin_thread;
Thread motorCtrlT(osPriorityNormal, 1024);
// thread to run a task every 1000ms,
//1024 bytes stack size, error printed out if an issue occurs

int8_t orState = 0; //Rotot offset at motor state 0
//
PwmOut motor_pin(PWMpin);

Queue<void, 16> inCharQ;
char char_array[18] = {0};
volatile int64_t velocity= 0;

int curr_pos = 0;

volatile uint64_t newKey = 0;
volatile uint64_t receivedKey;
float n_velocity;                   // for case 'V'
volatile int64_t no_velocity=0;
volatile int64_t no_rotation=0;
Mutex newKey_mutex;
Mutex newNumberOfRevolutionsPerSecond_mutex;
Mutex newMaximumSpeed_mutex;
Mutex hashRate_mutex;


int64_t Ts=0; //torque speed
int64_t Tr = 0; //rotatioal
int64_t torque_position = 0; 
int64_t torque_pos_output = 0;
int64_t Torque_output=0;
enum messageCode {
    nonceFound,
    velocity_found,
    position_found,
    hash_rate 
};


typedef struct{
    uint8_t code;
    uint64_t data;
} message_t ;

Mail<message_t,16> outgoingmsg;

void putMessage(uint8_t code, uint64_t data){
    message_t *pMessage = outgoingmsg.alloc();
    pMessage->code = code;
    pMessage->data = data;
    outgoingmsg.put(pMessage);
}
int8_t intState = 0;


void outcomm(){
    
    while(1) {
        osEvent newEvent = outgoingmsg.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
        switch (pMessage->code) {
            case nonceFound:
                pc.printf("Nonce with data 0x%016x\n",pMessage->data);
                outgoingmsg.free(pMessage);
                break;
            case velocity_found:
                pc.printf("Current Velocity %f\n\r",velocity);
                outgoingmsg.free(pMessage);
                break;
            case position_found:
                pc.printf("Current Position %d\n\r",pMessage->data);
                outgoingmsg.free(pMessage);
                break;
            case hash_rate:
                pc.printf("Computation Rate %d\n\r", pMessage->data);
                outgoingmsg.free(pMessage);
        }
        
    }
    
}

void serialISR(){
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}




void decode_instruction(char* input){
    if(input[0] == 'K'){
        newKey_mutex.lock();
        sscanf(char_array, "K%llx", &newKey);
        pc.printf("%llx\n\r",newKey);
        newKey_mutex.unlock();
    }
    // else if(input[0] == 'M'){
    //     sscanf(char_array, "M%f", &n_velocity);
    //     pc.printf("%f",n_velocity);
    //     motor_pin.write(n_velocity);
    // }
    else if(input[0] == 'V'){
        newMaximumSpeed_mutex.lock();
        sscanf(char_array, "V%d", &no_velocity);
        newMaximumSpeed_mutex.unlock();
        pc.printf("%d",no_velocity);
    }
    else if(input[0] == 'R'){
        newNumberOfRevolutionsPerSecond_mutex.lock();
        sscanf(char_array, "R%d", &no_rotation);
        newNumberOfRevolutionsPerSecond_mutex.unlock();
        pc.printf("%d",no_rotation);
    }
}

void decodeFn(void){
    pc.printf("Enter the command:\n\r");
    pc.attach(&serialISR);
    uint8_t ptr = 0;
    while(1){
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        if(newChar != '\r' && newChar != '\n'){
            char_array[ptr] = newChar;
            ptr++;
        }
        else{
            char_array[ptr] = '\0';
            ptr = 0;
            decode_instruction(char_array);
        }
    }
}




//Set a given drive state
void motorOut(int8_t driveState, uint64_t torque){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
    
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(torque);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(torque);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(torque);
    if (driveOut & 0x20) L3H = 0;
}


//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0,2000);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

void photointerrupt_ISR(){
    
    int8_t intState = 0;
    int8_t intStateOld = 0;
    int8_t diff_state=0;
    intState = readRotorState();
    diff_state= intState-intStateOld;
    //curr_position measures the number of revolutions
    //every 6 (0-->5) it adds one revolution to curr_pos
    motorOut((intState-orState+lead+6)%6, 2000);
    if (intState != intStateOld) {
        if (diff_state == 5)                  // from position 5 to position 0 (only want a step of 1)
        {
            curr_pos++;
        }
        else if (diff_state == -5)              // from position 0 to position 5 (only want a step of -1)
        {
            curr_pos--;
        }
        curr_pos= diff_state+curr_pos;
    }
    intStateOld = intState;
}
void motorCtrlTick(){
    
    motorCtrlT.signal_set(0x1); // signal is sent back to the motor control thread
    //avoids wasting computational power
}
void motorCtrlFn(){
    
    int   first_reading=0;
    int   second_reading=0;
    int     iteration_counter=0;
    int tick_interval = 100000; //tick interval 100ms
    int64_t es=0;
    int64_t kps=25;
    velocity= 0;
    int64_t abs_velocity=0;
    Ticker motorCtrlTicker;
    int64_t er =0; 
    float diff_er=0;
    int64_t kpr= 75; 
    int64_t kdr=20; 
    float previous_er=0;
    motorCtrlTicker.attach_us(&motorCtrlTick,tick_interval); // the ISR motorCtrlTick is triggered by Ticket
    
    while(1)
    {
        core_util_critical_section_enter();
        first_reading = curr_pos;         // to avoid ISR updating currentPosition
        core_util_critical_section_exit();
        
        first_reading=first_reading/6;
        motorCtrlT.signal_wait(0x1); // waiting for the tick to happen
        core_util_critical_section_enter();
        second_reading = curr_pos; // avoid subtraction here
        core_util_critical_section_exit();
        second_reading=second_reading/6;

        //velocity = (second_reading - first_reading) * 10;
        
        iteration_counter++;
        

//NOTE: Uncomment the speed function and comment the Tr function 

//----------------Ts function starts here-----------------------//
        
        
        //if(velocity==0){
        //velocity=200;
        //}
        
        
        
        // if(velocity<0){
        //     abs_velocity=velocity*-1; 
        // }else{
        //     abs_velocity=velocity;
        // }
        

        // newMaximumSpeed_mutex.lock();
        // //proportional speed controller
        // es= no_velocity- abs_velocity;
        // newMaximumSpeed_mutex.unlock();

        // Ts=kps *es;
        
        // if(Ts<0){
        //     //pc.printf("negative- %d\n", Ts);
        //     Torque_output= -Ts; 
        //     lead=-2;
        // }else{
        //     //pc.printf("positive- %d\n", Ts);
        //     Torque_output=Ts;
        //     lead=2;
        // }
        
        // if(Torque_output> 2000){
        //     //pc.printf("Ts>2000\n");
        //     //pc.printf("%d\n", Ts);
        //     Torque_output=2000;
        // }
        // if(iteration_counter>9){
        //     //attempted messages but overflow
        //    // putMessage(velocity_found, velocity);
        //    //  putMessage(position_found, er);
        //       //  pc.printf("%f/n", es);
        //       //  pc.printf("v: %d\n\r", velocity);
        // //     pc.printf("T: %d\n\r", Torque_output);
        // //     pc.printf("TARGET v: %d\n\r", no_velocity);
        //         iteration_counter=0;
        // }

//------------------Tr function starts here------------------------//
        if(no_rotation == 0){
            er = 1000;
        }

        newNumberOfRevolutionsPerSecond_mutex.lock();        
        er = no_rotation- second_reading;
        newNumberOfRevolutionsPerSecond_mutex.unlock();

        diff_er= (er- previous_er)*10;
        previous_er=er; 

        Tr= (kpr*er) + (kdr*diff_er);

        if(Tr<0){
            torque_pos_output= -Tr; 
            lead=-2;
        }else{
            torque_pos_output=Tr; 
            lead=2; 

        }

        if(iteration_counter>9){
            //attempted messages but overflow
           // putMessage(velocity_found, velocity);
           //  putMessage(position_found, er);
            // pc.printf("p: %d\n\r", second_reading);
            //pc.printf("TARGET p: %d\n\r", no_rotation);
            //pc.printf("Remaining: %d\n\r", er);
            // pc.printf("Torque_output: %d\n\r", torque_pos_output);
            iteration_counter=0;


        }

//-----------------attempted to do switching function statements ------------

        // if(diff_er>=0){
        //     //take the minimum 
        //     if(Ts<Tr){
        //         torque_pos_output=Ts;
        //     }else{
        //         torque_pos_output=Tr; 
        //     }
        // }else{
        //     //take the maximum
        //     if(Ts<Tr){
        //         torque_pos_output=Tr;

        //     }else{
        //         torque_pos_output=Ts; 
        //     }
            
        // }
        // if(torque_pos_output>2000){
        //     //limits the torque to max pwm
        //     torque_pos_output=2000;
        // }

        //this function will control the torque for the speed function
        motorOut((intState-orState+lead+6)%6, Torque_output);

        // this function will control the torque for rotation function
        motorOut((intState-orState+lead+6)%6, torque_pos_output);

        photointerrupt_ISR();
        
    }
}
void bitcoin(){
    
    
    int hashCount=0;
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
        0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
        0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
        0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
        0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
        0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
    
    
    Timer timer;
    timer.start();
    while (1) {
        
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();
        
        coin.computeHash(hash, sequence, 64);
        hashCount++;
        if ((hash[0] || hash[1]) == 0) {
            putMessage(nonceFound,*nonce);
        }
        (*nonce)++;
        if(timer.read() >= 1){
            int computationalRate= hashCount/timer.read();
            // pc.printf("\n\r");
            // pc.printf("Computational Rate:");
            // pc.printf("%d", computationalRate);


            putMessage(hash_rate, computationalRate);
            timer.reset();
            hashCount=0;
        }
        
    }
    
    
    
}


//Main
int main() {
    
    motor_pin.period(0.002f); //period of 2ms
    motor_pin.write(1.0f); // 100% duty cycle
    L1L.period_us(2000);
    L2L.period_us(2000);
    L3L.period_us(2000);
    L1L.write(100.0);
    L2L.write(100.0);
    L3L.write(100.0);
    //Initialise the serial port
    
    pc.printf("Hello\n\r");
    
    motorCtrlT.start(motorCtrlFn);
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    
    out.start(outcomm);
    
    decode.start(callback(decodeFn));
    bitcoin_thread.start(callback(bitcoin));
    //    duty_cycle.start(callback(motor_torque));
    
    
    I1.rise(&photointerrupt_ISR);
    I1.fall(&photointerrupt_ISR);
    I2.rise(&photointerrupt_ISR);
    I2.fall(&photointerrupt_ISR);
    I3.rise(&photointerrupt_ISR);
    I3.fall(&photointerrupt_ISR);
    
    
    //    motor_torque();
}


