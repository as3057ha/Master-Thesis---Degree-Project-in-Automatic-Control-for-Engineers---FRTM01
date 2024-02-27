#include <Stepper.h>

// Define the connection pins
const int DIR_PIN = 8; // DIR- connected to pin 8
const int PUL_PIN = 9; // PUL- connected to pin 9

// Define control variables
const int STEPS_PER_REVOLUTION = 3200; // Set this to the number of pulses per revolution as per DIP switch settings
const double SAMPLE_TIME = 0.1 * 1000; // In miliseconds

// Stepper definition
Stepper JMC(STEPS_PER_REVOLUTION,PUL_PIN,DIR_PIN);

// Define physical variables
const double NUMBER_TEETH_A = 32;
const double NUMBER_TEETH_B = 60;
const double FEEDING_RADIUS = 0.05; // In meters 

// Velocity
double vTcp;
double rpm;
int steps;

// Mock data
double vectorData[] = {
    0.0005923316160294426, -0.00010162865124387814, -6.350145888285325e-05, 0.016567409702998318, 0.02031301213100328,
    0.039191360139298095, 0.04485468382104221, 0.061892197622671084, 0.08104223845887772, 0.0960026794041281,
    0.1301056722696201, 0.14603005962256216, 0.1803643151348643, 0.2077796537233585, 0.24086499817887638,
    0.27752744682023295, 0.317042049311117, 0.3655560531800093, 0.40372121756680035, 0.4466164264865411,
    0.49960510503603506, 0.5008395768285745, 0.49868352823118817, 0.4989555376133149, 0.5095487986934275,
    0.48928230356882774, 0.49288713661499356, 0.5126332756436758, 0.49959272859014836, 0.5077766861150973,
    0.5025090994028439, 0.5079995610991677, 0.49921981924918996, 0.5022299838896609, 0.4914433883473273,
    0.49242010595099556, 0.47635897765675794, 0.46374710927750573, 0.46245896754703647, 0.4479422837332736,
    0.42623894200566825, 0.4153795724045813, 0.40894964570689013, 0.37753454843945905, 0.361723385197715,
    0.33443248199062087, 0.30724047922689757, 0.2879479991368391, 0.24875197098947824, 0.22903976606824003,
    0.18856741640727212, 0.1713098900731542, 0.12913118404039092, 0.10551717293616283, 0.06369625631762069,
    0.015065912578921915, 0.005795811647082465, -0.006424028443371094, -0.0006031530911661132, 0.003445980145674999,
    0.0026981447269134673, 0.006623734398884928
};
int currentIndex = 0;

int totalElements = sizeof(vectorData) / sizeof(vectorData[0]);


void setup() {
  // Set DIR_PIN and PUL_PIN as outputs
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PUL_PIN, OUTPUT);
  
  // Set DIR_PIN low
  digitalWrite(DIR_PIN, LOW);
}

void loop() {

  // Get the current time
  unsigned long t0 = millis();
  // Read robot velocity
  vTcp = readVTcp();
  // Calculate corrisponding rpm
  rpm = calculateRPM(vTcp);
  // Set the motor speed 
  JMC.setSpeed(rpm);
  // Calculate the number of steps needed to maintain the sample time
  steps = calculateNbrSteps(rpm);
  // Step the motor
  JMC.step(steps);
  // Get the current time
  unsigned long t1 = millis();
  // Calculate the time passed in this loop iteration
  unsigned long tPassed = t1 - t0;

  // Check if the time passed is less than the desired sample time
  if (tPassed < SAMPLE_TIME){
    // If so, delay for the remaining time to meet the sample time
    delay(SAMPLE_TIME- tPassed);
  }
  
}

double readVTcp() {
    // Check if all elements have been read
    if (currentIndex >= totalElements) {
        // Stop the main loop
        while (true) {
            // Do nothing, just keep the main loop running
        }
    }

    // Return the current value and update the index for the next call
    double currentValue = vectorData[currentIndex];
    currentIndex++;
    return currentValue;
}

double calculateRPM(double vTcp) {return vTcp*60*NUMBER_TEETH_B/(FEEDING_RADIUS*2*PI*NUMBER_TEETH_A);}

int calculateNbrSteps(double rpm){
  int nbrSteps = (int)(SAMPLE_TIME*STEPS_PER_REVOLUTION*rpm/60);
  return nbrSteps;
}


