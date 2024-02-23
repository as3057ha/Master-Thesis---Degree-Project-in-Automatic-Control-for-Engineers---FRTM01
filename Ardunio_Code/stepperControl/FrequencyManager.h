#ifndef FrequencyManager_h
#define FrequencyManager_h

#include "Arduino.h"

class FrequencyManager {
  private:
    double _feedingSpeed; // Current feeding speed
    double _feedPerPulse; // Constant representing the feed rate per pulse
    double _frequency;    // Calculated frequency based on feeding speed and feed per pulse
    double _threshold;    // Threshold for updating feeding speed to minimize noise or small variations

  public:
    FrequencyManager(double feedPerPulse); // Constructor
    void update(double newFeedingSpeed);   // Updates the feeding speed
    double getFrequency() const;           // Returns the current frequency
};

#endif
