#include "FrequencyManager.h"

// Constructor implementation
FrequencyManager::FrequencyManager(double feedPerPulse)
  : _feedingSpeed(0), _feedPerPulse(feedPerPulse), _frequency(0), _threshold(0.001) {
}

// Updates the feeding speed if the change is significant
void FrequencyManager::update(double newFeedingSpeed) {
    if (abs(_feedingSpeed - newFeedingSpeed) > _threshold) {
        _feedingSpeed = newFeedingSpeed;
        _frequency = _feedingSpeed / _feedPerPulse;
    }
}

// Returns the current frequency
double FrequencyManager::getFrequency() const {
    return _frequency;
}
