/**
 * @brief Bluethooth HC-06 Manager Device Library
 */
 
// Author: Gustavo Diaz

// Requiered Libraries
#include <Arduino.h>

class LedClass
{
    // Members
    uint8_t state_;
    uint8_t led_pin_;

public:
    // Public Members
    // ...

    // constructor de base (null)
    LedClass() {}

    // constructror parametrizado
    LedClass(uint8_t led_pin):
    led_pin_(led_pin)
    {}

    // methods
    uint8_t getState(void);
    void blink(void);
    void on(void);
    void off(void);

/*private:
    // methods
    
    void read(void);
    void write(int data);*/

};