#pragma once

// Production flashing loop (factory.cpp).

// What the status LED should show. The patterns all animate (even "solid" OK
// has a blip), so a latched-up or wedged probe -- frozen LED -- still stands
// out at a glance.
enum FactoryLed {
    FACTORY_LED_OFF = 0,   // loop not running
    FACTORY_LED_WAITING,   // short blip: alive, waiting for a target
    FACTORY_LED_BUSY,      // fast blink: flashing a target
    FACTORY_LED_BOOTING,   // flashed and verified; booting it / confirming / self-test
    FACTORY_LED_OK,        // on with a brief blip: last board verified good
    FACTORY_LED_FAIL,      // double blip: last board failed, set it aside
    FACTORY_LED_NO_IMAGE,  // no factory image stored: upload one over USB
};

// Start the loop in its own task; false if already running.
bool factory_start(void);

FactoryLed factory_led_state(void);
