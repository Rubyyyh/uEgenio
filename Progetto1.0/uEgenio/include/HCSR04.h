#ifndef HCSR04_H
#define HCSR04_H

#include "mbed.h"

/**
 * @brief Driver per sensore ultrasonico HC-SR04
 *
 * Collegamento STM32 (esempio Nucleo):
 *   VCC  -> 5V
 *   GND  -> GND
 *   TRIG -> D9  (output digitale)
 *   ECHO -> D10 (input digitale - attenzione: HC-SR04 uscita 5V,
 *                usare partitore resistivo se STM32 è 3.3V)
 *
 * Sequenza di misura:
 *   1. Impulso TRIG HIGH per almeno 10 µs
 *   2. Il sensore emette 8 burst ultrasonici a 40 kHz
 *   3. ECHO resta HIGH per un tempo proporzionale alla distanza
 *   4. Distanza (cm) = durata_echo_us / 58.0
 */
class HCSR04 {
public:
    /**
     * @param trig_pin  Pin digitale collegato a TRIG
     * @param echo_pin  Pin digitale collegato a ECHO
     * @param max_cm    Distanza massima attesa (default 400 cm)
     */
    HCSR04(PinName trig_pin, PinName echo_pin, float max_cm = 400.0f);

    /**
     * @brief  Esegue una singola misura di distanza
     * @return Distanza in centimetri, oppure -1.0 in caso di timeout
     */
    float read_cm();

    /**
     * @brief  Esegue una singola misura di distanza
     * @return Distanza in millimetri, oppure -1.0 in caso di timeout
     */
    float read_mm();

    /**
     * @brief  Media di N misure (filtra i valori di errore)
     * @param  samples  Numero di campioni (default 5)
     * @param  delay_ms Pausa tra campioni in ms (default 30)
     * @return Distanza media in cm, oppure -1.0 se tutti i campioni falliscono
     */
    float read_cm_avg(uint8_t samples = 5, uint32_t delay_ms = 30);

private:
    DigitalOut _trig;
    DigitalIn  _echo;
    Timer      _timer;
    uint32_t   _timeout_us;   // timeout calcolato da max_cm

    /**
     * @brief Invia l'impulso di trigger e misura la durata dell'echo
     * @return Durata in microsecondi, 0 in caso di timeout
     */
    uint32_t _pulse_us();
};

#endif // HCSR04_H
