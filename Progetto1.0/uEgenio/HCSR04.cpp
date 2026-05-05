#include "HCSR04.h"

// Velocità del suono a ~20°C: 343 m/s = 0.0343 cm/µs
// Andata + ritorno -> divisore 58.0 µs/cm
#define SOUND_DIV_CM   58.0f
#define SOUND_DIV_MM    5.8f

// Impulso di trigger minimo: 10 µs (usiamo 15 per sicurezza)
#define TRIG_PULSE_US  15

HCSR04::HCSR04(PinName trig_pin, PinName echo_pin, float max_cm)
    : _trig(trig_pin), _echo(echo_pin)
{
    _trig = 0;
    // Timeout: tempo massimo di echo per max_cm (andata+ritorno) + 20% margine
    _timeout_us = (uint32_t)((max_cm * SOUND_DIV_CM) * 1.2f);
}

uint32_t HCSR04::_pulse_us()
{
    // 1. Assicura TRIG basso
    _trig = 0;
    wait_us(2);

    // 2. Impulso TRIG alto
    _trig = 1;
    wait_us(TRIG_PULSE_US);
    _trig = 0;

    // 3. Attende fronte di salita di ECHO (con timeout)
    _timer.reset();
    _timer.start();
    while (_echo == 0) {
        if (_timer.elapsed_time().count() > _timeout_us) {
            _timer.stop();
            return 0; // timeout fronte salita
        }
    }

    // 4. Misura durata ECHO
    _timer.reset();
    _timer.start();
    while (_echo == 1) {
        if (_timer.elapsed_time().count() > _timeout_us) {
            _timer.stop();
            return 0; // timeout durata echo (oggetto troppo lontano)
        }
    }
    _timer.stop();

    return (uint32_t)_timer.elapsed_time().count(); // µs
}

float HCSR04::read_cm()
{
    uint32_t us = _pulse_us();
    if (us == 0) return -1.0f;
    return (float)us / SOUND_DIV_CM;
}

float HCSR04::read_mm()
{
    uint32_t us = _pulse_us();
    if (us == 0) return -1.0f;
    return (float)us / SOUND_DIV_MM;
}

float HCSR04::read_cm_avg(uint8_t samples, uint32_t delay_ms)
{
    float sum   = 0.0f;
    uint8_t ok  = 0;

    for (uint8_t i = 0; i < samples; i++) {
        float d = read_cm();
        if (d > 0.0f) {
            sum += d;
            ok++;
        }
        if (i < samples - 1) {
            thread_sleep_for(delay_ms);
        }
    }

    return (ok > 0) ? (sum / ok) : -1.0f;
}
