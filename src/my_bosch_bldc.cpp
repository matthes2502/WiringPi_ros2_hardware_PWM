#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main(void) {
    int pwmPin = 1; // GPIO18 = WPI1, GPIO19 = WPI24
    int range = 500; // PWM Range, 500
    int clock = 25;  // ergibt ca. 1500 Hz, --> 25
    int pwmMin = 210;  // 5 % --> 180
    int pwmMax = 350;  // 10 % --> 350

    // Initialisierung mit PHYS-Nummerierung
    wiringPiSetupPinType(WPI_PIN_WPI);

    // // Pin freigeben und initial auf 0 setzen
    // pinMode(pwmPin, PM_OFF);
    pwmWrite(pwmPin, 0);

    // PWM konfigurieren
    pinMode(pwmPin, PWM_MS_OUTPUT);
    pwmSetMode(PWM_MODE_MS);       // Mark/Space-Modus
    pwmSetRange(range);
    pwmSetClock(clock);

    // Motor specs
    // motor starts running at 0.73 V which is 22 % of 3.3 V
    // motor increases speed until 1.25 V which is 38 % of 3.3 V
    // the voltage level must be mapped to the range
    // pwm val = 

    // ein Mal auf 1100 µs setzen
    int pwmValue = pwmMin;
    // pwmValue = 200;
    pwmWrite(pwmPin, pwmValue+5);
    delay(250);
    pwmWrite(pwmPin, pwmValue-5);

    printf("BLDC-Steuerung über PWM gestartet.\n");
    printf("Gib Prozentwert (0–100) ein, oder -1 zum Beenden:\n");

    int input_per;
    while (1) {
        printf("Leistung [%%]: ");
        fflush(stdout);

        if (scanf("%d", &input_per) != 1) {
            fprintf(stderr, "Ungültige Eingabe. Abbruch.\n");
            break;
        }

        if (input_per < 0 || input_per > 100) {
            if (input_per == -1) break;
            printf("Bitte 0–100 eingeben (oder -1 zum Beenden).\n");
            continue;
        }

        // ESC erwartet: 5–10 % Duty Cycle → umrechnen:
        // int pwmMin = 226;  // 5 %
        // int pwmMax = 390;  // 10 %

        if (input_per == 0) {
        // Sicherer Wert unterhalb pwmMin
        pwmValue = 0; // z. B. deutlich kleiner als 225
        printf("Motor soll stehen bleiben bei %d.\n", pwmValue);
        } else {
            pwmValue = pwmMin + input_per * (pwmMax - pwmMin) / 100;
        }

        pwmWrite(pwmPin, pwmValue);
        printf(" Eingabe: %d\n PWM gesetzt auf %d (%.2f ms)\n", input_per, pwmValue, pwmValue * 20.0 / range);
    }

    // Stop
    printf("PWM Stop.\n");
    pwmWrite(pwmPin, 0);
    pinMode(pwmPin, PM_OFF);

    return 0;
}
