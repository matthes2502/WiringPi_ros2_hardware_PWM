#include <iostream>
#include <cstdlib>
#include <unistd.h>

int main() {
    std::cout << "ESC wird initialisiert..." << std::endl;
    
    // GPIO 19 als PWM konfigurieren
    system("gpio -g mode 19 pwm");
    system("gpio pwm-ms");
    system("gpio pwmc 19");      // Clock = 19
    system("gpio pwmr 20000");   // Range = 20000
    
    // 0-Stellung senden (1100µs bei 20000 range = 1100)
    std::cout << "Sende 0-Stellung (1100µs)..." << std::endl;
    system("gpio -g pwm 19 1100");
    
    std::cout << "Warte auf ESC-Initialisierung..." << std::endl;
    sleep(3);
    
    std::cout << "ESC bereit!" << std::endl;
    std::cout << "\nGib Prozentwert ein (0-100%) oder -1 zum Beenden:" << std::endl;
    
    double prozent;
    while(true) {
        std::cout << "\nEingabe (%): ";
        std::cin >> prozent;
        
        if(prozent == -1) {
            // Vor dem Beenden auf 0 setzen
            std::cout << "Setze auf 0% und beende..." << std::endl;
            system("gpio -g pwm 19 1100");
            sleep(1);
            break;
        }
        
        if(prozent < 0 || prozent > 100) {
            std::cout << "Ungültige Eingabe! Bitte 0-100 oder -1" << std::endl;
            continue;
        }
        
        // Mapping: 0% = 1100µs, 100% = 1940µs
        // PWM-Wert = 1100 + (prozent / 100) * (1940 - 1100)
        int pwm_wert = 1100 + (prozent / 100.0) * 840;
        
        std::cout << "Setze " << prozent << "% (PWM: " << pwm_wert << "µs)" << std::endl;
        
        char cmd[50];
        sprintf(cmd, "gpio -g pwm 19 %d", pwm_wert);
        system(cmd);
    }
    
    std::cout << "Programm beendet." << std::endl;
    return 0;
}