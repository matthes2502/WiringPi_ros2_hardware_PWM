#include <iostream>
#include <cstdlib>
#include <unistd.h>

int main() {
    std::cout << "VESC/Pumpe wird initialisiert..." << std::endl;
    
    // GPIO 18 als PWM konfigurieren
    system("gpio -g mode 18 pwm");
    system("gpio pwm-ms");
    system("gpio pwmc 19");      // Clock = 19
    system("gpio pwmr 20000");   // Range = 20000
    
    // 0-Stellung senden (1000µs)
    std::cout << "Sende 0-Stellung (1000µs)..." << std::endl;
    system("gpio -g pwm 18 1000");
    
    std::cout << "Warte auf VESC-Initialisierung..." << std::endl;
    sleep(3);
    
    std::cout << "VESC bereit!" << std::endl;
    std::cout << "\nGib Wert ein (1000-2000) oder -1 zum Beenden:" << std::endl;
    
    int wert;
    while(true) {
        std::cout << "\nEingabe: ";
        std::cin >> wert;
        
        if(wert == -1) {
            // Vor dem Beenden auf 0 setzen
            std::cout << "Setze auf 1000 und beende..." << std::endl;
            system("gpio -g pwm 18 1000");
            sleep(1);
            break;
        }
        
        if(wert < 1000 || wert > 2000) {
            std::cout << "Ungültige Eingabe! Bitte 1000-2000 oder -1" << std::endl;
            continue;
        }
        
        std::cout << "Setze PWM auf " << wert << "µs" << std::endl;
        
        char cmd[50];
        sprintf(cmd, "gpio -g pwm 18 %d", wert);
        system(cmd);
    }
    
    std::cout << "Programm beendet." << std::endl;
    return 0;
}