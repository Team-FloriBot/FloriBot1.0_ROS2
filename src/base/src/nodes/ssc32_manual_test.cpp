#include "base/diff_drive_lib.hpp"
#include <iostream>
#include <string>

int main() {
    std::string port = "/dev/ttyS1";
    int baud = 115200;

    try {
        std::cout << "Initialisiere SSC32 an " << port << "..." << std::endl;
        SSC32Driver driver(port, baud);
        
        std::cout << "Verbindung erfolgreich!" << std::endl;
        std::cout << "Befehle: 'w' (Vorwaerts), 's' (Rueckwaerts), 'a' (Links), 'd' (Rechts), ' ' (Stop), 'q' (Exit)" << std::endl;

        int p_left = 1500;
        int p_right = 1500;
        bool running = true;

        while (running) {
            char input;
            std::cin >> input;

            switch (input) {
                case 'w': p_left = 1400; p_right = 1600; break; // Beachte Invertierung Rechts
                case 's': p_left = 1600; p_right = 1400; break;
                case 'a': p_left = 1600; p_right = 1600; break;
                case 'd': p_left = 1400; p_right = 1400; break;
                case ' ': p_left = 1500; p_right = 1500; break;
                case 'q': running = false; break;
            }

            std::cout << "Sende: Links=" << p_left << " Rechts=" << p_right << std::endl;
            driver.send_commands(p_left, p_right);
        }
    } catch (const std::exception& e) {
        std::cerr << "Fehler: " << e.what() << std::endl;
    }

    return 0;
}
