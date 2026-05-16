# STM32 Ultrasonic Radar

Projekt predstavlja preprost ultrazvočni radar, narejen z razvojno ploščo STM32H750B-DK. Sistem uporablja ultrazvočni senzor HC-SR04 za merjenje razdalje. Izmerjeni podatki se sproti prikazujejo na LCD zaslonu, zraven pa je dodan tudi zvočni signal s pasivnim buzzerjem.

## Kaj projekt počne

Radar periodično obrača servo motor in pri različnih kotih meri razdaljo do ovire. Na zaslonu se prikazuje trenutni položaj senzorja in zaznane razdalje, kar ustvari osnovno vizualizacijo okolice pred senzorjem.

Projekt vključuje tudi branje joysticka, uporabo internega temperaturnega senzorja mikrokontrolerja in izpis debug podatkov prek UART povezave.

## Uporabljeno

Pri projektu sem uporabil:

- STM32H750B-DK razvojno ploščo
- HC-SR04 ultrazvočni senzor
- MG90S servo motor
- pasivni buzzer
- joystick
- LCD zaslon na razvojni plošči
- STM32 HAL knjižnice
- CMake za build okolje
- UART za debug izpis

## Struktura projekta

Glavna koda se nahaja v mapi `Core`, gonilniki in podporne knjižnice pa so razdeljeni po posameznih modulih. Projekt vsebuje tudi konfiguracijske datoteke za STM32CubeMX, CMake in nastavitve za razvojno okolje.

## Demo brez servo motorja
https://github.com/user-attachments/assets/1f65830c-f0af-4018-8498-691af0bc8aee
