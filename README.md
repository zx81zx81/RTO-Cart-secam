# RTO-Cart-secam
Multicart for Mattel Intellivision based on Teensy 4.1

Fork repository of https://github.com/aotta/RTO-Cart
The software is only compatible with the PCB v1.0

Warning: you need to manually ground the pin 8 in the cart's slot (video-ext input) on that version of the pcb.
See here https://github.com/aotta/RTO-Cart/tree/main/old%20ver for more details.

Few changes compared to original software v1.0 :
- Fix timing to work on my French Intellivision version (Secam)
- Remove all the sub-directories browsing (support only the set of .bin and .cfg at the root directory)
- Navigation menu with Up and Down buttons
- Selection of the game while pressing Up and Down at the same time
- Speed up the browsing when a button is pressed during 10 seconds
- Reset the multicart while pressing Up button during cartridge emulation
- Bug fixing

Many thanks to Aotta for his incredible work.
