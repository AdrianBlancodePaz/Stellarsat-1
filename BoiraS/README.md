# BoiraS
Aplicación en python diseñada para comunicarse con el hardware del cubesat St3llarSat-1 Boira desarrollado para como TFG de Adrián Blanco de Paz en la Universidad Carlos III de Madrid. Este programa permite enviar telecomandos así como recibir el hosekeeping y la telemetría del satélite. Además, spectrogram.py permite ver la intensidad de la señal en frecuencia recibida por la SDR en tiempo real.

Atención! Esta aplicación no está terminada, por lo que podría tener problemas y/o comandos que no hagan ningún efecto.

--------------------------------------------------------------------------------------------

## Instalación

Para instalar las librerías necesarias para correr este programa:

chmod +x install.sh

./install.sh

La ip de la PlutoSDR debería ser 192.168.2.1, de no ser así, cambiarlo en el archivo .py

## Uso

Para utilizar el programa BoiraS: python3 BoiraS.py

Para utilizar la herramienta de espectrograma: python3 spectrogram.py


## Comandos

###PAYLOAD

    pay-tc-001: TC to start PAY Experiment(spectrometer (Configuration 1)
    
    pay-tc-002: TC to start PAY Experiment(spectrometer)(Configuration 2)
    
    pay-tc-003: TC to start PAY Experiment(spectrometer)(Configuration 3)
    
    pay-tc-004: TC to downlink the complete PAY Experiment(spectrometer)(Configuration 1)
    
    pay-tc-005: TC to downlink the complete PAY Experiment(spectrometer)(Configuration 2)
    
    pay-tc-006: TC to downlink the complete PAY Experiment(spectrometer)(Configuration 3)
    
    pay-tc-007: TC to download PAY Experiment(spectrometer)(Configuration 1) from package N to M (will be asked after to get N and M)
    
    pay-tc-008: TC to download PAY Experiment(spectrometer)(Configuration 2) from package N to M (will be asked after to get N and M)
    
    pay-tc-009: TC to download PAY Experiment(spectrometer)(Configuration 3) from package N to M (will be asked after to get N and M)

###TTC

    ttc-tc-001: TC for RF-Cessation
    
    ttc-tc-002: TC for RF-Activation
    
    ttc-tc-003: Handshake TC for first contact for a SC pass over the GS
    
    ttc-tc-004: Goodbye TC for last contact for a SC pass over the GS
    
    
###OBC

    obc-tc-001: TC to change from Operation to LEOP Mode
    
    obc-tc-002: TC to change from LEOP to Operation Mode
    
    obc-tc-003: TC to change from Operation to Safe Mode
    
    obc-tc-004: TC to change from Safe to Operation Mode
    
    obc-tc-005: TC to get the SC Events
    
    obc-tc-006: TC to perform the EPS Reboot
    
    obc-tc-007: TC to perform the ADCS Reboot
    
    obc-tc-008: TC to perform the OBC-TTC Reboot
    
    obc-tc-009: TC to perform the PAY Reboot
    
    obc-tc-010: TC to return the OBSW Baseline
    
    obc-tc-011: TC to activate the EOL-Skyfall Procedure
    
    obc-tc-012: TC to abort the EOL-Skyfall Procedure
    
    obc-tc-013: TC to abort Solution/Experiment(ADCS or PAY)
    
    obc-tc-014: TC to abort Solution/Experiment queue(ADCS or PAY)
    
###ADCS

    adcs-tc-001: TC to upload the TLE from GS to the SC
    
    adcs-tc-002: TC to return to ADCS Baseline
    
    adcs-tc-003: TC to start the ADCS Solution 1
    
    adcs-tc-004: TC to start the ADCS Solution 2
    
    adcs-tc-005: TC to start the ADCS Solution 3
    
    adcs-tc-006: TC to download TBD hours of ADCS Solution 1
    
    adcs-tc-007: TC to download TBD hours of ADCS Solution 2
    
    adcs-tc-008: TC to download TBD hours of ADCS Solution 3
    
    adcs-tc-009: TC to download ADCS Solution 1 from package N to M (will be asked after to get N and M)
    
    adcs-tc-010: TC to download ADCS Solution 2 from package N to M (will be asked after to get N and M)
    
    adcs-tc-011: TC to download ADCS Solution 3 from package N to M (will be asked after to get N and M)
    
    
###TTC-CONFIG

    set code X: Change SC codification scheme
    
              X=0: No codification
              
              X=1: Convolutional 1/2
              
              X=2: Reed Solomon
              
              X=3: Convolutional + Reed Solomon
              
    set modulation X
    set interleaver X: Change SC interleaver settings
              X=0: No interleaver
              X=1: Interleaver
    
###BS-CONFIG

    bs set code X: Change BS codification scheme
    
              X=0: No codification
              
              X=1: Convolutional 1/2
              
              X=2: Reed Solomon
              
              X=3: Convolutional + Reed Solomon
              
    bs set modulation X
    
    bs set interleaver X: Change BS interleaver settings
    
              X=0: No interleaver
              
              X=1: Interleaver
    
###GENERAL

    exit/quit: Close program
    
    help: Display the help menu
