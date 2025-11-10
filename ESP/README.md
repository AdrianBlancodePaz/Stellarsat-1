| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# StellarSat-1 Boira

Código para ESP32 para el sistema de telemetría y telecomandos para el satélite StellarSat-1 boira desarrollado para Trabajo de Fin de Grado de Adrián Blanco de Paz. Atención: Esta es una primera revisión incompleta. Su funcionamiento consta de un mensaje de housekeeping, enviado por defecto cada 7 segundos. También está implementada la gestión de telecomandos así como de telecomandos prioritarios.

## Hardware

Este proyecto está desarrollado para un ESP32-S3 así como dos transceptores sx127X.

# Firmware - ESP32 StellarSat-1 Boira (VSCode + ESP-IDF Extension) 

Este proyecto está diseñado para el **ESP32** y se puede compilar y subir fácilmente usando **Visual Studio Code** junto con la **extensión ESP-IDF**. No es necesario instalar ESP-IDF manualmente; la extensión se encarga de instalar todas las herramientas necesarias.

---

## Requisitos

1. **Visual Studio Code**  
2. **Extensión ESP-IDF para VSCode**  
   - [Descargar extensión](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)  
   - Esta extensión instalará automáticamente ESP-IDF, Python y otras herramientas requeridas.
   - Preconfigurar extensión en VSCode.
3. Un **ESP32** conectado por USB.

---

## Clonar el proyecto

```bash
git clone https://github.com/TU_USUARIO/TU_REPO.git
cd TU_REPO/Proyecto2
