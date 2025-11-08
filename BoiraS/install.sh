#!/bin/bash
# ============================================================
# Instalador librerías para BoiraS
# Autor: Adrián Blanco de Paz
# Sistema: Linux
# ============================================================

echo "==============================================="
echo "🚀 Instalador BoiraS | Adrián Blanco"
echo "==============================================="

echo " Comprobando instalación de Python3..."
if ! command -v python3 &> /dev/null; then
    echo " Python3 no está instalado. Instalando..."
    sudo apt update && sudo apt install -y python3
else
    echo " Python3 detectado: $(python3 --version)"
fi

echo " Comprobando pip3..."
if ! command -v pip3 &> /dev/null; then
    echo " pip3 no está instalado. Instalando..."
    sudo apt install -y python3-pip
else
    echo " pip3 detectado: $(pip3 --version)"
fi

echo " Comprobando tkinter..."
python3 -c "import tkinter" 2>/dev/null
if [ $? -ne 0 ]; then
    echo " tkinter no está instalado. Instalando..."
    sudo apt install -y python3-tk
else
    echo " tkinter ya está instalado."
fi

echo " Instalando dependencias del sistema..."
sudo apt-get update -y
sudo apt-get install -y build-essential git cmake bison flex \
    libxml2-dev libusb-1.0-0-dev libavahi-client-dev \
    libavahi-common-dev libaio-dev libcdk5-dev

echo " Instalando librerías necesarias con pip3..."
packages=(
    numpy
    matplotlib
    pandas
    scipy
    reedsolo
    customtkinter
    scikit-dsp-comm
)

for pkg in "${packages[@]}"; do
    echo " Instalando $pkg ..."
    pip3 install --upgrade "$pkg"
done

echo " Instalando pyadi-iio y dependencias específicas..."
pip3 install --upgrade "pylibiio==0.23.1" || {
    echo " Error instalando pylibiio - revise que libiio esté presente en la distribución Linux que use."
}
pip3 install --upgrade "pyadi-iio==0.0.14" || {
    echo " Error instalando pyadi-iio - revise que pyadi-iio esté presente en la distribución Linux que use."
}

# ============================================================
# Instalación de PlutoSDR (libiio + libad9361-iio + pyadi-iio)
# ============================================================
echo " Instalando librerías de soporte para PlutoSDR..."

WORKDIR="$HOME/plutosdr_setup"
mkdir -p "$WORKDIR"
cd "$WORKDIR"

echo " Instalando libiio..."
if [ ! -d "libiio" ]; then
    git clone --branch v0.23 https://github.com/analogdevicesinc/libiio.git
fi
cd libiio
mkdir -p build && cd build
cmake -DPYTHON_BINDINGS=ON ..
make -j$(nproc)
sudo make install
sudo ldconfig
cd "$WORKDIR"

echo " Instalando libad9361-iio..."
if [ ! -d "libad9361-iio" ]; then
    git clone https://github.com/analogdevicesinc/libad9361-iio.git
fi
cd libad9361-iio
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
cd "$WORKDIR"

echo " Instalando pyadi-iio (desde repositorio oficial)..."
if [ ! -d "pyadi-iio" ]; then
    git clone --branch v0.0.14 https://github.com/analogdevicesinc/pyadi-iio.git
fi
cd pyadi-iio
sudo pip3 install --upgrade pip
sudo pip3 install -r requirements.txt
sudo python3 setup.py install

# Limpieza de temporales
cd ~
rm -rf "$WORKDIR"

echo "-----------------------------------------------"
echo " Todas las dependencias han sido instaladas correctamente."
echo "-----------------------------------------------"
