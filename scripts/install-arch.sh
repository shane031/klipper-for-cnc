#!/bin/bash
# This script runs Klipper on an Arch Linux system

PYTHONDIR="${HOME}/klippy-env"
SRCDIR="${HOME}/klipper"
CONFIG="${HOME}/printer.cfg"

# Step 1: Install system packages
install_packages()
{
    # Packages for python cffi
    PKGLIST="libffi base-devel"
    # kconfig requirements
    PKGLIST="${PKGLIST} ncurses"
    # hub-ctrl
    PKGLIST="${PKGLIST} libusb"
    # AVR chip installation and building
    # PKGLIST="${PKGLIST} avrdude avr-gcc avr-binutils avr-libc"
    # ARM chip installation and building
    # AURLIST="stm32flash"
    # PKGLIST="${PKGLIST} arm-none-eabi-newlib"
    # PKGLIST="${PKGLIST} arm-none-eabi-gcc arm-none-eabi-binutils"

    # Install desired packages
    report_status "Installing packages..."
    sudo pacman -S ${PKGLIST}
}

# Step 2: Create python virtual environment
create_virtualenv()
{
    report_status "Updating python virtual environment..."

    # Create virtualenv if it doesn't already exist
    [ ! -d ${PYTHONDIR} ] && python3 -m venv ${PYTHONDIR}
    
    activate_virtualenv
    
    # Install/update dependencies
    ${PYTHONDIR}/bin/pip install -r ${SRCDIR}/scripts/klippy-requirements.txt
}

activate_virtualenv()
{
    # Activate virtual environment
    report_status "Activating venv at ${PYTHONDIR}/bin/activate."
    source ${PYTHONDIR}/bin/activate
}

# Step 4: Start host software
start_software()
{
    report_status "Launching Klipper host software..."
    deactivate && echo "Deactivated current venv." || echo "No venv to deactivate."
    activate_virtualenv
    python3 ${SRCDIR}/klippy/klippy.py $CONFIG
}

# Helper functions
report_status()
{
    echo -e "\n\n###### $1"
}

verify_ready()
{
    if [ "$EUID" -eq 0 ]; then
        echo "This script must not run as root"
        exit -1
    fi
}

# Force script to exit if an error occurs
set -e

# Check for dirs
ls $PYTHONDIR > /dev/null
ls $SRCDIR    > /dev/null

# Find SRCDIR from the pathname of this script
# SRCDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. && pwd )"

# Run installation steps defined above
verify_ready
install_packages
create_virtualenv
start_software
