#!/bin/bash

################################################################################
# Coral Server - Install Dependencies and Run Server
################################################################################
#
# This script:
# 1. Checks and installs required Python packages (if not already installed)
# 2. Sets up the environment for Coral Server
# 3. Runs the Coral Server on port 9900
#
# Usage: ./install_and_run.sh [--server-only] [--use-coral] [--help]
#
# Options:
#   --server-only    Skip dependency installation and run server directly
#   --use-coral      Use coral_server.py instead of coral_server_tflite.py
#   --help           Display this help message
#
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_CMD=${PYTHON_CMD:-python3}
SKIP_INSTALL=false
USE_CORAL_SERVER=false

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1" >&2
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Function to display help
show_help() {
    cat << EOF
Coral Server - Install and Run

Usage: ./install_and_run.sh [OPTIONS]

OPTIONS:
    --server-only    Skip dependency installation and run server directly
    --use-coral      Use coral_server.py instead of coral_server_tflite.py
    --help           Display this help message

EXAMPLES:
    ./install_and_run.sh                    # Normal: install and run
    ./install_and_run.sh --server-only      # Just run the server
    ./install_and_run.sh --use-coral        # Run with full pycoral library

EOF
    exit 0
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --server-only)
            SKIP_INSTALL=true
            shift
            ;;
        --use-coral)
            USE_CORAL_SERVER=true
            shift
            ;;
        --help)
            show_help
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            ;;
    esac
done

# Check if Python is available
if ! command -v "$PYTHON_CMD" &> /dev/null; then
    print_error "Python3 not found. Please install Python 3.7 or later."
    exit 1
fi

print_info "Using Python: $PYTHON_CMD"
print_info "Python version: $("$PYTHON_CMD" --version)"

# Function to check if a Python package is installed
check_package() {
    "$PYTHON_CMD" -c "import $1" 2>/dev/null
    return $?
}

# Function to install a package
install_package() {
    local package=$1
    local pip_name=${2:-$1}
    
    if check_package "$package"; then
        print_success "Package '$package' already installed"
        return 0
    fi
    
    print_info "Installing package: $pip_name"
    if "$PYTHON_CMD" -m pip install --upgrade "$pip_name"; then
        print_success "Package '$pip_name' installed successfully"
        return 0
    else
        print_error "Failed to install '$pip_name'"
        return 1
    fi
}

# Installation phase
if [ "$SKIP_INSTALL" = false ]; then
    print_info "=========================================="
    print_info "Installing Dependencies"
    print_info "=========================================="
    
    # Ensure pip is up to date
    print_info "Upgrading pip..."
    "$PYTHON_CMD" -m pip install --upgrade pip > /dev/null 2>&1 || true
    
    # Required packages
    print_info "Checking required packages..."
    install_package "numpy" "numpy>=1.23.2" || exit 1
    install_package "cv2" "opencv-python>=4.5.0" || exit 1
    install_package "msgpack" "msgpack>=1.0.0" || exit 1
    
    # TFLite runtime - try multiple methods
    if ! check_package "tflite_runtime"; then
        print_info "Attempting to install tflite-runtime..."
        if ! "$PYTHON_CMD" -m pip install --upgrade tflite-runtime 2>/dev/null; then
            print_warning "tflite-runtime not available for this Python version"
            print_info "Installing TensorFlow as alternative (heavier but compatible)..."
            install_package "tensorflow" "tensorflow" || exit 1
        fi
    fi
    
    # Optional pycoral package
    if [ "$USE_CORAL_SERVER" = true ]; then
        print_info "Attempting to install pycoral for TPU support..."
        if "$PYTHON_CMD" -m pip install git+https://github.com/google-coral/pycoral.git 2>/dev/null; then
            print_success "pycoral installed successfully"
        else
            print_warning "pycoral installation failed. Continuing with CPU-only mode."
            print_warning "For TPU support, install pycoral manually: pip install git+https://github.com/google-coral/pycoral.git"
        fi
    fi
    
    print_success "All dependencies installed successfully"
fi

# Server selection
if [ "$USE_CORAL_SERVER" = true ]; then
    SERVER_FILE="$SCRIPT_DIR/coral_server.py"
else
    SERVER_FILE="$SCRIPT_DIR/coral_server_tflite.py"
fi

# Verify server file exists
if [ ! -f "$SERVER_FILE" ]; then
    print_error "Server file not found: $SERVER_FILE"
    exit 1
fi

print_info "=========================================="
print_info "Starting Coral Server"
print_info "=========================================="
print_info "Using server: $(basename "$SERVER_FILE")"
print_info "Host: 0.0.0.0"
print_info "Port: 9900"
print_info "Press Ctrl+C to stop"
print_info "=========================================="

# Run the server
"$PYTHON_CMD" "$SERVER_FILE"
