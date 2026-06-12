#!/bin/bash
# MINT macOS/Linux automated build script
# Terminates the script immediately if any command returns a non-zero exit code
set -e

# Change directory to the root of the project
cd "$(dirname "$0")"

echo "============================================"
echo "         Building MINT Application          "
echo "============================================"

# 1. Compile the React Frontend
echo ""
echo "👉 Step 1: Building React Frontend..."
cd frontend
npm install
npm run build
cd ..

# 2. Verify / Setup Virtual Environment
echo ""
echo "👉 Step 2: Preparing Python Virtual Environment..."
if [ -d ".venv" ]; then
    echo "✓ Existing virtual environment found (.venv)."
else
    echo "⚠ Virtual environment not found. Creating a new one..."
    python3 -m venv .venv
fi

# Ensure latest pip is installed
.venv/bin/python -m pip install --upgrade pip

# Install dependencies
echo "✓ Installing Python dependencies..."
.venv/bin/python -m pip install -r requirements.txt

# Verify / Build Telemetry Router Binary
echo ""
echo "👉 Verify Telemetry Router Binary..."
if [ ! -f "resources/bin/mavp2p" ]; then
    echo "⚠ Telemetry router binary (resources/bin/mavp2p) not found."
    if command -v go &> /dev/null; then
        echo "✓ Go compiler detected. Compiling mavp2p automatically..."
        mkdir -p resources/bin
        GOBIN="$(pwd)/resources/bin" go install github.com/bluenviron/mavp2p@latest
    else
        echo "❌ Error: Go compiler not found. Please install Go or manually place the mavp2p binary in resources/bin/."
        exit 1
    fi
else
    echo "✓ Telemetry router binary found in resources/bin/."
fi

# 3. Package with PyInstaller
echo ""
echo "👉 Step 3: Compiling single-folder executable..."
rm -rf dist build
.venv/bin/python -m PyInstaller mint.spec

echo ""
echo "============================================"
echo "🎉 Build Completed Successfully!"
echo "📍 Output directory: dist/mint/"
echo "🚀 Run the app via:  ./dist/mint/mint"
echo "============================================"
