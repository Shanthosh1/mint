# MINT Windows automated build script (PowerShell)
$ErrorActionPreference = "Stop"

# Set working directory to project root
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Definition
Set-Location $ProjectRoot

Write-Host "============================================" -ForegroundColor Cyan
Write-Host "         Building MINT Application          " -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan

# 1. Compile the React Frontend
Write-Host ""
Write-Host "👉 Step 1: Building React Frontend..." -ForegroundColor Yellow
Set-Location frontend
npm install
npm run build
Set-Location ..

# 2. Verify / Setup Virtual Environment
Write-Host ""
Write-Host "👉 Step 2: Preparing Python Virtual Environment..." -ForegroundColor Yellow
if (Test-Path -Path ".venv") {
    Write-Host "✓ Existing virtual environment found (.venv)."
} else {
    Write-Host "⚠ Virtual environment not found. Creating a new one..."
    python -m venv .venv
}

# Ensure latest pip is installed
& .venv\Scripts\python -m pip install --upgrade pip

# Install dependencies
Write-Host "✓ Installing Python dependencies..."
& .venv\Scripts\python -m pip install -r requirements.txt

# Verify / Build Telemetry Router Binary
Write-Host ""
Write-Host "👉 Verify Telemetry Router Binary..." -ForegroundColor Yellow
if (-not (Test-Path -Path "resources/bin\mavp2p.exe")) {
    Write-Host "⚠ Telemetry router binary (resources\bin\mavp2p.exe) not found." -ForegroundColor Yellow
    if (Get-Command go -ErrorAction SilentlyContinue) {
        Write-Host "✓ Go compiler detected. Compiling mavp2p automatically..." -ForegroundColor Green
        New-Item -ItemType Directory -Force -Path resources/bin
        $env:GOBIN = "$(Get-Location)/resources/bin"
        go install github.com/bluenviron/mavp2p@latest
    } else {
        Write-Host "❌ Error: Go compiler not found. Please install Go or manually place the mavp2p.exe binary in resources\bin\." -ForegroundColor Red
        Exit 1
    }
} else {
    Write-Host "✓ Telemetry router binary found in resources/bin\." -ForegroundColor Green
}

# 3. Package with PyInstaller
Write-Host ""
Write-Host "👉 Step 3: Compiling single-folder executable..." -ForegroundColor Yellow
Remove-Item -Recurse -Force dist, build -ErrorAction SilentlyContinue
& .venv\Scripts\python -m PyInstaller mint.spec

Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "🎉 Build Completed Successfully!" -ForegroundColor Green
Write-Host "📍 Output directory: dist\mint\" -ForegroundColor Green
Write-Host "🚀 Run the app via:  .\dist\mint\mint.exe" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
