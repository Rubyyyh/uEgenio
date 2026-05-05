@echo off
title Setup progetto Robot AI
echo ===============================
echo Installazione dipendenze Python
echo ===============================

python -m pip install --upgrade pip
pip install -r requirements.txt

echo.
echo ===============================
echo Installazione FFmpeg
echo ===============================

winget install Gyan.FFmpeg

echo.
echo ===============================
echo Download modello Ollama
echo ===============================

ollama pull qwen3:4b

echo.
echo ===============================
echo Setup completato
echo ===============================
pause