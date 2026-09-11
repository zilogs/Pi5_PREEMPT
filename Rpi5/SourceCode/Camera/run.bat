@echo off
:: run.bat - สคริปต์สำหรับ Windows
echo Compiling for Linux...
set GOOS=linux
set GOARCH=amd64
go build -o camcap camcap.go

echo Build complete. Please copy 'camcap' file to your Linux machine.
pause