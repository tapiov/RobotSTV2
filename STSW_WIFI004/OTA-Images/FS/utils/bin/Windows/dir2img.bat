@echo off
cls
echo ****************************************************
echo *            Convert Dir Files 2 Img format        *
echo ****************************************************
echo *
echo *
@set CONVERTER=".\CreateFS.exe"
@set WAIT=pause>nul
%CONVERTER% 46 "..\..\..\APP_Disk"
echo *
echo * 
%WAIT%
