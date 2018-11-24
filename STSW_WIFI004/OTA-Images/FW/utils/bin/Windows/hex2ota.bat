@echo off
cls
echo ****************************************************
echo *               Convert Hex 2 Ota formats          *
echo ****************************************************
echo *
echo *
@set CONVERTER=".\create_ota.exe"
@set WAIT=pause>nul
dir ..\..\..\*.hex /b > temp.txt
@set /p res=<temp.txt
rm temp.txt
@set RELEASE_NAME=%res:~0,-4%
@set IMAGE=..\..\..\%RELEASE_NAME%.hex
%CONVERTER% %IMAGE% %RELEASE_NAME%.fota
%CONVERTER% %IMAGE% %RELEASE_NAME%.sfota key.bin
echo *
echo * 
%WAIT%
