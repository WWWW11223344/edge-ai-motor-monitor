@echo off
cd /d %~dp0

del *.bak /s /q 2>nul
del *.ddk /s /q 2>nul
del *.edk /s /q 2>nul
del *.lst /s /q 2>nul
del *.lnp /s /q 2>nul
del *.mpf /s /q 2>nul
del *.mpj /s /q 2>nul
del *.obj /s /q 2>nul
del *.omf /s /q 2>nul
del *.plg /s /q 2>nul
del *.rpt /s /q 2>nul
del *.tmp /s /q 2>nul
del *.__i /s /q 2>nul
del *.crf /s /q 2>nul
del *.o   /s /q 2>nul
del *.d   /s /q 2>nul
del *.axf /s /q 2>nul
del *.tra /s /q 2>nul
del *.dep /s /q 2>nul
del *.iex /s /q 2>nul
del *.htm /s /q 2>nul
del *.sct /s /q 2>nul
del *.map /s /q 2>nul
del JLinkLog.txt /s /q 2>nul

exit
