cd \"%~dp0\"
PowerShell -NoProfile -ExecutionPolicy Bypass -Command "& {Start-Process PowerShell -ArgumentList '-NoProfile -ExecutionPolicy Bypass -File \"%~dp0getDependencies.ps1\"' -Verb RunAs}"