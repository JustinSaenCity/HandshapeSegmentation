@echo off
git clone --depth=1 https://github.com/lvdmaaten/bhtsne.git
rmdir /s /q bhtsne\.git
del bhtsne\.gitignore