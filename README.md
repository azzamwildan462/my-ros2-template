# This is just a template 

## Compile 
```
mkdir build && cd build 
cmake .. 
make 
```

## How to run? 
Just a simple thing doing binary run in `bin/`

## Run using all launcher 
Just doing `./run.sh`

## Run as a Service 
Create a copy of systemd_service_template.service   
Just change the `Description`, `Environment`, and `ExecStart` 
Optionally change the `RestartSec`  

btw, i'm too lazy to make a script or something to do automation xd
