# ice-flight
TinyFPGA-BX based quadcopter flight controller

# Test
```
cd src
apio sim
```

# Build
```
cd src
apio build
```


# TODO:
  - [x] 32bit memory for storing the fixed point flight data values
    * Split memories, one for each module (imu_mem, attitud_mem, motor_mem)?
    * Virtual mapping to combine all module memories into single memory? (mem map io?)
  - [ ] Define addresses for each flight variable
  - [ ] Move to memory storage for all values to reduce register count
  - [ ] Iterate main flight memory to send as a dump for debugging
