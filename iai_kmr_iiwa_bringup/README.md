### ips
marge `192.168.102.78`
nav pc `192.168.103.16`
smart pad `192.168.103.17`
sick laser stuff `192.168.103.18`

### run shit

log in marge and execute root thingy

on marge:
```
roscore
roslaunch iai_kmr_iiwa_bringup base.launch
```

nomachine to nav pc 
```
start start.bat
start bin/fakeodometry
```

on roboter
```
click red button
click green button
```

on smart pad
```
flip key 
click AUT
flip key
start ros smart servo with green play button
```
    
    
### troubleshooting
## how to change sick laser safety fields
- install sick stuff on windoff `https://www.sick.com/de/de/cds/p/p37940`

- connect to sick laser ip

- click on schutzfelder and change them all
