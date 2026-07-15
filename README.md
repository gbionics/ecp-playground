```
pixi shell
pixi run build
```

```
cd build
pixi run gui-capabilities          # one-time setcap on the built binary
./build/gui/actuator-test-gui
```
or with sudo
```
sudo ./build/gui/actuator-test-gui
```

