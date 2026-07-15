```
pixi shell
pixi run build
```

```
cd build
pixi run gui-capabilities          # one-time setcap on the built binary
./gui/actuator-test-gui
```
or with sudo
```
sudo ./gui/actuator-test-gui
```

