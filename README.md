# Viskositas

Firmware for a falling-ball viscometer. A steel ball falls through a fluid past six sensors. An Arduino times the ball between sensors, gets its average velocity and works out the viscosity with Stokes' law:

```
viscosity = (2/9) * g * r^2 * (rho_ball - rho_fluid) / v
```

The result and the last velocity show on a 16x2 I2C LCD. The timings and velocities also go out over serial.

## Use

Press the button to start a run, then drop the ball. The board is ready for the next run after the ball passes the last sensor.

The sensor positions, ball radius and densities are constants at the top of `src/main.cpp`. They are set for a ball of radius 7.7 mm and density 7850 kg/m³ in a fluid of density 1260 kg/m³. No wall correction is applied.

## Build

This is a PlatformIO project for the Arduino Uno. It uses the `LiquidCrystal_I2C` library.

```sh
pio run --target upload
```
