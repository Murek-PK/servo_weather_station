# servo_weather_station

ESP8266 weather station with SG90 servos, PCA9685 driver and a web-based calibration panel.

Suggested GitHub repository name:
- `servo_weather_station`

This project is designed for a physical 3D-printed weather display that presents:
- current temperature
- current weather condition
- forecast temperature trend
- forecast weather condition

3D files: https://cults3d.com/en/3d-model/home/modular-mechanical-wifi-weather-station-wemos-d1-mini-sg90

## Features

- ESP8266-based control
- PCA9685 servo driver support
- support for `1-5` forecast modules
- web UI for calibration and diagnostics
- first-run calibration workflow
- current weather and forecast from Open-Meteo
- hourly / daily forecast switching with a physical switch
- buffered forecast switching for fast mode changes
- 3-point calibration for weather servos:
  - `Full sun`
  - `Partly cloudy`
  - `Fog`

## Hardware

Main components:
- ESP8266 board
- PCA9685 16-channel servo driver
- SG90 servos
- external 5V power supply for servos
- 3D-printed weather station body and indicators

## Location Configuration

The firmware supports two ways to define the weather location:

1. `City + country code`
2. `Latitude + longitude`

In the sketch:
- `WEATHER_LOCATION_USE_COORDINATES = false`
  - uses `WEATHER_CITY` and `WEATHER_COUNTRY`
  - location is resolved through Open-Meteo geocoding
- `WEATHER_LOCATION_USE_COORDINATES = true`
  - uses `WEATHER_LATITUDE` and `WEATHER_LONGITUDE`
  - geocoding is skipped

Recommended usage:
- use `city + country` for quick setup
- use `latitude + longitude` for the most stable and precise location mapping

Example country codes:
- `PL`
- `DE`
- `FR`
- `US`

## PCA9685 Channel Map

- `0` -> actual temperature
- `1` -> actual weather
- `2` -> forecast module 1 temperature
- `3` -> forecast module 1 weather
- `4` -> forecast module 2 temperature
- `5` -> forecast module 2 weather
- `6` -> forecast module 3 temperature
- `7` -> forecast module 3 weather
- `8` -> forecast module 4 temperature
- `9` -> forecast module 4 weather
- `10` -> forecast module 5 temperature
- `11` -> forecast module 5 weather

## ESP8266 Pin Usage

- `GPIO13` -> physical forecast mode switch
  - `LOW` to `GND` = daily forecast
  - `HIGH` / open = hourly forecast
- `GPIO4` / `GPIO5` are typically used for I2C with PCA9685 (`SDA` / `SCL`)

## Weather States

The mechanical weather dial uses 7 states:
- `Full sun`
- `Snow`
- `Rain`
- `Partly cloudy`
- `Cloudy`
- `Thunderstorm`
- `Fog`

## Weather Interpretation Logic

The displayed weather state is not based on a simple one-to-one mapping from `weather_code`.

For the live module, the firmware evaluates multiple Open-Meteo parameters and then chooses the most suitable visual state for the mechanical dial. The logic gives priority to stronger weather events first:
- thunderstorm
- snow
- rain
- fog

If none of those conditions are detected, the firmware estimates how the sky should look visually by combining several cloud-related values:
- total cloud cover
- low cloud cover
- mid cloud cover
- high cloud cover
- day / night flag
- direct solar radiation

This produces a visual cloud score, which is then translated into:
- `Full sun`
- `Partly cloudy`
- `Cloudy`

For hourly forecast modules, the code also uses a multi-parameter interpretation and applies extra safeguards so obvious cases such as WMO `3` (`Overcast`) are not shown as `Full sun`.

For daily forecast modules, the firmware uses a lighter interpretation based on the fields available in daily forecast data and maps them to the same 7 physical weather states.

## Forecast Logic

- Current weather is displayed on the base module.
- Forecast modules display:
  - weather condition on weather servos
  - relative temperature trend on temperature servos

Temperature trend rules:
- hourly mode uses current temperature as the reference
- daily mode uses today's maximum temperature as the reference
- the largest forecast temperature difference gets the largest servo deflection
- other forecast temperature servos move proportionally

## Calibration

The web UI supports:
- current temperature calibration with 3 points:
  - `+30 C`
  - `0 C`
  - `-30 C`
- weather servo calibration with 3 points:
  - `Full sun`
  - `Partly cloudy`
  - `Fog`
- direct test buttons for all 7 weather positions

On first boot, the station starts in calibration mode.

After saving settings:
- calibration is marked as completed
- the station starts in weather mode on the next boot

## Web UI

The browser panel provides:
- calibration controls
- live weather diagnostics
- current forecast mode
- current switch state
- hourly interval selection
- last weather data timestamp from Open-Meteo
- last successful station update time

## Arduino Libraries

This sketch uses:
- `ESP8266WiFi`
- `ESP8266WebServer`
- `ESP8266mDNS`
- `EEPROM`
- `ESP8266HTTPClient`
- `WiFiClientSecure`
- `ArduinoJson`
- `FaBoPWM_PCA9685`

External libraries and platforms keep their own licenses.
This repository contains the project sketch and documentation, but does not re-distribute those third-party libraries.

## Project File

- `servo_weather_station.ino` -> main firmware sketch

## Public Repository Notes

The public sketch already uses placeholder Wi-Fi credentials:
- `YOUR_WIFI_SSID`
- `YOUR_WIFI_PASSWORD`

Before uploading firmware to hardware, replace them with your own network settings.

## Suggested GitHub Additions

When publishing the full project, consider adding:
- photos or renders of the build
- STL / 3D model files
- wiring diagram
- assembly guide
- BOM (bill of materials)
- screenshots of the calibration page

## Weather Data Attribution

This project uses weather data from [Open-Meteo](https://open-meteo.com/).

Open-Meteo API data are offered under the [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) license.

Recommended attribution text:

`Weather data by Open-Meteo.com`

Useful links:
- [Open-Meteo Licence](https://open-meteo.com/en/licence)
- [Open-Meteo Terms](https://open-meteo.com/en/terms)

## Usage Responsibility

Users of this project are responsible for:
- configuring their own API usage in compliance with Open-Meteo terms
- respecting request limits of the free API
- checking whether their own usage is non-commercial or requires a paid plan

If this project is used commercially or distributed as part of a commercial product, review Open-Meteo terms before deployment.

## Author

Project published as `Murek`.

## License

This project is released under the MIT License. See `LICENSE`.
