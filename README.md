# esp8266-sensor-pv-ctrl-node

ESP8266 sketch which is responsible for sensor reporting and photovoltaic monitoring/control use-cases.

This implementation may be fairly specific to my use-case, but may provide a useful example or inspiration to others.

# Features

* Reads one-wire temperature sensors supported by the DallasTemperature library
* Reads stats from EPever (EPSolar) Tracer AN solar charge controllers
* Reports metrics back to a configurable influxdb url
* Supports turning off the load from the solar charge controller when voltage drops below a configurable threshold - turns power back on for 15 minutes on even hours to allow for periodic metrics recording
* OTA updates via a configurable update url - the **config.json** file contains the sensitive details like wifi credentials, thus the actual OTA update shouldn't need to be kept secret
* Supports multiple "stages" (i.e. deployment sites) with different update/reporting urls. This is useful for testing without polluting one's production DB
* Deep sleeps ~60 seconds between iterations for power saving

# Configuration

**config.json**

```json
{
  "hostname": "node123",
  "ip": "192.168.88.11",
  "gateway": "192.168.88.1",
  "subnet": "255.255.255.0",
  "primaryDNS": "192.168.88.1",
  "secondaryDNS": "8.8.8.8",
  "ntpServer0": "192.168.88.1",
  "tzInfo": "PST8PDT",
  "stationType": "device_status",
  "stages": [
    {
        "name": "test",
        "ssid": "MY_TEST_WIFI",
        "pass": "Secret Password",
        "updateUrl": "https://updates.test/deviceUpdates/node123.ino.bin",
        "influxUrl": "https://influxdb.test/write?db=my_metrics_db&u=influxrw&p=1234567890",
        "influxCaCertFile": "/TestCA.cert"
    },
    {
        "name": "prod",
        "ssid": "MY_PROD_WIFI",
        "pass": "Secret Password",
        "updateUrl": "https://updates.example.com/deviceUpdates/node123.ino.bin",
        "influxUrl": "https://influxdb.example.com/write?db=my_metrics_db&u=influxrw&p=1234567890",
        "influxCaCertFile": "/ProdCA.cert"
    }
  ],
  "sensors": [
    {
        "name": "AIR_TEMP0",
        "id": "0x28, 0xFF, 0xCB, 0x37, 0xB5, 0x16, 0x03, 0xF9"
    },
    {
        "name": "AIR_TEMP1",
        "id": "0x28, 0xFF, 0xF3, 0x7A, 0xB5, 0x16, 0x03, 0x3E"
    }
  ],
  "monitorSolar": true,
  "lowPowerModeThreshold": 12.0
}
```

**ProdCA.cert**

The root certificate relavent to your CA. e.g. for Let's Encrypt see https://letsencrypt.org/certificates/

```
-----BEGIN CERTIFICATE-----
MIIEkjCCA3qgAwIBAgIQCgFBQgAAAVOFc2oLheynCDANBgkqhkiG9w0BAQsFADA/
MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT
DkRTVCBSb290IENBIFgzMB4XDTE2MDMxNzE2NDA0NloXDTIxMDMxNzE2NDA0Nlow
SjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxIzAhBgNVBAMT
GkxldCdzIEVuY3J5cHQgQXV0aG9yaXR5IFgzMIIBIjANBgkqhkiG9w0BAQEFAAOC
AQ8AMIIBCgKCAQEAnNMM8FrlLke3cl03g7NoYzDq1zUmGSXhvb418XCSL7e4S0EF
q6meNQhY7LEqxGiHC6PjdeTm86dicbp5gWAf15Gan/PQeGdxyGkOlZHP/uaZ6WA8
SMx+yk13EiSdRxta67nsHjcAHJyse6cF6s5K671B5TaYucv9bTyWaN8jKkKQDIZ0
Z8h/pZq4UmEUEz9l6YKHy9v6Dlb2honzhT+Xhq+w3Brvaw2VFn3EK6BlspkENnWA
a6xK8xuQSXgvopZPKiAlKQTGdMDQMc2PMTiVFrqoM7hD8bEfwzB/onkxEz0tNvjj
/PIzark5McWvxI0NHWQWM6r6hCm21AvA2H3DkwIDAQABo4IBfTCCAXkwEgYDVR0T
AQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8EBAMCAYYwfwYIKwYBBQUHAQEEczBxMDIG
CCsGAQUFBzABhiZodHRwOi8vaXNyZy50cnVzdGlkLm9jc3AuaWRlbnRydXN0LmNv
bTA7BggrBgEFBQcwAoYvaHR0cDovL2FwcHMuaWRlbnRydXN0LmNvbS9yb290cy9k
c3Ryb290Y2F4My5wN2MwHwYDVR0jBBgwFoAUxKexpHsscfrb4UuQdf/EFWCFiRAw
VAYDVR0gBE0wSzAIBgZngQwBAgEwPwYLKwYBBAGC3xMBAQEwMDAuBggrBgEFBQcC
ARYiaHR0cDovL2Nwcy5yb290LXgxLmxldHNlbmNyeXB0Lm9yZzA8BgNVHR8ENTAz
MDGgL6AthitodHRwOi8vY3JsLmlkZW50cnVzdC5jb20vRFNUUk9PVENBWDNDUkwu
Y3JsMB0GA1UdDgQWBBSoSmpjBH3duubRObemRWXv86jsoTANBgkqhkiG9w0BAQsF
AAOCAQEA3TPXEfNjWDjdGBX7CVW+dla5cEilaUcne8IkCJLxWh9KEik3JHRRHGJo
uM2VcGfl96S8TihRzZvoroed6ti6WqEBmtzw3Wodatg+VyOeph4EYpr/1wXKtx8/
wApIvJSwtmVi4MFU5aMqrSDE6ea73Mj2tcMyo5jMd6jmeWUHK8so/joWUoHOUgwu
X4Po1QYz+3dszkDqMp4fklxBwXRsW10KXzPMTZ+sOPAveyxindmjkW8lGy+QsRlG
PfZ+G6Z6h7mjem0Y+iWlkYcV4PIWL1iwBi8saCbGS5jN2p8M+X+Q7UNKEkROb3N6
KOqkqm57TH2H3eDJAkSnh6/DNFu0Qg==
-----END CERTIFICATE-----

```

# Future Work / TODOs

* Support updating charge controller time when a more accurate time was just fetched from NTP
* Report solar charge parameters - e.g. boost/float voltages, timings, etc
* Support multiple CA certs per stage to allow for replacing expired certificates without downtime
* Support per-network ip/dns configuration
* Provide a secure mechanism for OTA config updates
* Consider making sleep mechanism more accurate and/or making sleep interval configurable
