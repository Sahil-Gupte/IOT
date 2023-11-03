import Adafruit_DHT

sensor = Adafruit_DHT.DHT11
pin = 4 
humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

if humidity is not None and temperature is not None:
    print(f'Temperature: {temperature:.1f}°C')
    print(f'Humidity: {humidity:.1f}%')
else:
    print('Failed to retrieve data from DHT11 sensor')
