1. Om de code te gebruiken moet Docker Desktop geinstalleerd worden of Docker via de terminal worden aangeroepen
2. Vervolgens kan de map Website+MQTTBroker/mosquitto worden geopend en in de terminal `docker compose up` ingetyped worden
4. Python moet geinstalleerd zijn in de omgeving voor de volgende stap
3. Daarnaast wordt paho gebruikt en dat kan geinstalleerd worden door `pip3 install paho-mqtt` in de terminal te typen

De commando's die gebruikt kunnen worden zijn:

x = x waarde op het grid (Vanaf 0 tot en met 9)
y = y waarde op het grid (Vanaf 0 tot en met 9)
d = d id van de target   (Vanaf 0 tot en met 1000, hangt er van af wat in de queue staat)
n = nummer van de robot  (Vanaf 0 tot en met 3)

web_robot_n:loca -> Om de locatie van een robot op te vragen. 
web_robot_n:targ:(x,y) ->Om een robot naar een locatie te sturen
web_robot_n:dire -> Om de directie van de robot op te vragen
web_robot_n:sens -> Om de waardes van de sensororen op te vragen

remov_queue:targ:(x,y):id:d

emgc_status
