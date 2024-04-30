Empfohlener Luftstrom des Ventilators: 236,6 - 304,2 m3/h
Raumvolumen: 33,8 m3


HTML Uploader ESP8266
#####################
Geht wohl (aktuell) nur mit der alten Arduino IDE

Benötigte Tools für diese Anleitung:
Neben der Arduino IDE und der Installation der ESP8266 Plugins (hier beschrieben) wird das Tool “ESP8266 Sketch Data Upload” benötigt.

Laden Sie das Tool aus folgendem Git-Repository herunter: https://github.com/esp8266/arduino-esp8266fs-plugin/releases/download/0.1.3/ESP8266FS-0.1.3.zip
Erstellen Sie in Ihrem Arduino Installations-Verzeichnis das Verzeichnis “tools”, falls es noch nicht existiert.
Entpacken Sie das .zip Archiv in dieses Verzeichnis (der Pfad sollte dann wie folgt aussehen: <Installations-Verzeichnis>/Arduino/tools/ESP8266FS/)
Starten Sie die Arduino IDE neu
Erstellen Sie einen neuen Sketch und speichern Sie diesen
Navigieren Sie das Abgabeverzeichnis des Sketches (auch über die Arduino IDE möglich durch: Sketch -> Sketch-Ordner Anzeigen)
Erstellen Sie den Unterordner “data” und legen Sie die Bilddateien und HTML-Dateien hier ab.
Wenn nun das richtige Board und COM-Port ausgewählt ist, so wie der Serielle Monitor geschlossen ist, können die Dateien über:
Werkzeuge -> ESP8266 Sketch Data Upload an den ESP übertragen werden.

https://randomnerdtutorials.com/install-esp8266-filesystem-uploader-arduino-ide/