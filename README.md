# guncon3_2_guncon2

Programación para convertir la entrada de una guncon3 a una trama de guncon2 para compatibilidad con la consola ps2.
Consta de una placa olimex stm32e407 rev.b que hace la función de comunicarse con la consola playstation 2 por usb.
Por la entrada de UART3 se comunica con un convertidor ttl-serie-usb con el pc linux. Cabe la posibilidad de compilarlo para raspberry pi.
En el pc/raspberry se conecta la guncon3.
La aplicación contiene un botón de calibrado, que convertirá los valores de la pistola a las dimensiones de la pantalla.

El procedimiento para conectar el sistema es el siguiente:
1.-Conectar la placa a la ps2 y cargar el juego.
2.- Seleccionar el checkbox de guncon2
3.- Seleccionar el terminal serie en el combobox inferior al checkbox anterior.
4.- Apretar serial connect
5.- Apretar connect guncon
6.- Apareceran valores por pantalla, apretar Calibration
7.- Disparar con la guncon3 a las mirillas mostradas a pantalla completa
8.- Apretar run

Esta configuración está probada con un ubuntu 17.04 y el virtua cop elite.
En la calibración dentro de este juego se necesita mantener el botón "C2" de la pistola mientras se dispara a la mirilla para que la pantalla no se quede en negro.

Agradecimientos: 
beardypig: Por la ingeniería inversa llevada acabo a la gcon3
equipo de nuvee plugin: por compartir el software de comunicación con la guncon2 
