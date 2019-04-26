# PM_proyecto_final
Archivos iniciales del proyecto final Principios Mecatrónica 2019

## Directorios
### src
En este directorio se encuentran los paquetes a utilizar. 

### rosbags 
Contiene las bolsas de datos que se pueden utilizar para probar los algoritmos implementados

### build y devel
Estos directorios se crean DESPUES de la compilación. Contienen los compilados y archivos auxiliares. NO se deben modificar.

## Paquetes
### graphical_client
Cliente para visualizar los obstáculos, punto objetivo y trajectoria propuesta

### pub_example
Ejemplo de un publicador en python al tópico "/trajectory". 

### ssl_shared_library
Libraria de robocup ssl.

## Ejecución
### Clonar
En la terminal:
```
cd PATH/TO/DESIRE/DIR
git clone https://github.com/garygra/PM_proyecto_final.git
cd PM_proyecto_final
```

### Compilar
Desde PM_proyecto_final
```
catkin_make
```

### Source
En CADA terminal nueva, es necesario que ROS encuentre los archivos del proyecto (compilados, launch, etc). Para esto utilizar el comando:
```
cd PATH/TO/REPO/PM_proyecto_final
source devel/setup.bash
```


### Ejecución
Como ejemplo, se va a ejecutar el cliente gráfica, puntos de una trajectoria arbitraria y una rosbag. Para esto se necesitan tres terminales.

Terminal 1:
```
roslaunch graphical_client graphical_client.launch
```

Terminal 2:
```
rosrun pub_example publisher.py
```

Terminal 3:
```
rosbag play rosbags/obstacles_5.bag
```

Cada comando se debe ejecutar en orden (primero el 1, al último el 3). El orden importa porque el primero "roslaunch" se encarga de ejecutar "roscore". Si se quisiera ejecutar sin importar el orden sería necesario tener otra terminal donde se ejecute "roscore".

