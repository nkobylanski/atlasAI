### 1. Introducción
Este proyecto tiene como objetivo demostrar la viabilidad de un sistema de navegación autónoma basado en visión artificial y sensores de proximidad. El robot Pioneer P3DX será programado para detectar cubos de colores en un orden específico (rojo, verde, azul, amarillo) y desplazarse hacia ellos, evitando un obstáculo fijo colocado en el entorno. La simulación se lleva a cabo en CoppeliaSim, utilizando Python para la lógica de control y OpenCV para el procesamiento de imágenes.

### 2. Objetivos
Implementar la detección de colores (rojo, verde, azul, amarillo) mediante una cámara simulada en CoppeliaSim.
Programar el robot para que se desplace hacia el cubo detectado, ajustando su trayectoria en tiempo real.
Integrar sensores ultrasónicos para la detección y evitación de obstáculos.
Lograr que el robot complete un ciclo de reconocimiento y navegación hacia los cuatro colores en el orden establecido.

Documentar el proceso y resultados en un PoC que valide la viabilidad del sistema.

### 3. Descripción técnica
Componentes utilizados:
Robot Pioneer P3DX con dos ruedas motrices y rueda loca.
16 sensores ultrasónicos para detección de proximidad.
1 cámara RGB montada en el robot.
Cubos de colores (rojo, verde, azul, amarillo) y un obstáculo estático.

Arquitectura del sistema:
Control del robot: Clase PioneerP3DX_Robot que maneja los motores y sensores.
Procesamiento de imagen: Función filter_color() que usa OpenCV para detectar colores en el espacio HSV.

Lógica de navegación:
El robot gira hasta detectar un color.
Si detecta el color, se calcula el error de posición y se ajusta la velocidad de las ruedas (control proporcional).
Si el tamaño del objeto supera un umbral, se considera alcanzado y se pasa al siguiente color.
Los sensores ultrasónicos se leen en cada iteración para evitar colisiones.

### Escenario:

<img width="1759" height="1377" alt="Captura de pantalla (10)" src="https://github.com/user-attachments/assets/d02915c8-a3cf-4ef5-aa3d-2bcc90b6419e" />

