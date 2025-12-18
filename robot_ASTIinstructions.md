# Control de Marcha con IK en CoppeliaSim (Asti)

Este script implementa el control de la marcha de un robot bípedo llamado **Asti** en **CoppeliaSim**, utilizando:

- La API remota ZMQ (`coppeliasim_zmqremoteapi_client`)
- Cinemática inversa (IK) con el módulo `simIK`
- Trayectorias predefinidas para ambos pies
- Interpolación temporal y correcciones de marcha

---

## Dependencias

```python
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
