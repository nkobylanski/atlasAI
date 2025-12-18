from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

def main():
    client = RemoteAPIClient()
    sim = client.require("sim")

    sim.setStepping(True)
    sim.startSimulation()

    # 1) Handle del modelo ASTI (ajusta el path si en tu escena se llama distinto)
    asti = sim.getObject("/Asti")

    # 2) Obtener TODOS los joints que cuelgan de ASTI (en su árbol)
    joint_handles = sim.getObjectsInTree(asti, sim.object_joint_type, 0)

    print(f"ASTI handle: {asti}")
    print(f"Número de joints encontrados: {len(joint_handles)}")
    print(f"Listado de joints (alias):")

    for j in joint_handles:
        alias = sim.getObjectAlias(j, 1)  # 1 = alias completo con ruta
        print(f"  - {alias} (handle={j})")
    
    # 2) Diccionario con los JOINTS que usaremos (6 principales)
    leg_joints = {
        # Pierna derecha
        "hipR": sim.getObject("/Asti/rightUpperLeg/rightLegJoint"),
        "kneeR": sim.getObject("/Asti/rightLowerLeg/rightLegJoint"),
        "ankleR": sim.getObject(
            "/Asti/rightLowerLeg/rightLegJoint/rightLegJointSphere/rightLegJoint"
        ),

        # Pierna izquierda
        "hipL": sim.getObject("/Asti/leftUpperLeg/leftLegJoint"),
        "kneeL": sim.getObject("/Asti/leftLowerLeg/leftLegJoint"),
        "ankleL": sim.getObject(
            "/Asti/leftLowerLeg/leftLegJoint/leftLegJointSphere/leftLegJoint"
        ),
    }

    print("\nJoints seleccionados para caminar:")
    for name, h in leg_joints.items():
        alias = sim.getObjectAlias(h, 1)
        print(f"  {name}: {alias} (handle={h})")
    
        print("\nTipos de joints seleccionados:")
    for name, h in leg_joints.items():
        jt = sim.getJointType(h)
        # 10=revolute, 11=prismatic, 12=spherical (lo importante: distinguir spherical)
        print(f"  {name}: jointType={jt} | alias={sim.getObjectAlias(h, 1)}")

    
    # === POSICIÓN INICIAL NEUTRA ===
    neutral_pose = {
        "hipR": 0.0,
        "kneeR": 0.0,
        "ankleR": 0.0,
        "hipL": 0.0,
        "kneeL": 0.0,
        "ankleL": 0.0,
    }

    print("\nAsignando postura inicial neutra:")

    for joint_name, angle in neutral_pose.items():
        joint_handle = leg_joints[joint_name]
        sim.setJointTargetPosition(joint_handle, angle)
        print(f"  {joint_name} -> {angle} rad")
    
        # === TEST: MOVIMIENTO PERIÓDICO DE UN SOLO JOINT ===
    '''test_joint_name = "ankleR"          # vamos a probar con la cadera derecha
    test_joint_handle = leg_joints[test_joint_name]

    A = 0.05   # amplitud en radianes (empieza pequeño)
    f = 0.2    # frecuencia en Hz (0.5 = una oscilación cada 2 segundos)

    print(f"\nTest periódico en {test_joint_name}: A={A} rad, f={f} Hz")

    t0 = sim.getSimulationTime()

    while True:
        t = sim.getSimulationTime()

        if t - t0 > 5.0:
            break

        angle = A * math.sin(2.0 * math.pi * f * t)

        sim.setJointTargetPosition(test_joint_handle, angle)

        print(f"t={t:.2f} | {test_joint_name} -> {angle:.3f} rad")

        sim.step()'''
    
    # === TEST: TOBILLOS EN OPOSICIÓN DE FASE ===
    '''jointR_name = "ankleR"
    jointL_name = "ankleL"

    jointR_handle = leg_joints[jointR_name]
    jointL_handle = leg_joints[jointL_name]

    A = 0.02
    f = 0.15

    t0 = sim.getSimulationTime()

    print(f"\nTest: {jointR_name} y {jointL_name} en oposición | A={A} rad, f={f} Hz (5 s)")

    while True:
        t = sim.getSimulationTime()

        if t - t0 > 5.0:
            break

        angle = A * math.sin(2.0 * math.pi * f * t)

        sim.setJointTargetPosition(jointR_handle, angle)
        sim.setJointTargetPosition(jointL_handle, -angle)

        sim.step()'''
    
        # === TEST: TOBILLOS + RODILLAS EN OPOSICIÓN DE FASE ===
    '''jointR_name = "ankleR"
    jointL_name = "ankleL"

    jointR_handle = leg_joints[jointR_name]
    jointL_handle = leg_joints[jointL_name]

    kneeR_handle = leg_joints["kneeR"]
    kneeL_handle = leg_joints["kneeL"]

    A_ankle = 0.02   # amplitud tobillos (rad)
    A_knee = 0.04    # amplitud rodillas (rad)
    f = 0.15         # frecuencia (Hz)

    t0 = sim.getSimulationTime()

    print(f"\nTest: tobillos y rodillas en oposición | A_ankle={A_ankle} rad, A_knee={A_knee} rad, f={f} Hz (5 s)")

    while True:
        t = sim.getSimulationTime()

        if t - t0 > 5.0:
            break

        ankle_angle = A_ankle * math.sin(2.0 * math.pi * f * t)
        knee_angle = A_knee * math.sin(2.0 * math.pi * f * t)

        sim.setJointTargetPosition(jointR_handle, ankle_angle)
        sim.setJointTargetPosition(jointL_handle, -ankle_angle)

        sim.setJointTargetPosition(kneeR_handle, knee_angle)
        sim.setJointTargetPosition(kneeL_handle, -knee_angle)

        sim.step()'''
    
        # === TEST: TOBILLOS + RODILLAS EN OPOSICIÓN, CON DESFASE EN RODILLAS ===
    '''jointR_name = "ankleR"
    jointL_name = "ankleL"

    jointR_handle = leg_joints[jointR_name]
    jointL_handle = leg_joints[jointL_name]

    kneeR_handle = leg_joints["kneeR"]
    kneeL_handle = leg_joints["kneeL"]

    A_ankle = 0.02   # amplitud tobillos (rad)
    A_knee = 0.04    # amplitud rodillas (rad)
    f = 0.15         # frecuencia (Hz)

    phase_knee = math.pi / 2.0  # desfase de rodillas (90 grados)

    t0 = sim.getSimulationTime()

    print(
        f"\nTest: tobillos y rodillas en oposición con desfase | "
        f"A_ankle={A_ankle} rad, A_knee={A_knee} rad, f={f} Hz, phase_knee=pi/2 (5 s)"
    )

    while True:
        t = sim.getSimulationTime()

        if t - t0 > 5.0:
            break

        ankle_angle = A_ankle * math.sin(2.0 * math.pi * f * t)
        knee_angle = A_knee * math.sin(2.0 * math.pi * f * t + phase_knee)

        sim.setJointTargetPosition(jointR_handle, ankle_angle)
        sim.setJointTargetPosition(jointL_handle, -ankle_angle)

        sim.setJointTargetPosition(kneeR_handle, knee_angle)
        sim.setJointTargetPosition(kneeL_handle, -knee_angle)

        sim.step()'''
    
            # === TEST: CADERAS + RODILLAS + TOBILLOS EN OPOSICIÓN ===
    ankleR_handle = leg_joints["ankleR"]
    ankleL_handle = leg_joints["ankleL"]

    kneeR_handle = leg_joints["kneeR"]
    kneeL_handle = leg_joints["kneeL"]

    hipR_handle = leg_joints["hipR"]
    hipL_handle = leg_joints["hipL"]

    A_ankle = 0.015   # rad
    A_knee  = 0.03    # rad
    A_hip   = 0.02    # rad
    f = 0.12          # Hz

    phase_knee  = math.pi / 2.0   # 90°
    phase_ankle = math.pi / 6.0   # 30°
    phase_hip   = 0.0

    ankle_offset = 0.0
    knee_offset  = 0.0
    hip_offset   = 0.0

    print(
        f"\nTest final: coordinación de piernas | "
        f"A_hip={A_hip}, A_knee={A_knee}, A_ankle={A_ankle}, f={f} Hz"
    )

    # Anclar TODO el robot (modelo raíz)
    asti_pos0 = sim.getObjectPosition(asti, -1)
    asti_ori0 = sim.getObjectOrientation(asti, -1)

    dt = sim.getSimulationTimeStep()
    t = 0.0

    while True:
        t = t + dt

        if t > 5.0:
            break

        ankle_angle = A_ankle * math.sin(2.0 * math.pi * f * t + phase_ankle)
        knee_angle  = A_knee  * math.sin(2.0 * math.pi * f * t + phase_knee)
        hip_angle   = A_hip   * math.sin(2.0 * math.pi * f * t + phase_hip)

        sim.setJointTargetPosition(ankleR_handle, ankle_offset + ankle_angle)
        sim.setJointTargetPosition(ankleL_handle, ankle_offset - ankle_angle)

        sim.setJointTargetPosition(kneeR_handle, knee_offset + knee_angle)
        sim.setJointTargetPosition(kneeL_handle, knee_offset - knee_angle)

        sim.setJointTargetPosition(hipR_handle, hip_offset + hip_angle)
        sim.setJointTargetPosition(hipL_handle, hip_offset - hip_angle)

        # Mantener el robot fijo en el sitio
        sim.setObjectPosition(asti, -1, asti_pos0)
        sim.setObjectOrientation(asti, -1, asti_ori0)

        sim.step()
    
        # === MARCHA SIMPLE (ESTILO COMPAÑERA): CADERAS + RODILLAS ===
    '''lHip = leg_joints["hipL"]
    lKnee = leg_joints["kneeL"]
    rHip = leg_joints["hipR"]
    rKnee = leg_joints["kneeR"]

    hip_amp = 0.15
    knee_amp = 0.30

    dt = sim.getSimulationTimeStep()
    phase = 0.0
    w = 2.0 * math.pi * 0.6  # velocidad angular (0.6 Hz aprox.)

    print("\nMarcha simple: caderas y rodillas (setJointPosition)")

    while True:
        phase = phase + w * dt

        lHipPos =  hip_amp * math.sin(phase)
        rHipPos = -hip_amp * math.sin(phase)

        lKneePos = knee_amp * max(0.0, math.sin(phase))
        rKneePos = knee_amp * max(0.0, -math.sin(phase))

        sim.setJointPosition(lHip, lHipPos)
        sim.setJointPosition(rHip, rHipPos)
        sim.setJointPosition(lKnee, lKneePos)
        sim.setJointPosition(rKnee, rKneePos)

        sim.step()'''




    sim.step()
    sim.stopSimulation()


if __name__ == "__main__":
    main()