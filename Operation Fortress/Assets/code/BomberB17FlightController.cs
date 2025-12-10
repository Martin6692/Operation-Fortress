using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// BomberB17FlightController
/// Realistische Propeller-Bomber-Physik für eine B-17:
/// - Propeller-Schub abhängig von RPM und Luftgeschwindigkeit
/// - AoA-basierter Lift mit AnimationCurve
/// - Induzierter und parasitärer Drag
/// - Stall (Strömungsabriss) mit stark reduziertem Lift und chaotischem Wobble
/// - Steuerflächen: Aileron (roll), Elevator (pitch), Rudder (yaw)
/// - Flaps (erhöhen Lift, reduzieren stallAngle)
/// - Mehrere Triebwerke (engineCount) berücksichtigen Asymmetrien möglich
/// - Kompatibel mit neuem Input System (PlaneControls)
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class BomberB17FlightController : MonoBehaviour
{
    [Header("References")]
    public Rigidbody rb;
    public Collider planeCollider; // set in inspector (Box/MeshCollider convex)
    // OPTIONAL: list of engine transforms for visual RPM/prop animation
    public Transform[] engineTransforms;

    [Header("Mass & Rigidbody Tuning")]
    public float mass = 12000f;          // kg (B-17 Ballpark)
    public float angularDrag = 1.2f;
    public float linearDrag = 0.02f;
    public Vector3 centerOfMassOffset = new Vector3(0f, -0.5f, 0f);

    [Header("Wing / Aerodynamics")]
    public float wingArea = 150f;        // m^2 (approximated)
    public float airDensity = 1.225f;    // sea level
    public AnimationCurve liftCurve = // Cl vs AoA (deg)
        new AnimationCurve(
            new Keyframe(-15f, 0.05f),
            new Keyframe(0f, 0.4f),
            new Keyframe(8f, 1.0f),
            new Keyframe(12f, 1.2f),
            new Keyframe(18f, 0.3f) // post-stall drop
        );
    public float baseDragCoefficient = 0.025f; // parasite
    public float inducedDragFactor = 0.045f;   // induced drag scaling

    [Header("Stall / Stability")]
    public float stallAngle = 16f;       // deg, AoA über dem Stall
    public float stallLiftMultiplier = 0.15f;
    public float stallYawChaos = 3000f;  // torque applied during stall
    public float stallRecoveryDamping = 0.8f; // angular damping applied while stalled

    [Header("Control Surface Effectiveness")]
    public float elevatorPower = 45000f; // pitch torque scale
    public float aileronPower  = 30000f; // roll torque scale
    public float rudderPower   = 20000f; // yaw torque scale
    public float controlResponseSpeed = 3f; // smoothing for inputs

    [Header("Propulsion (4 engines typical for B-17)")]
    public int engineCount = 4;
    public float maxEngineRPM = 2500f;
    public float idleRPM = 600f;
    public float propellerThrustAtZeroAirspeed = 6000f; // N per engine at full throttle & V=0
    public AnimationCurve thrustVsAirspeed = // normalized 0..1 by speed/mach rough curve
        new AnimationCurve(
            new Keyframe(0f, 1f),
            new Keyframe(60f, 0.75f),
            new Keyframe(160f, 0.4f),
            new Keyframe(300f, 0.1f)
        );
    public float throttleResponse = 2.5f; // smoothing of throttle

    [Header("Flaps / Brakes")]
    [Range(0f, 1f)] public float flaps = 0f; // 0..1
    public float maxFlapEffect = 0.45f; // increases liftCoefficient
    public float flapStallReduction = 2.5f; // reduces stallAngle in degrees
    public float brakeDragFactor = 0.5f; // additional drag when braking

    [Header("Turbulence / Small Wobble")]
    public float turbulenceAmplitude = 0.6f; // small angle degrees effect for torque
    public float turbulenceFreq = 0.6f;

    [Header("Ground & Safety")]
    public LayerMask groundLayer;
    public float minAltitudeForGroundCollisionChecks = 0.5f; // start checking near ground
    public bool ensureColliderIsConvex = true; // check inspector

    // --- Internal state ---
    private PlaneControls controls;
    private float inputPitch = 0f;
    private float inputRoll = 0f;
    private float inputYaw = 0f;
    private float inputThrottle = 0f;
    private bool inputBrakes = false;
    private float smoothedThrottle = 0f;

    // cached
    private float currentAoA = 0f;
    private bool isStalled = false;

    void Awake()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        rb.mass = mass;
        rb.angularDamping = angularDrag;
        rb.linearDamping = linearDrag;
        rb.centerOfMass = centerOfMassOffset;
        if (planeCollider == null) planeCollider = GetComponent<Collider>();

        // Input System
        controls = new PlaneControls();

        // We'll read raw values in FixedUpdate via ReadValue to keep code compact and robust
        // but also set temporary on performed to mitigate jitter (optionally)
        controls.Flight.Fire.performed += ctx => { /* placeholder if you want weapons */ };

        smoothedThrottle = idleRPM / maxEngineRPM; // start at idle-like throttle

        // sanity checks
        if (engineCount <= 0) engineCount = 4;
    }

    void OnEnable()
    {
        controls.Enable();
    }

    void OnDisable()
    {
        controls.Disable();
    }

    void FixedUpdate()
    {
        ReadInputs();
        ApplyPropulsionAndForces();
        ApplyControlTorques();
        ApplyTurbulenceAndStallBehavior();
        ApplyBrakesIfNeeded();
        AnimateProps();
    }

    void ReadInputs()
    {
        // Read from InputSystem action map Flight. Values expected -1..1 for axes (Throttle typically 0..1)
        // We smooth controls a bit for realism
        float rawPitch = controls.Flight.Pitch.ReadValue<float>();   // -1..1
        float rawRoll  = controls.Flight.Roll.ReadValue<float>();    // -1..1
        float rawYaw   = controls.Flight.Yaw.ReadValue<float>();     // -1..1
        float rawThrottle = controls.Flight.Throttle.ReadValue<float>(); // expected 0..1 or -1..1 depending binding
        float rawFlaps = controls.Flight.Flaps != null ? controls.Flight.Flaps.ReadValue<float>() : 0f;
        bool rawBrakes = controls.Flight.Brakes.ReadValue<float>() > 0.5f;

        // normalize throttle to 0..1 if it might be -1..1
        if (rawThrottle < 0f) rawThrottle = (rawThrottle + 1f) * 0.5f;

        // smoothing
        inputPitch = Mathf.Lerp(inputPitch, rawPitch, Time.fixedDeltaTime * controlResponseSpeed);
        inputRoll  = Mathf.Lerp(inputRoll, rawRoll, Time.fixedDeltaTime * controlResponseSpeed);
        inputYaw   = Mathf.Lerp(inputYaw, rawYaw, Time.fixedDeltaTime * controlResponseSpeed);
        inputThrottle = Mathf.Clamp01(rawThrottle);
        smoothedThrottle = Mathf.Lerp(smoothedThrottle, inputThrottle, Time.fixedDeltaTime * throttleResponse);

        flaps = Mathf.Clamp01(rawFlaps);
        inputBrakes = rawBrakes;
    }

    void ApplyPropulsionAndForces()
    {
        // --- Kinematics / velocity & AoA ---
        Vector3 vel = rb.linearVelocity;
        float speed = vel.magnitude;
        // Angle of Attack: angle between forward and velocity in vertical plane
        Vector3 velLocal = transform.InverseTransformDirection(vel);
        currentAoA = Mathf.Atan2(-velLocal.y, Mathf.Max(velLocal.z, 0.0001f)) * Mathf.Rad2Deg; // pos = nose up
        // Clamp AoA sensibly for curve lookup
        float aoaForCurve = Mathf.Clamp(currentAoA, -20f, 30f);

        // --- Lift ---
        float Cl = liftCurve.Evaluate(aoaForCurve);
        // Flaps increase lift coefficient
        Cl += maxFlapEffect * flaps;
        // Basic lift formula
        float lift = 0.5f * airDensity * speed * speed * wingArea * Cl;
        Vector3 liftDir = transform.up; // approximate: lift vector aligned with plane 'up'
        rb.AddForce(liftDir * lift * Time.fixedDeltaTime, ForceMode.Force);

        // --- Stall detection & effect on lift ---
        float effectiveStallAngle = stallAngle - (flapStallReduction * flaps);
        isStalled = Mathf.Abs(currentAoA) > effectiveStallAngle;
        if (isStalled)
        {
            // reduce lift drastically
            float reducedLift = lift * stallLiftMultiplier;
            // remove previous lift and reapply reduced? We'll apply additional negative force to simulate drop
            Vector3 stallForce = (reducedLift - lift) * liftDir;
            rb.AddForce(stallForce * Time.fixedDeltaTime, ForceMode.Force);
        }

        // --- Parasitic drag ---
        float parasiteDrag = 0.5f * airDensity * speed * speed * wingArea * baseDragCoefficient;
        // induced drag ~ Cl^2 factor
        float inducedDrag = inducedDragFactor * Cl * Cl * 0.5f * airDensity * speed * speed * wingArea;
        Vector3 totalDrag = -rb.linearVelocity.normalized * (parasiteDrag + inducedDrag);
        rb.AddForce(totalDrag * Time.fixedDeltaTime, ForceMode.Force);

        // --- Propeller thrust per engine scaling with airspeed ---
        float thrustCurveValue = thrustVsAirspeed.Evaluate(speed);
        // per-engine thrust at full throttle
        float perEngineMaxThrust = propellerThrustAtZeroAirspeed;
        // total thrust at current smoothedThrottle
        float totalThrust = engineCount * perEngineMaxThrust * thrustCurveValue * smoothedThrottle;
        // Add forward thrust
        rb.AddForce(transform.forward * totalThrust * Time.fixedDeltaTime, ForceMode.Force);

        // --- Propeller torque reaction (yaw roll due to torque) ---
        // We'll add a small yaw torque proportional to throttle * engineCount
        float propTorqueYaw = (smoothedThrottle - 0.5f) * engineCount * 800f; // tweakable
        rb.AddTorque(transform.up * propTorqueYaw * Time.fixedDeltaTime, ForceMode.Force);
    }

    void ApplyControlTorques()
    {
        // If stalled, controls are much less effective (especially elevator & ailerons)
        float controlEffect = isStalled ? 0.25f : 1f;

        // Elevator (pitch) -> torque around local X (right)
        float pitchTorque = inputPitch * elevatorPower * controlEffect;
        rb.AddTorque(transform.right * pitchTorque * Time.fixedDeltaTime, ForceMode.Force);

        // Ailerons (roll) -> torque around local Z (forward)
        float rollTorque = -inputRoll * aileronPower * controlEffect; // negative sign for intuitive mapping
        rb.AddTorque(transform.forward * rollTorque * Time.fixedDeltaTime, ForceMode.Force);

        // Rudder (yaw) -> torque around local Y (up)
        float yawTorque = inputYaw * rudderPower * controlEffect;
        rb.AddTorque(transform.up * yawTorque * Time.fixedDeltaTime, ForceMode.Force);
    }

    void ApplyTurbulenceAndStallBehavior()
    {
        // gentle continuous turbulence
        float t = Time.time * turbulenceFreq;
        float nx = (Mathf.PerlinNoise(t, 0.1f) - 0.5f) * 2f;
        float ny = (Mathf.PerlinNoise(0.2f, t) - 0.5f) * 2f;
        float nz = (Mathf.PerlinNoise(0.3f, t * 0.7f) - 0.5f) * 2f;
        Vector3 turb = new Vector3(nx, ny, nz) * turbulenceAmplitude * 2000f;
        rb.AddTorque(turb * Time.fixedDeltaTime, ForceMode.Force);

        if (isStalled)
        {
            // apply random chaotic torque to simulate wing drop / buffet
            Vector3 chaos = new Vector3(
                (Random.value - 0.5f) * stallYawChaos,
                (Random.value - 0.5f) * stallYawChaos,
                (Random.value - 0.5f) * stallYawChaos
            );
            rb.AddTorque(chaos * Time.fixedDeltaTime, ForceMode.Force);

            // add extra angular damping to simulate loss of smooth controllability
            rb.angularVelocity *= (1f - stallRecoveryDamping * Time.fixedDeltaTime);
        }
    }

    void ApplyBrakesIfNeeded()
    {
        if (inputBrakes)
        {
            // add linear drag to slow the plane
            rb.linearVelocity *= (1f - brakeDragFactor * Time.fixedDeltaTime);
        }
    }

    void AnimateProps()
    {
        if (engineTransforms == null || engineTransforms.Length == 0) return;

        // compute approximate RPM from smoothedThrottle
        float rpm = Mathf.Lerp(idleRPM, maxEngineRPM, smoothedThrottle);
        float revsPerSecond = rpm / 60f;
        // rotate props visually (simple)
        foreach (Transform t in engineTransforms)
        {
            if (t == null) continue;
            t.Rotate(Vector3.forward, revsPerSecond * 360f * Time.fixedDeltaTime, Space.Self);
        }
    }

    // Optional small editor helper
    void OnDrawGizmosSelected()
    {
        Gizmos.color = isStalled ? Color.red : Color.green;
        Gizmos.DrawWireSphere(transform.position + transform.up * 2f, 0.35f);
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position, transform.position + transform.forward * 10f);
    }

    // === Helpful debug / fix suggestions (don't modify runtime) ===
    // - Ensure planeCollider exists and if using MeshCollider -> set Convex = true when RB present.
    // - Rigidbody: Collision Detection = Continuous, Interpolate = Interpolate, Use Gravity = true.
    // - Do NOT move the GameObject via transform.position while physics is enabled; use AddForce/AddTorque only.
}
