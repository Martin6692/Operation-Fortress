using UnityEngine;

/// <summary>
/// BomberFlightController
/// Realistische vereinfachte Aerodynamik:
/// - Schub (thrust)
/// - Auftrieb (lift) abhängig von Geschwindigkeit & Anstellwinkel
/// - Strömungsabriss (stall) über einem kritischen Anstellwinkel
/// - Pitch / Yaw / Roll Controls
/// - leichte einstellbare Turbulenz (wackeln)
/// - automatisches Weiterfliegen wenn keine Eingabe
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class BomberFlightController : MonoBehaviour
{
    // --- Rigidbody / Physik ---
    Rigidbody rb;
    [Header("Rigidbody Settings")]
    public float mass = 12000f;                   // kg, große Bomberwerte
    public Vector3 centerOfMassOffset = new Vector3(0, -0.5f, 0f);

    // --- Aerodynamik-Grundgrößen ---
    [Header("Aerodynamics")]
    public float wingArea = 120f;                 // m^2, Flügelfläche
    public float airDensity = 1.225f;             // kg/m^3 (Sea level)
    public float baseLiftCoefficient = 0.5f;      // Baseline Lift coefficient (Cl0)
    public AnimationCurve liftCurve = AnimationCurve.Linear(-15f, 0.1f, 15f, 1.0f);
    public float stallAngle = 15f;                // deg: oberhalb -> Stall
    public float stallLiftMultiplier = 0.2f;      // Lift reduziert bei Stall
    public float dragCoefficient = 0.025f;        // Parasite drag
    public float inducedDragFactor = 0.04f;       // sinkender Auftrieb erhöht induzierte Luftwiderstände

    // --- Schub (Thrust) ---
    [Header("Thrust")]
    public float maxThrust = 200000f;             // N (maximaler Schub)
    public float minThrust = 0f;
    [Range(0f,1f)] public float thrustInput = 0.5f; // Inspector readout / debug

    // --- Steuerung (Inputs sind normalisiert -1..1) ---
    [Header("Controls")]
    public float pitchPower = 40000f;             // Drehmoment für Nick
    public float rollPower  = 30000f;             // Drehmoment für Rolle
    public float yawPower   = 15000f;             // Seitenruder
    public float controlSensitivity = 1.0f;

    // --- Automatische Vorwärtsbewegung / Dämpfung ---
    [Header("Auto Forward & Damping")]
    public float autoThrottle = 0.25f;            // bei null Eingabe, minimaler Schub
    public float forwardSpeedForNeutral = 60f;    // m/s: gewünschte "cruise" speed (nur als referenz)

    // --- Turbulenz / Wackeln ---
    [Header("Turbulence / Wobble")]
    public float turbulenceAmplitude = 0.5f;      // grad, max Wackeln
    public float turbulenceFrequency = 0.4f;      // Hz
    public float turbulenceInfluenceOnTorque = 2000f; // wie viel Drehmoment Turbulenz verursacht

    // --- Debug / Tuning ---
    [Header("Debug / Tuning")]
    public bool enableDebugForces = false;

    // internal
    Vector3 localVelocity => transform.InverseTransformDirection(rb.linearVelocity);
    float forwardSpeed => Vector3.Dot(rb.linearVelocity, transform.forward);
    float angleOfAttackDeg; // aktuell berechneter Anstellwinkel

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.mass = mass;
        rb.centerOfMass = centerOfMassOffset;
        rb.angularDamping = 1.5f;
        rb.linearDamping = 0.02f;
    }

    void FixedUpdate()
    {
        // 1) Inputs (du kannst das an dein Input-System binden)
        float pitchInput = Input.GetAxis("Vertical") * controlSensitivity; // -1..1
        float rollInput  = Input.GetAxis("Horizontal") * controlSensitivity; // -1..1
        float yawInput   = 0f;
        if (Input.GetKey(KeyCode.Q)) yawInput = -1f;
        if (Input.GetKey(KeyCode.E)) yawInput =  1f;
        float thrustAxis = Input.GetAxis("Thrust"); // mappe in InputManager "Thrust" oder nutze "Fire1" als Ersatz

        // fallback: wenn kein Thrust-Axis vorhanden, nutze W/S
        if (Mathf.Approximately(thrustAxis, 0f))
            thrustAxis = Input.GetAxis("Fire1"); // optional

        // set thrustInput für Inspector-Visibility und automatische Weiterfahrt
        if (Mathf.Approximately(thrustAxis, 0f))
            thrustInput = Mathf.Lerp(thrustInput, autoThrottle, Time.fixedDeltaTime * 0.5f);
        else
            thrustInput = Mathf.Clamp01((thrustAxis + 1f) * 0.5f); // falls -1..1 range

        // 2) Thrust-Kraft entlang Flugzeug-Forward
        float currentThrust = Mathf.Lerp(minThrust, maxThrust, thrustInput);
        Vector3 thrustForce = transform.forward * currentThrust;
        rb.AddForce(thrustForce * Time.fixedDeltaTime, ForceMode.Force);

        // 3) Aerodynamischer Auftrieb basierend auf Anstellwinkel & Geschwindigkeit
        Vector3 worldVelocity = rb.linearVelocity;
        float speed = worldVelocity.magnitude;
        // Anstellwinkel = Winkel zwischen Flugzeug-Längsachse und Velocity (in Lokalraum)
        if (speed > 0.1f)
        {
            Vector3 velLocal = transform.InverseTransformDirection(worldVelocity);
            // AoA berechnen: angle between forward vector (z) and velocity projected on plane vertical (z-y)
            angleOfAttackDeg = Mathf.Atan2(-velLocal.y, velLocal.z) * Mathf.Rad2Deg; // negative weil Unity y up
        }
        else
        {
            angleOfAttackDeg = 0f;
        }

        // Lift-Koeff via Kurve (besseres Tuning)
        float cl = baseLiftCoefficient * liftCurve.Evaluate(angleOfAttackDeg);
        bool inStall = Mathf.Abs(angleOfAttackDeg) > stallAngle;
        if (inStall)
        {
            cl *= stallLiftMultiplier; // starker Lift-Verlust
        }

        // Lift-Formel: L = 0.5 * rho * V^2 * S * Cl
        float liftMagnitude = 0.5f * airDensity * speed * speed * wingArea * cl;
        // Auftrieb wirkt senkrecht zur Flugrichtung, in Flugzeug-Lokalraum normal nach oben (transform.up)
        Vector3 liftForce = transform.up * liftMagnitude;
        rb.AddForce(liftForce * Time.fixedDeltaTime, ForceMode.Force);

        // 4) Drag (einfach): parasitärer Drag + induzierter Drag ~ L^2
        float inducedDrag = inducedDragFactor * (liftMagnitude * liftMagnitude) / (0.5f * airDensity * wingArea);
        float parasiteDrag = 0.5f * airDensity * speed * speed * wingArea * dragCoefficient;
        Vector3 totalDrag = -rb.linearVelocity.normalized * (parasiteDrag + inducedDrag);
        rb.AddForce(totalDrag * Time.fixedDeltaTime, ForceMode.Force);

        // 5) Steuerung: Drehmomente (Pitch / Yaw / Roll)
        Vector3 torque = Vector3.zero;
        // Pitch: um lokale X-Achse (Nick)
        torque += transform.right * (pitchInput * pitchPower);
        // Roll: um lokale Z-Achse (Rolle) -> in Unity roll ist Rotation um forward axis => use -forward for sign
        torque += transform.forward * (-rollInput * rollPower);
        // Yaw: um lokale Y-Achse (Gieren)
        torque += transform.up * (yawInput * yawPower);

        // Wenn im Stall: Steuerkräfte reduziert
        if (inStall)
        {
            torque *= 0.25f; // kaum Kontrolle bei Strömungsabriss
        }

        rb.AddTorque(torque * Time.fixedDeltaTime, ForceMode.Force);

        // 6) Turbulenz / Wackeln: small Perlin noise applied as tiny torques
        float t = Time.time * turbulenceFrequency;
        float nx = (Mathf.PerlinNoise(t, 0.1f) - 0.5f) * 2f;
        float ny = (Mathf.PerlinNoise(0.2f, t) - 0.5f) * 2f;
        float nz = (Mathf.PerlinNoise(0.3f, t * 0.7f) - 0.5f) * 2f;
        Vector3 turbulenceTorque = new Vector3(nx, ny, nz) * turbulenceInfluenceOnTorque * turbulenceAmplitude;
        rb.AddTorque(turbulenceTorque * Time.fixedDeltaTime, ForceMode.Force);

        // 7) Kleine automatische Stabilisierung (Dämpfung) - optional: hilft Flieger nicht sofort überschlagen
        ApplyAngularStabilization();

        // Debug-Visuals
        if (enableDebugForces)
        {
            Debug.DrawLine(transform.position, transform.position + thrustForce.normalized * 10f, Color.yellow);
            Debug.DrawLine(transform.position, transform.position + liftForce.normalized * 10f, Color.green);
            Debug.DrawLine(transform.position, transform.position + totalDrag.normalized * 10f, Color.red);
        }
    }

    void ApplyAngularStabilization()
    {
        // leichte automatische Rückführung um überschüssige Rotationen zu dämpfen (aber nicht zu stark)
        float angularSpeed = rb.angularVelocity.magnitude;
        float dampingFactor = 0.02f;
        rb.AddTorque(-rb.angularVelocity * rb.mass * dampingFactor * 10f, ForceMode.Force);
    }

    // Hilfsfunktionen (können erweitert werden)
    public bool IsStalled()
    {
        return Mathf.Abs(angleOfAttackDeg) > stallAngle;
    }

    // Optional: Visualisierung im Editor
    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position, transform.position + transform.forward * forwardSpeedForNeutral);
        Gizmos.color = IsStalled() ? Color.red : Color.green;
        Gizmos.DrawWireSphere(transform.position + transform.up * 2f, 0.3f);
    }
}
