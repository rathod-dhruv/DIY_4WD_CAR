/*
 * ============================================================
 *  AcebottCarController.cs  —  XR Interaction Toolkit version
 *  Works on: Meta Quest 3, Pico 4 Ultra, any OpenXR device
 * ============================================================
 *
 *  PACKAGES NEEDED (Window → Package Manager):
 *  ─────────────────────────────────────────────────────────
 *  1. XR Interaction Toolkit  (com.unity.xr.interaction.toolkit)
 *     → also install "Starter Assets" sample from its Samples tab
 *  2. XR Plugin Management    (com.unity.xr.management)
 *  3. OpenXR Plugin           (com.unity.xr.openxr)
 *     → Project Settings → XR → OpenXR → add your headset feature
 *       Quest:  "Meta Quest Support"  feature
 *       Pico:   "PICO OpenXR" feature  (needs Pico Unity SDK too)
 *  4. NativeWebSocket
 *     → Package Manager → + → git URL:
 *        https://github.com/endel/NativeWebSocket.git#upm
 *     → OR manually copy WebSocket folder into Assets/Plugins/WebSocket/
 *
 *  SCENE SETUP:
 *  ─────────────────────────────────────────────────────────
 *  1. Add XR Origin (Action Based) to scene
 *     (GameObject → XR → XR Origin)
 *  2. Create empty GameObject "CarController"
 *  3. Attach this script
 *  4. In Inspector:
 *     - Esp32 Ip: your ESP32's IP from Serial Monitor
 *     - Left Hand Device / Right Hand Device:
 *       drag in the Left/Right Controller objects from XR Origin
 *     - Head Camera: drag in "Main Camera" inside XR Origin
 *     - (optional) UI Text fields for sensor HUD
 *
 *  CONTROLS (XRI / OpenXR standard — same on Quest and Pico):
 *  ─────────────────────────────────────────────────────────
 *  Left  joystick            → drive (forward / back / strafe)
 *  Right joystick horizontal → spin left / right
 *  Head yaw rotation         → servo pan (ultrasonic follows gaze)
 *  Primary   button (A / X)  → honk (hold)
 *  Secondary button (B / Y)  → toggle LEDs
 *  Left  grip                → speed down
 *  Right grip                → speed up
 * ============================================================
 */

using System;
using System.Collections;
using UnityEngine;
using UnityEngine.XR;                     // XRNode, InputDevice
using UnityEngine.XR.Interaction.Toolkit; // XRI core
using NativeWebSocket;
using OHGAR;                              // XRKJOutputSO

// ── Sensor data — matches ESP32 JSON ────────────────────────
[Serializable]
public class SensorData {
    public int    d;     // distance cm  (999 = no echo)
    public int    l;     // trace left
    public int    m;     // trace middle
    public int    r;     // trace right
    public int    ir;    // IR receiver
    public float  t;     // temperature °C
    public int    obs;   // obstacle present
    public string zone;  // SAFE / WARN / DANGER / NO_ECHO
}

public class AcebottCarController : MonoBehaviour
{
    // ════════════════════════════════════════════════════════
    //  INSPECTOR FIELDS
    // ════════════════════════════════════════════════════════

    [Header("WebSocket")]
    [Tooltip("IP printed in Serial Monitor after flashing sketch")]
    public string esp32Ip = "192.168.1.100";
    public int    wsPort  = 81;

    [Header("XR References")]
    [Tooltip("Drag the Main Camera (inside XR Origin) here")]
    public Transform headCamera;

    [Header("Joystick Scriptable Object")]
    [Tooltip("Drag your XRKJOutputSO asset here — Value.z = fwd/back, Value.x = strafe")]
    public XRKJOutputSO joystickOutput;  // ← assign in Inspector

    [Header("Input Tuning")]
    [Range(0.05f, 0.3f)]
    [Tooltip("Ignore joystick movement smaller than this")]
    public float deadzone     = 0.15f;

    [Tooltip("Seconds between drive command sends")]
    public float sendInterval = 0.05f;   // 20 Hz

    [Tooltip("Degrees of head turn before updating servo")]
    public float servoThreshold = 2f;

    [Header("Sensor HUD (optional — assign UI Text objects)")]
    public UnityEngine.UI.Text distanceText;
    public UnityEngine.UI.Text zoneText;
    public UnityEngine.UI.Text traceText;
    public UnityEngine.UI.Text tempText;
    public UnityEngine.UI.Text speedText;

    [Header("Visual Obstacle Indicator (optional)")]
    [Tooltip("A Renderer whose color changes with obstacle zone")]
    public Renderer obstacleIndicator;

    // ════════════════════════════════════════════════════════
    //  PRIVATE STATE
    // ════════════════════════════════════════════════════════
    WebSocket ws;
    bool      isConnected  = false;

    float sendTimer       = 0f;
    float lastServoAngle  = 90f;
    bool  ledsOn          = false;
    float currentSpeed    = 180f;
    const float SPD_MIN   = 80f;
    const float SPD_MAX   = 255f;
    string lastDriveCmd   = "";

    // ── XRI Input Devices ────────────────────────────────────
    // InputDevice is the XRI/OpenXR abstraction — works on any headset.
    // We look them up by XRNode (LeftHand / RightHand).
    // These are found at runtime, not assigned in Inspector,
    // because they may not exist at Start() on some platforms.
    InputDevice leftController;
    InputDevice rightController;

    // Button state tracking (to detect press/release edges)
    bool prevPrimary   = false;   // A (Quest) / X (Pico left) — honk
    bool prevSecondary = false;   // B / Y — LEDs
    bool prevLeftGrip  = false;   // speed down
    bool prevRightGrip = false;   // speed up

    // ════════════════════════════════════════════════════════
    //  UNITY LIFECYCLE
    // ════════════════════════════════════════════════════════
    void Start() {
        // Auto-find head camera if not assigned
        if (headCamera == null && Camera.main != null)
            headCamera = Camera.main.transform;

        StartCoroutine(ConnectWebSocket());
    }

    void Update() {
        if (ws == null) return;

        // ── REQUIRED every frame: flush incoming WS messages ──
        // NativeWebSocket receives on a background thread;
        // this call moves them safely onto the Unity main thread.
        #if !UNITY_WEBGL || UNITY_EDITOR
            ws.DispatchMessageQueue();
        #endif

        if (!isConnected) return;

        // Try to find controllers if not yet found
        // (they may connect after Start on some runtimes)
        if (!leftController.isValid)
            leftController  = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        if (!rightController.isValid)
            rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);

        // Drive at fixed rate
        sendTimer += Time.deltaTime;
        if (sendTimer >= sendInterval) {
            sendTimer = 0f;
            HandleDriveInput();
        }

        HandleHeadTracking();
        HandleButtons();
    }

    void OnApplicationQuit() {
        if (isConnected) Send("S");
        ws?.Close();
    }

    // ════════════════════════════════════════════════════════
    //  WEBSOCKET CONNECTION
    // ════════════════════════════════════════════════════════
    IEnumerator ConnectWebSocket() {
        string url = $"ws://{esp32Ip}:{wsPort}";
        Debug.Log($"[Car] Connecting → {url}");

        ws = new WebSocket(url);

        ws.OnOpen    += ()      => { isConnected = true;
                                     Debug.Log("[Car] Connected!");
                                     Send("V" + (int)currentSpeed); };

        ws.OnClose   += (code)  => { isConnected = false;
                                     Debug.Log($"[Car] Disconnected ({code})"); };

        ws.OnMessage += OnMessage;

        ws.OnError   += (err)   => Debug.LogError($"[Car] WS Error: {err}");

        ws.Connect();

        // Wait up to 10s for connection
        float t = 0f;
        while (!isConnected && t < 10f) { t += Time.deltaTime; yield return null; }

        if (!isConnected)
            Debug.LogError("[Car] Timeout. Is the ESP32 on the same WiFi? Check IP.");
    }

    // ════════════════════════════════════════════════════════
    //  RECEIVE SENSOR JSON FROM ESP32
    // ════════════════════════════════════════════════════════
    void OnMessage(byte[] bytes) {
        string json = System.Text.Encoding.UTF8.GetString(bytes);
        SensorData data;
        try   { data = JsonUtility.FromJson<SensorData>(json); }
        catch { return; }

        // ── Update HUD text ─────────────────────────────────
        if (distanceText != null)
            distanceText.text = data.d >= 999 ? "—" : $"{data.d} cm";

        if (zoneText != null) {
            zoneText.text  = data.zone ?? "—";
            zoneText.color = data.zone switch {
                "DANGER" => Color.red,
                "WARN"   => Color.yellow,
                "SAFE"   => Color.green,
                _        => Color.gray
            };
        }

        if (traceText != null)
            traceText.text = $"Trace  {data.l} · {data.m} · {data.r}";

        if (tempText != null)
            tempText.text = $"{data.t:F1} °C";

        // ── Obstacle indicator color ─────────────────────────
        if (obstacleIndicator != null) {
            obstacleIndicator.material.color = data.zone switch {
                "DANGER" => Color.red,
                "WARN"   => new Color(1f, 0.6f, 0f),
                "SAFE"   => Color.green,
                _        => Color.gray
            };
        }

        // ── Haptic rumble (XRI — works on Quest AND Pico) ────
        // XRI uses InputDevice.SendHapticImpulse instead of OVRInput
        // amplitude 0..1, duration in seconds
        if (data.obs == 1 && rightController.isValid) {
            float amp = data.zone == "DANGER" ? 0.8f : 0.3f;
            // channel 0 = default haptic motor
            rightController.SendHapticImpulse(0, amp, 0.1f);
        }
    }

    // ════════════════════════════════════════════════════════
    //  DRIVE INPUT  — reads from XRKJOutputSO Scriptable Object
    // ════════════════════════════════════════════════════════
    //
    //  XRKJOutputSO.Value is a Vector3 set by your joystick system:
    //
    //    Value.z  (+)  → FORWARD   (car moves forward)
    //    Value.z  (-)  → BACKWARD  (car moves backward)
    //    Value.x  (+)  → STRAFE RIGHT (Mecanum sideways)
    //    Value.x  (-)  → STRAFE LEFT  (Mecanum sideways)
    //    Value.y       → unused (ignored)
    //
    //  Right controller primary2DAxis.x is STILL used for spinning.
    //  Everything else (buttons, head tracking) works as before.
    // Tuning
  
    [SerializeField] private int maxPwm = 220;            // cap speed (0-255)
    [SerializeField] private int minPwm = 80;             // minimum usable speed when moving
    [SerializeField] private float sendRateHz = 20f;      // limit network spam

    private float _nextSendTime;
    private string _lastCmd = "S";
    private int _lastPwm = -1;

    private void HandleDriveInput()
    {
        if (joystickOutput == null) return;

        Vector3 v = joystickOutput.Value;
        float fwd = Mathf.Clamp(v.z, -1f, 1f);
        float str = Mathf.Clamp(v.x, -1f, 1f);

        if (Mathf.Abs(fwd) < deadzone) fwd = 0f;
        if (Mathf.Abs(str) < deadzone) str = 0f;

        string cmd = "S";
        float mag = 0f;

        if (fwd == 0f && str == 0f)
        {
            cmd = "S";
            mag = 0f;
        }
        else if (Mathf.Abs(fwd) >= Mathf.Abs(str))
        {
            cmd = (fwd > 0f) ? "F" : "B";
            mag = Mathf.Abs(fwd);
        }
        else
        {
            cmd = (str > 0f) ? "SR" : "SL";
            mag = Mathf.Abs(str);
        }

        int pwm = 0;
        if (cmd != "S")
        {
            pwm = Mathf.RoundToInt(Mathf.Lerp(minPwm, maxPwm, mag));
            pwm = Mathf.Clamp(pwm, 0, 255);
        }

        if (Time.time < _nextSendTime) return;
        _nextSendTime = Time.time + (1f / sendRateHz);

        bool cmdChanged = cmd != _lastCmd;
        bool pwmChanged = Mathf.Abs(pwm - _lastPwm) >= 5;

        if (cmd == "S")
        {
            if (_lastCmd != "S")
            {
                Send("S");
                _lastCmd = "S";
                _lastPwm = 0;
            }
            return;
        }

        if (pwmChanged)
        {
            Send($"V{pwm}");
            _lastPwm = pwm;
        }

        if (cmdChanged)
        {
            Send(cmd);
            _lastCmd = cmd;
        }
    }
    //Helper: safely read a 2D axis (returns zero if device not ready)
    Vector2 GetAxis2D(InputDevice device, InputFeatureUsage<Vector2> usage) {
        if (!device.isValid) return Vector2.zero;
        device.TryGetFeatureValue(usage, out Vector2 val);
        return val;
    }

    // Helper: safely read a bool button
    bool GetButton(InputDevice device, InputFeatureUsage<bool> usage) {
        if (!device.isValid) return false;
        device.TryGetFeatureValue(usage, out bool val);
        return val;
    }

    // ════════════════════════════════════════════════════════
    //  HEAD TRACKING → SERVO
    // ════════════════════════════════════════════════════════
    void HandleHeadTracking() {
        if (headCamera == null) return;

        // eulerAngles.y is 0..360. Normalize to -180..180.
        float yaw = headCamera.eulerAngles.y;
        if (yaw > 180f) yaw -= 360f;

        yaw = Mathf.Clamp(yaw, -90f, 90f);

        // Map yaw (-90..+90) → servo angle (0..180)
        float angle = Mathf.Lerp(0f, 180f, (yaw + 90f) / 180f);

        if (Mathf.Abs(angle - lastServoAngle) > servoThreshold) {
            lastServoAngle = angle;
            Send($"HEAD:{(int)yaw}:0");
        }
    }

    // ════════════════════════════════════════════════════════
    //  BUTTON INPUTS  (XRI CommonUsages — cross-platform)
    // ════════════════════════════════════════════════════════
    // CommonUsages.primaryButton   = A (right) / X (left)
    // CommonUsages.secondaryButton = B (right) / Y (left)
    // CommonUsages.gripButton      = grip squeeze
    //
    // These names are the SAME on Quest and Pico because
    // OpenXR standardizes them — no device-specific code needed!

    void HandleButtons() {
        // ── Read current button states ───────────────────────
        bool primary   = GetButton(rightController, CommonUsages.primaryButton);
        bool secondary = GetButton(rightController, CommonUsages.secondaryButton);
        bool leftGrip  = GetButton(leftController,  CommonUsages.gripButton);
        bool rightGrip = GetButton(rightController, CommonUsages.gripButton);

        // ── A button (right) / X button (left) → HONK ────────
        // Hold = honk on, release = honk off
        if (primary  && !prevPrimary)  Send("H1");  // pressed
        if (!primary &&  prevPrimary)  Send("H0");  // released

        // ── B button (right) / Y button (left) → TOGGLE LEDs ─
        if (secondary && !prevSecondary) {
            ledsOn = !ledsOn;
            Send("D" + (ledsOn ? "1" : "0"));
            Debug.Log($"[Car] LEDs: {(ledsOn ? "ON" : "OFF")}");
        }

        // ── Left grip → speed DOWN ────────────────────────────
        if (leftGrip && !prevLeftGrip) {
            currentSpeed = Mathf.Max(SPD_MIN, currentSpeed - 20f);
            Send("V" + (int)currentSpeed);
            if (speedText != null) speedText.text = $"Speed {(int)currentSpeed}";
            Debug.Log($"[Car] Speed ↓ {currentSpeed}");
        }

        // ── Right grip → speed UP ─────────────────────────────
        if (rightGrip && !prevRightGrip) {
            currentSpeed = Mathf.Min(SPD_MAX, currentSpeed + 20f);
            Send("V" + (int)currentSpeed);
            if (speedText != null) speedText.text = $"Speed {(int)currentSpeed}";
            Debug.Log($"[Car] Speed ↑ {currentSpeed}");
        }

        // Store state for next frame (edge detection)
        prevPrimary   = primary;
        prevSecondary = secondary;
        prevLeftGrip  = leftGrip;
        prevRightGrip = rightGrip;
    }

    // ════════════════════════════════════════════════════════
    //  SEND HELPER
    // ════════════════════════════════════════════════════════
    void Send(string cmd) {
        if (ws == null || !isConnected) return;
        ws.SendText(cmd);
    }

    // ════════════════════════════════════════════════════════
    //  PUBLIC API  (call from UI buttons or other scripts)
    // ════════════════════════════════════════════════════════
    public void DriveForward()      => Send("F");
    public void DriveBackward()     => Send("B");
    public void SpinLeft()          => Send("L");
    public void SpinRight()         => Send("R");
    public void StrafeLeft()        => Send("SL");
    public void StrafeRight()       => Send("SR");
    public void StopCar()           => Send("S");
    public void Honk()              => Send("H1");
    public void SetServo(int a)     => Send("P" + a);
    public void ToggleLEDs() {
        ledsOn = !ledsOn;
        Send("D" + (ledsOn ? "1" : "0"));
    }
}
