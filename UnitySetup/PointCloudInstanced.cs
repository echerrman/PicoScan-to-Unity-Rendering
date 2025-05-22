using UnityEngine;
using UnityEngine.UI;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;
using System;
using System.Text;

public class PointCloudInstanced : MonoBehaviour
{
    [Header("Point Cloud Rendering")]
    public Mesh pointMesh; // Assign your Quad or Sphere asset in Inspector
    public Material pointMaterial; // Assign InstancedPointMaterial in Inspector
    [Tooltip("Size of each rendered point")]
    public float pointSize = 0.05f;

    [Header("UDP Settings")]
    [Tooltip("UDP port to listen for point cloud data")]
    public int udpPort = 5005;
    [Tooltip("UDP port to send control commands")]
    public int controlPort = 5006;

    [Header("Point Cloud Settings")]
    [Tooltip("Maximum number of points to keep in the cloud")]
    public int maxPoints = 10000;

    [Header("UI Controls")]
    [Tooltip("Button to toggle between Persistent and Live-only modes")]
    public Button toggleModeButton;
    [Tooltip("Button to clear all points")]
    public Button clearPointsButton;
    [Tooltip("Text to display current mode")]
    public Text modeStatusText;
    [Tooltip("Text to display point count")]
    public Text pointCountText;

    private UdpClient udpClient;
    private UdpClient controlClient;
    private Thread receiveThread;
    private List<Vector3> allPoints = new List<Vector3>();
    private object lockObj = new object();
    private HashSet<Vector3> uniquePoints = new HashSet<Vector3>();
    
    private bool isPersistentMode = true;
    private string currentMode = "PERSISTENT";
    private int currentPointCount = 0;

    void Start()
    {
        // Setup UDP clients
        udpClient = new UdpClient(udpPort);
        controlClient = new UdpClient();
        
        // Setup receive thread
        receiveThread = new Thread(ReceiveData);
        receiveThread.IsBackground = true;
        receiveThread.Start();

        // Setup UI
        SetupUI();
        UpdateUI();
    }

    void SetupUI()
    {
        if (toggleModeButton != null)
        {
            toggleModeButton.onClick.AddListener(ToggleMode);
        }

        if (clearPointsButton != null)
        {
            clearPointsButton.onClick.AddListener(ClearPoints);
        }
    }

    void UpdateUI()
    {
        if (modeStatusText != null)
        {
            modeStatusText.text = $"Mode: {currentMode}";
        }

        if (pointCountText != null)
        {
            pointCountText.text = $"Points: {currentPointCount}";
        }

        if (toggleModeButton != null)
        {
            Text buttonText = toggleModeButton.GetComponentInChildren<Text>();
            if (buttonText != null)
            {
                buttonText.text = isPersistentMode ? "Switch to Live Only" : "Switch to Persistent";
            }
        }
    }

    public void ToggleMode()
    {
        isPersistentMode = !isPersistentMode;
        string command = isPersistentMode ? "PERSISTENT" : "LIVE_ONLY";
        SendControlCommand(command);
        currentMode = command;
        
        if (!isPersistentMode)
        {
            // Clear points when switching to live-only mode
            lock (lockObj)
            {
                allPoints.Clear();
                uniquePoints.Clear();
            }
        }
        
        UpdateUI();
        Debug.Log($"Switched to {command} mode");
    }

    public void ClearPoints()
    {
        SendControlCommand("CLEAR");
        lock (lockObj)
        {
            allPoints.Clear();
            uniquePoints.Clear();
        }
        UpdateUI();
        Debug.Log("Cleared all points");
    }

    private void SendControlCommand(string command)
    {
        try
        {
            byte[] data = Encoding.UTF8.GetBytes(command);
            controlClient.Send(data, data.Length, "127.0.0.1", controlPort);
        }
        catch (Exception ex)
        {
            Debug.LogError("Error sending control command: " + ex.Message);
        }
    }

    void ReceiveData()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, udpPort);
        bool expectingPoints = false;
        List<Vector3> framePoints = new List<Vector3>();

        while (true)
        {
            try
            {
                byte[] data = udpClient.Receive(ref remoteEndPoint);
                string message = Encoding.UTF8.GetString(data, 0, Math.Min(10, data.Length));

                if (message.StartsWith("LIVE:"))
                {
                    // Fast path for live-only mode - immediate processing
                    int pointDataStart = 5; // Skip "LIVE:" prefix
                    int numPoints = (data.Length - pointDataStart) / (3 * 4);
                    List<Vector3> points = new List<Vector3>(numPoints);

                    for (int i = 0; i < numPoints; i++)
                    {
                        int offset = pointDataStart + i * 12;
                        float x = BitConverter.ToSingle(data, offset);
                        float y = BitConverter.ToSingle(data, offset + 4);
                        float z = BitConverter.ToSingle(data, offset + 8);
                        points.Add(new Vector3(x, y, z));
                    }

                    // Immediately replace points for live mode
                    lock (lockObj)
                    {
                        allPoints = new List<Vector3>(points);
                        uniquePoints = new HashSet<Vector3>(points);
                        currentMode = "LIVE_ONLY";
                    }
                }
                else if (message.StartsWith("MODE:"))
                {
                    // Mode information received
                    string mode = Encoding.UTF8.GetString(data, 5, data.Length - 5);
                    currentMode = mode;
                    framePoints.Clear();
                    expectingPoints = true;
                }
                else if (message.StartsWith("POINTS:"))
                {
                    // Point data received
                    if (expectingPoints)
                    {
                        int pointDataStart = 7; // Skip "POINTS:" prefix
                        int numPoints = (data.Length - pointDataStart) / (3 * 4);

                        for (int i = 0; i < numPoints; i++)
                        {
                            int offset = pointDataStart + i * 12;
                            float x = BitConverter.ToSingle(data, offset);
                            float y = BitConverter.ToSingle(data, offset + 4);
                            float z = BitConverter.ToSingle(data, offset + 8);
                            framePoints.Add(new Vector3(x, y, z));
                        }
                    }
                }
                else if (message.StartsWith("END"))
                {
                    // End of frame - process all received points
                    if (expectingPoints)
                    {
                        lock (lockObj)
                        {
                            if (currentMode == "LIVE_ONLY")
                            {
                                // Replace all points with current frame
                                allPoints = new List<Vector3>(framePoints);
                                uniquePoints = new HashSet<Vector3>(framePoints);
                            }
                            else
                            {
                                // Add to existing points (persistent mode)
                                foreach (var pt in framePoints)
                                {
                                    if (uniquePoints.Add(pt))
                                    {
                                        allPoints.Add(pt);
                                    }
                                }
                            }
                        }
                        expectingPoints = false;
                    }
                }
                else
                {
                    // Legacy format support - assume all data is points
                    int numPoints = data.Length / (3 * 4);
                    List<Vector3> points = new List<Vector3>(numPoints);
                    
                    for (int i = 0; i < numPoints; i++)
                    {
                        int offset = i * 12;
                        float x = BitConverter.ToSingle(data, offset);
                        float y = BitConverter.ToSingle(data, offset + 4);
                        float z = BitConverter.ToSingle(data, offset + 8);
                        points.Add(new Vector3(x, y, z));
                    }

                    lock (lockObj)
                    {
                        if (currentMode == "LIVE_ONLY")
                        {
                            allPoints = new List<Vector3>(points);
                            uniquePoints = new HashSet<Vector3>(points);
                        }
                        else
                        {
                            foreach (var pt in points)
                            {
                                if (uniquePoints.Add(pt))
                                {
                                    allPoints.Add(pt);
                                }
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError("UDP Receive error: " + ex.Message);
            }
        }
    }

    void Update()
    {
        List<Vector3> pointsCopy;
        lock (lockObj)
        {
            pointsCopy = new List<Vector3>(allPoints);
            currentPointCount = pointsCopy.Count;
        }

        // Update UI on main thread
        UpdateUI();

        // Render points
        int batchSize = 1023; // Unity's limit per DrawMeshInstanced call
        Matrix4x4[] matrices = new Matrix4x4[batchSize];
        int count = pointsCopy.Count;
        
        for (int i = 0; i < count; i += batchSize)
        {
            int thisBatch = Mathf.Min(batchSize, count - i);
            for (int j = 0; j < thisBatch; j++)
            {
                matrices[j] = Matrix4x4.TRS(pointsCopy[i + j], Quaternion.identity, Vector3.one * pointSize);
            }
            Graphics.DrawMeshInstanced(pointMesh, 0, pointMaterial, matrices, thisBatch);
        }
    }

    void OnApplicationQuit()
    {
        if (receiveThread != null && receiveThread.IsAlive)
            receiveThread.Abort();
        if (udpClient != null)
            udpClient.Close();
        if (controlClient != null)
            controlClient.Close();
    }

    // Public methods for external control (e.g., from other scripts or inspector buttons)
    [ContextMenu("Toggle Mode")]
    public void ToggleModeFromMenu()
    {
        ToggleMode();
    }

    [ContextMenu("Clear Points")]
    public void ClearPointsFromMenu()
    {
        ClearPoints();
    }
}
