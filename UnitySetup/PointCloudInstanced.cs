using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;
using System;

public class PointCloudInstanced : MonoBehaviour
{
    [Header("Rendering Settings")]
    public Mesh pointMesh; // Assign your Quad or Sphere asset in Inspector
    public Material pointMaterial; // Assign InstancedPointMaterial in Inspector
    public float pointSize = 0.05f;

    [Header("Network Settings")]
    public int udpPort = 5005;
    public int maxPoints = 10000;
    public int bufferSize = 65536;

    [Header("Scanner Visualization")]
    public bool showScanner = true;
    public GameObject scannerPrefab; // Assign a prefab for the scanner visualization
    public Color scannerPathColor = Color.red;
    public bool drawScannerPath = true;
    public float pathPointInterval = 0.5f;

    // Internal state
    private UdpClient udpClient;
    private Thread receiveThread;
    private HashSet<Vector3> uniquePoints = new HashSet<Vector3>();
    private List<Vector3> allPoints = new List<Vector3>();
    private object lockObj = new object();
    private bool threadRunning = true;

        // Scanner tracking
    private Vector3 scannerPosition = Vector3.zero;
    private Quaternion scannerRotation = Quaternion.identity;
    private GameObject scannerObject;
    private List<Vector3> scannerPath = new List<Vector3>();
    private float lastPathPointDistance = 0f;
    
    // Performance tracking
    private int receivedPackets = 0;
    private int receivedPoints = 0;
    private float lastStatsTime = 0f;


    void Start()
    {
        // Create UDP client
        try
        {
            udpClient = new UdpClient(udpPort);
            udpClient.Client.ReceiveBufferSize = bufferSize;
            Debug.Log($"Listening for point cloud data on port {udpPort}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to initialize UDP client: {ex.Message}");
            return;
        }
        
        // Create scanner visualization if enabled
        if (showScanner && scannerPrefab != null)
        {
            scannerObject = Instantiate(scannerPrefab, scannerPosition, scannerRotation);
        }
        
        // Start receive thread
        threadRunning = true;
        receiveThread = new Thread(ReceiveData);
        receiveThread.IsBackground = true;
        receiveThread.Start();
        
        // Initialize performance tracking
        lastStatsTime = Time.time;
    }

    void ReceiveData()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        while (threadRunning)
        {
            try
            {
                byte[] data = udpClient.Receive(ref remoteEndPoint);
                receivedPackets++;

                // Check if this is a pose update packet
                if (data.Length >= 4 && data[0] == 'P' && data[1] == 'O' && data[2] == 'S' && data[3] == 'E')
                {
                    ProcessPoseData(data);
                }
                else
                {
                    ProcessPointCloudData(data);
                }
            }
            catch (SocketException ex)
            {
                if (threadRunning)
                {
                    Debug.LogWarning($"Socket exception: {ex.Message}");
                }
            }
            catch (ThreadAbortException)
            {
                break;
            }
            catch (Exception ex)
            {
                if (threadRunning)
                {
                    Debug.LogError($"UDP Receive error: {ex.Message}");
                }
            }
        }
    }

    void ProcessPoseData(byte[] data)
    {
        // Parse pose data: 'POSE' + position (xyz) + rotation (wxyz)
        if (data.Length >= 32)  // 4 bytes for POSE + 7*4 bytes for floats
        {
            float posX = BitConverter.ToSingle(data, 4);
            float posY = BitConverter.ToSingle(data, 8);
            float posZ = BitConverter.ToSingle(data, 12);
            
            float quatW = BitConverter.ToSingle(data, 16);
            float quatX = BitConverter.ToSingle(data, 20);
            float quatY = BitConverter.ToSingle(data, 24);
            float quatZ = BitConverter.ToSingle(data, 28);

            // Update scanner position and rotation
            lock (lockObj)
            {
                scannerPosition = new Vector3(posX, posY, posZ);
                scannerRotation = new Quaternion(quatX, quatY, quatZ, quatW);

                // Add to scanner path if enabled
                if (drawScannerPath)
                {
                    if (scannerPath.Count == 0)
                    {
                        scannerPath.Add(scannerPosition);
                    }
                    else
                    {
                        float distanceFromLast = Vector3.Distance(scannerPosition, scannerPath[scannerPath.Count - 1]);
                        lastPathPointDistance += distanceFromLast;

                        if (lastPathPointDistance >= pathPointInterval)
                        {
                            scannerPath.Add(scannerPosition);
                            lastPathPointDistance = 0f;
                        }
                    }
                }
            }
        }
    }

    void ProcessPointCloudData(byte[] data)
    {
        if (data.Length % 12 != 0)
        {
            Debug.LogWarning($"Received malformed data packet: {data.Length} bytes is not divisible by 12");
            return;
        }

        int numPoints = data.Length / 12;
        receivedPoints += numPoints;

        List<Vector3> newPoints = new List<Vector3>(numPoints);
        for (int i = 0; i < numPoints; i++)
        {
            int offset = i * 12;
            float x = BitConverter.ToSingle(data, offset);
            float y = BitConverter.ToSingle(data, offset + 4);
            float z = BitConverter.ToSingle(data, offset + 8);
            newPoints.Add(new Vector3(x, y, z));
        }

        lock (lockObj)
        {
            foreach (var point in newPoints)
            {
                uniquePoints.Add(point);

                // Limit the number of points to maxPoints
                if (uniquePoints.Count >= maxPoints)
                {
                    break;
                }
            }

            allPoints = new List<Vector3>(uniquePoints);
        }
    }

    void Update()
    {
        // Update scanner object position and rotation if available
        if (showScanner && scannerObject != null)
        {
            lock (lockObj)
            {
                scannerObject.transform.position = scannerPosition;
                scannerObject.transform.rotation = scannerRotation;
            }
        }

        // Render point cloud using instancing
        List<Vector3> pointsCopy;
        lock (lockObj)
        {
            pointsCopy = new List<Vector3>(allPoints);
        }

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

        // Draw scanner path if enabled
        if (drawScannerPath && scannerPath.Count > 1)
        {
            for (int i = 0; i < scannerPath.Count - 1; i++)
            {
                Debug.DrawLine(scannerPath[i], scannerPath[i + 1], scannerPathColor);
            }
        }

        // Log statistics every 5 seconds
        if (Time.time - lastStatsTime > 5.0f)
        {
            Debug.Log($"Point CLoud Stats: {uniquePoints.Count} points, {receivedPackets} packets received");
            lastStatsTime = Time.time;
            receivedPackets = 0;
            receivedPoints = 0;
        }
    }

    void OnApplicationQuit()
    {
        Cleanup();
    }

    void OnDestroy()
    {
        CleanUp();
    }

    void CleanUp()
    {
        threadRunning = false;
        
        if (receiveThread != null && receiveThread.IsAlive)
        {
            try
            {
                receiveThread.Join(500); // Wait 500ms for thread to exit gracefully
                if (receiveThread.IsAlive)
                    receiveThread.Abort();
            }
            catch {}
        }
        
        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }
    }

    // Helper method to clear all points
    public void ClearPoints()
    {
        lock (lockObj)
        {
            uniquePoints.Clear();
            allPoints.Clear();
        }
    }

}