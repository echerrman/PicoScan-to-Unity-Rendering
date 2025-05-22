using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Generic;
using System;

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

    [Header("Point Cloud Settings")]
    [Tooltip("Maximum number of points to keep in the cloud")]
    public int maxPoints = 10000;

    private UdpClient udpClient;
    private Thread receiveThread;
    private List<Vector3> allPoints = new List<Vector3>();
    private object lockObj = new object();
    private HashSet<Vector3> uniquePoints = new HashSet<Vector3>();

    void Start()
    {
        udpClient = new UdpClient(udpPort);
        receiveThread = new Thread(ReceiveData);
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void ReceiveData()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, udpPort);
        while (true)
        {
            try
            {
                byte[] data = udpClient.Receive(ref remoteEndPoint);
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
                    foreach (var pt in points)
                        uniquePoints.Add(pt);
                    allPoints = new List<Vector3>(uniquePoints);
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
    }

    void OnApplicationQuit()
    {
        if (receiveThread != null && receiveThread.IsAlive)
            receiveThread.Abort();
        if (udpClient != null)
            udpClient.Close();
    }
}
