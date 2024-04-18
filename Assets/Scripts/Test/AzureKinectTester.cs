using System;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Azure.Kinect.BodyTracking;
using Microsoft.Azure.Kinect.Sensor;
using UnityEngine;

public class AzureKinectTester : MonoBehaviour
{
    [SerializeField] private GameObject rightHand;
    [SerializeField] private GameObject leftHand;
    
    private bool isSkeletalTrackingRunning = false;
    private CancellationTokenSource cancellationTokenSource;
    private CancellationToken token;

    private ulong numOfBodies;
    private Body[] bodies;
    
    void Start()
    {
        bodies = new Body[20];
        for (var i = 0; i < 20; i++)
        {
            bodies[i] = new Body(100);
        }

        cancellationTokenSource = new CancellationTokenSource();
        token = cancellationTokenSource.Token;
        Task.Run(() => RunBackgroundThreadAsync(0, token));
    }
    
    void OnApplicationQuit()
    {
        cancellationTokenSource?.Cancel();
        cancellationTokenSource?.Dispose();
        cancellationTokenSource = null;
    }
    
    void Update()
    {
        if (isSkeletalTrackingRunning && numOfBodies > 0)
        {
            UpdateTracker();
        }
    }

    private void UpdateTracker()
    {
        var closestBody = FindClosestTrackedBody();
        var skeleton = bodies[closestBody];
        
        var rHand = skeleton.JointPositions3D[(int)JointId.HandRight];
        rightHand.transform.position  = new Vector3(rHand.X, -rHand.Y, rHand.Z);
        var lHand = skeleton.JointPositions3D[(int)JointId.HandLeft];
        leftHand.transform.position  = new Vector3(lHand.X, -lHand.Y, lHand.Z);
    }
    
    private int FindClosestTrackedBody()
    {
        var closestBody = -1;
        var minDistanceFromKinect = 5000.0f; // kinectのmax distanceで初期化
        for (var i = 0; i < (int)numOfBodies; i++)
        {
            var pelvisPosition = bodies[i].JointPositions3D[(int)JointId.Pelvis];
            var pelvisPos = new Vector3(pelvisPosition.X, pelvisPosition.Y, pelvisPosition.Z);
            if (pelvisPos.magnitude < minDistanceFromKinect)
            {
                closestBody = i;
                minDistanceFromKinect = pelvisPos.magnitude;
            }
        }
        return closestBody;
    }

    private void RunBackgroundThreadAsync(int id, CancellationToken cancel)
    {
        using var device = Device.Open(id);
        device.StartCameras(new DeviceConfiguration
        {
            CameraFPS = FPS.FPS30,
            ColorResolution = ColorResolution.Off,
            DepthMode = DepthMode.NFOV_Unbinned,
            WiredSyncMode = WiredSyncMode.Standalone,
        });
        Debug.Log("Kinectの起動に成功しました。 id " + id + "sn:" + device.SerialNum);

        var deviceCalibration = device.GetCalibration();
        var trackerConfiguration = new TrackerConfiguration
        {
            ProcessingMode = TrackerProcessingMode.Cpu,
            SensorOrientation = SensorOrientation.Default
        };
        using var tracker = Tracker.Create(deviceCalibration, trackerConfiguration);
        Debug.Log("Trackerの作成に成功しました。");

        while (!cancel.IsCancellationRequested)
        {
            using (var sensorCapture = device.GetCapture())
            {
                tracker.EnqueueCapture(sensorCapture);
            }

            using (var frame = tracker.PopResult(TimeSpan.Zero, throwOnTimeout: false))
            {
                if (frame == null)
                {
                    Debug.LogWarning("Trackerからデータの読み込みがタイムアウトしました。");
                }
                else
                {
                    isSkeletalTrackingRunning = true;
                    numOfBodies = frame.NumberOfBodies;

                    for (uint i = 0; i < numOfBodies; i++)
                    {
                        bodies[i].CopyFromBodyTrackingSdk(frame.GetBody(i), deviceCalibration);
                    }
                }
            }
        }
    }
}
