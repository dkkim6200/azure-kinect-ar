using Microsoft.Azure.Kinect.Sensor;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.MixedReality.WebRTC.Unity;
using System;
using System.Threading;

namespace DKDevelopment.AzureKinect.Server
{
    public class PointCloudServer : MonoBehaviour
    {
        private static readonly int NUM_FLOATS_PER_VECTOR = 3;
        private static readonly int NUM_BYTES_PER_COLOR = 4;
        private static readonly int NUM_BYTES_PER_FLOAT = 4;

        public PeerConnection _peerConnection;
        private Microsoft.MixedReality.WebRTC.DataChannel _dataChannel;

        //Variable for handling Kinect
        private Device kinect;
        //Number of all points of PointCloud 
        private int numPoints;
        //Used to draw a set of points
        private Mesh mesh;
        //Array of coordinates for each point in PointCloud
        private Vector3[] vertices;
        //Array of colors corresponding to each point in PointCloud
        private Color32[] colors;
        //List of indexes of points to be rendered
        private int[] indices;
        //Class for coordinate transformation(e.g.Color-to-depth, depth-to-xyz, etc.)
        private Transformation transformation;
        
        // Byte array for Kinect data to send over to clients
        private byte[] webRTCData;
        private float[] verticesRaw;

        private Thread _mainThread;

        private void Start()
        {
            _mainThread = Thread.CurrentThread;

            //The method to initialize Kinect
            InitKinect();
            //Initialization for point cloud rendering
            InitMesh();
        }

        private void SetDataChannel(Microsoft.MixedReality.WebRTC.DataChannel channel)
        {
            _dataChannel = channel;
        }

        public void StartKinect()
        {
            _peerConnection.Peer.AddDataChannelAsync(42, "chat", true, true).ContinueWith((prevTask) =>
            { 
                if (prevTask.Exception != null)
                {
                    throw prevTask.Exception;
                }
                var newDataChannel = prevTask.Result;

                SetDataChannel(newDataChannel);
            });

            //Loop to get data from Kinect and rendering
            Task t = KinectLoop();
        }

        //Initialization of Kinect
        private void InitKinect()
        {
            //Connect with the 0th Kinect
            kinect = Device.Open(0);
            //Setting the Kinect operation mode and starting it
            kinect.StartCameras(new DeviceConfiguration
            {
                ColorFormat = ImageFormat.ColorBGRA32,
                ColorResolution = ColorResolution.R720p,
                DepthMode = DepthMode.NFOV_Unbinned,
                SynchronizedImagesOnly = true,
                CameraFPS = FPS.FPS30
            });
            //Access to coordinate transformation information
            transformation = kinect.GetCalibration().CreateTransformation();
        }

        //Prepare to draw point cloud.
        private void InitMesh()
        {
            //Get the width and height of the Depth image and calculate the number of all points
            int width = kinect.GetCalibration().DepthCameraCalibration.ResolutionWidth;
            int height = kinect.GetCalibration().DepthCameraCalibration.ResolutionHeight;
            numPoints = width * height;

            //Instantiate mesh
            mesh = new Mesh();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

            //Allocation of vertex and color storage space for the total number of pixels in the depth image
            vertices = new Vector3[numPoints];
            colors = new Color32[numPoints];
            indices = new int[numPoints];

            //Initialization of index list
            for (int i = 0; i < numPoints; i++)
            {
                indices[i] = i;
            }

            //Allocate a list of point coordinates, colors, and points to be drawn to mesh
            mesh.vertices = vertices;
            mesh.colors32 = colors;
            mesh.SetIndices(indices, MeshTopology.Points, 0);

            gameObject.GetComponent<MeshFilter>().mesh = mesh;

            if (webRTCData == null)
            {
                webRTCData = new byte[4 + (numPoints * NUM_FLOATS_PER_VECTOR * NUM_BYTES_PER_FLOAT) + (numPoints * NUM_BYTES_PER_COLOR)];
                verticesRaw = new float[numPoints * NUM_FLOATS_PER_VECTOR];
            }
        }

        private async Task KinectLoop()
        {
            while (true)
            {
                using (Capture capture = await Task.Run(() => kinect.GetCapture()).ConfigureAwait(true))
                {
                    //Getting color information
                    Image colorImage = transformation.ColorImageToDepthCamera(capture);
                    BGRA[] colorArray = colorImage.GetPixels<BGRA>().ToArray();

                    //Getting vertices of point cloud
                    Image xyzImage = transformation.DepthImageToPointCloud(capture.Depth);
                    Short3[] xyzArray = xyzImage.GetPixels<Short3>().ToArray();

                    for (int i = 0; i < numPoints; i++)
                    {
                        vertices[i].x = xyzArray[i].X * 0.001f;
                        vertices[i].y = -xyzArray[i].Y * 0.001f;
                        vertices[i].z = xyzArray[i].Z * 0.001f;

                        colors[i].b = colorArray[i].B;
                        colors[i].g = colorArray[i].G;
                        colors[i].r = colorArray[i].R;
                        colors[i].a = 255;
                    }

                    mesh.vertices = vertices;
                    mesh.colors32 = colors;
                    mesh.RecalculateBounds();

                    Debug.Log(_dataChannel);

                    if (_dataChannel != null && _dataChannel.State == Microsoft.MixedReality.WebRTC.DataChannel.ChannelState.Open)
                    {
                        int index = 0;

                        BitConverter.GetBytes(numPoints).CopyTo(webRTCData, index);
                        index += 4; // number of bytes per int

                        for (int i = 0; i < numPoints; i++)
                        {
                            verticesRaw[i * NUM_FLOATS_PER_VECTOR] = vertices[i].x;
                            verticesRaw[i * NUM_FLOATS_PER_VECTOR + 1] = vertices[i].y;
                            verticesRaw[i * NUM_FLOATS_PER_VECTOR + 2] = vertices[i].z;
                        }

                        Buffer.BlockCopy(verticesRaw, 0, webRTCData, index, numPoints * NUM_FLOATS_PER_VECTOR * NUM_BYTES_PER_FLOAT);
                        index += numPoints * NUM_FLOATS_PER_VECTOR * NUM_BYTES_PER_FLOAT;

                        for (int i = 0; i < numPoints; i++)
                        {
                            webRTCData[index] = colors[i].b;
                            index++; // Only one byte per field
                            webRTCData[index] = colors[i].g;
                            index++;
                            webRTCData[index] = colors[i].r;
                            index++;
                            webRTCData[index] = colors[i].a;
                            index++;
                        }

                        _dataChannel.SendMessage(webRTCData);
                    }
                }
            }
        }

        private void CopyBytes(byte[] source, byte[] destination, int index, int length)
        {
            for (int i = 0; i < length; i++)
            {
                destination[index + i] = source[i];
            }
        }

        //Stop Kinect as soon as this object disappear
        private void OnDestroy()
        {
            kinect.StopCameras();
        }
    }
}
