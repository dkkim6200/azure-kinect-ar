using Microsoft.Azure.Kinect.Sensor;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.MixedReality.WebRTC.Unity;
using Microsoft.MixedReality.WebRTC;
using System;
using System.Threading;
using System.Runtime.InteropServices;

namespace DKDevelopment.AzureKinect.Server
{
    public class PointCloudSource : CustomVideoSource<Argb32VideoFrameStorage>
    {
        private static readonly int NUM_FLOATS_PER_VECTOR = 3;
        private static readonly int NUM_BYTES_PER_COLOR = 4;
        private static readonly int NUM_BYTES_PER_FLOAT = 4;
        private static readonly int WEBRTC_MESSAGE_SIZE = 131264; // Real limit is 262528

        public Microsoft.MixedReality.WebRTC.Unity.PeerConnection _peerConnection;
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
        
        private Image _colorImage;
        private Image _depthImage;
        private Image _testImage;

        private int[] _colorImageIntArray;
        private IntPtr _colorImageDataBuffer;
        private int[] _depthImageIntArray;
        private IntPtr _depthImageDataBuffer;

        private int[] _testImageIntArray;
        private IntPtr _testImageDataBuffer;

        private void Start()
        {
            //The method to initialize Kinect
            InitKinect();
            //Initialization for point cloud rendering
            InitMesh();
        }

        public void StartKinect()
        {
            _peerConnection.Peer.DataChannelAdded += OnDataChannelAdded;

            //Loop to get data from Kinect and rendering
            Task t = KinectLoop();
        }

        public void OnDataChannelAdded(Microsoft.MixedReality.WebRTC.DataChannel channel)
        {
            _dataChannel = channel;
            Debug.Log("OnDataChannelAdded");
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
            
            _colorImageIntArray = new int[width * height];
            _colorImageDataBuffer = Marshal.AllocHGlobal(width * height * 4);
            _depthImageIntArray = new int[width * height];
            _depthImageDataBuffer = Marshal.AllocHGlobal(width * height * 4);
            _depthImageIntArray = new int[kinect.GetCalibration().ColorCameraCalibration.ResolutionWidth * kinect.GetCalibration().ColorCameraCalibration.ResolutionHeight];
            _depthImageDataBuffer = Marshal.AllocHGlobal(kinect.GetCalibration().ColorCameraCalibration.ResolutionWidth * kinect.GetCalibration().ColorCameraCalibration.ResolutionHeight * 4);
        }

        private async Task KinectLoop()
        {
            while (true)
            {
                using (Capture capture = await Task.Run(() => kinect.GetCapture()).ConfigureAwait(true))
                {
                    //Getting color information
                    _colorImage = transformation.ColorImageToDepthCamera(capture);
                    BGRA[] colorArray = _colorImage.GetPixels<BGRA>().ToArray();

                    //Getting vertices of point cloud
                    _depthImage = transformation.DepthImageToPointCloud(capture.Depth);
                    Short3[] xyzArray = _depthImage.GetPixels<Short3>().ToArray();

                    _testImage = transformation.DepthImageToColorCamera(capture);

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
                }
            }
        }

        protected override void OnFrameRequested(in FrameRequest request)
        {
            if (_colorImage == null || _depthImage == null || _testImage == null)
                return;

            for (int i = 0; i < _testImage.HeightPixels; i++)
            {
                for (int j = 0; j < _testImage.WidthPixels; j++)
                {
                    // _colorImageIntArray[i * _colorImage.WidthPixels + j] = _colorImage.GetPixel<BGRA>(i, j).Value;
                    
                    // int xComp = Convert.ToSByte((float) _depthImage.GetPixel<Short3>(i, j).X * -256f / _colorImage.WidthPixels);
                    // int yComp = Convert.ToSByte((float) _depthImage.GetPixel<Short3>(i, j).Y * -256f / _colorImage.HeightPixels);
                    // int zComp = Convert.ToSByte((float) _depthImage.GetPixel<Short3>(i, j).Z * 256f / 6000f);
                    // int depthValue = (xComp << 4) | (yComp << 2) | (zComp); // ARGB = 0XYZ;
                    // _depthImageIntArray[i * _colorImage.WidthPixels + j] = depthValue;

                    _testImageIntArray[i * _testImage.WidthPixels + j] = _testImage.GetPixel<BGRA>(i, j).Value;
                }
            }
            
            // Marshal.Copy(_colorImageIntArray, 0, _colorImageDataBuffer, _colorImageIntArray.Length);
            // Marshal.Copy(_depthImageIntArray, 0, _depthImageDataBuffer, _depthImageIntArray.Length);
            Marshal.Copy(_testImageIntArray, 0, _testImageDataBuffer, _testImageIntArray.Length);

            // Argb32VideoFrame colorFrame = new Argb32VideoFrame();
            // colorFrame.data = _colorImageDataBuffer;
            // colorFrame.width = (uint) _colorImage.WidthPixels;
            // colorFrame.height = (uint) _colorImage.HeightPixels;
            // colorFrame.stride = _colorImage.StrideBytes;

            // Argb32VideoFrame depthFrame = new Argb32VideoFrame();
            // depthFrame.data = _depthImageDataBuffer;
            // depthFrame.width = (uint) _depthImage.WidthPixels;
            // depthFrame.width = (uint) _depthImage.HeightPixels;
            // depthFrame.stride = _depthImage.StrideBytes;

            Argb32VideoFrame testFrame = new Argb32VideoFrame();
            testFrame.data = _testImageDataBuffer;
            testFrame.width = (uint) _testImage.WidthPixels;
            testFrame.height = (uint) _testImage.HeightPixels;
            testFrame.stride = _testImage.StrideBytes;
            
            request.CompleteRequest(testFrame);
        }

        //Stop Kinect as soon as this object disappear
        private void OnDestroy()
        {
            kinect.StopCameras();
        }
    }
}
