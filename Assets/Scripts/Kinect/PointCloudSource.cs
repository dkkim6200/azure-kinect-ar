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

        private Vector3[] xyTable;
        private int _width;
        private int _height;
        
        private Image _colorImage;
        private Image _depthImage;
        private Image _testImage;

        private int[] _imageIntArray;
        private IntPtr _imageDataBuffer;

        private void Start()
        {
            //The method to initialize Kinect
            InitKinect();
            //Initialization for point cloud rendering
            InitMesh();
        }

        private void OnDestroy()
        {
            kinect.StopCameras();
            Marshal.FreeHGlobal(_imageDataBuffer);
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

        private static (uint, uint) GetDepthModeRange(DepthMode depthMode)
        {
            switch (depthMode)
            {
            case DepthMode.NFOV_2x2Binned:
                return (500, 6800);
            case DepthMode.NFOV_Unbinned:
                return (500, 4000);
            case DepthMode.WFOV_2x2Binned:
                return (250, 3000);
            case DepthMode.WFOV_Unbinned:
                return (250, 2500);
            case DepthMode.PassiveIR:
            default:
                return (0, 0);
            }
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
            _width = kinect.GetCalibration().DepthCameraCalibration.ResolutionWidth;
            _height = kinect.GetCalibration().DepthCameraCalibration.ResolutionHeight;
            numPoints = _width * _height;

            xyTable = new Vector3[numPoints];

            System.Numerics.Vector2 point = System.Numerics.Vector2.Zero;
            for (int y = 0, index = 0; y < _height; y++)
            {
                point.Y = y;

                for (int x = 0; x < _width; x++, index++)
                {
                    point.X = x;

                    System.Numerics.Vector3? transformedPoint = kinect.GetCalibration().TransformTo3D(point, 1f, CalibrationDeviceType.Depth, CalibrationDeviceType.Depth);

                    if (transformedPoint != null)
                    {
                        xyTable[index].x = transformedPoint.Value.X;
                        xyTable[index].y = transformedPoint.Value.Y;
                        xyTable[index].z = transformedPoint.Value.Z;
                    }
                    else
                    {
                        xyTable[index].x = 0f;
                        xyTable[index].y = 0f;
                        xyTable[index].z = 0f;
                    }
                }
            }

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
            
            _imageIntArray = new int[_width * _height * 2];
            _imageDataBuffer = Marshal.AllocHGlobal(_width * _height * 4 * 2);
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
                    Image xyzImage = transformation.DepthImageToPointCloud(capture.Depth);
                    Short3[] xyzArray = xyzImage.GetPixels<Short3>().ToArray();

                    for (int i = 0; i < numPoints; i++)
                    {
                        vertices[i].x = xyzArray[i].X * 0.001f;
                        vertices[i].y = xyzArray[i].Y * -0.001f;
                        vertices[i].z = xyzArray[i].Z * 0.001f;

                        colors[i].b = colorArray[i].B;
                        colors[i].g = colorArray[i].G;
                        colors[i].r = colorArray[i].R;
                        colors[i].a = 255;
                    }

                    mesh.vertices = vertices;
                    mesh.colors32 = colors;
                    mesh.RecalculateBounds();

                    UInt16[] depthArray = capture.Depth.GetPixels<UInt16>().ToArray();

                    for (int i = 0; i < _height; i++)
                    {
                        for (int j = 0; j < _width; j++)
                        {
                            _imageIntArray[i * _width * 2 + j] = colorArray[i * _width + j].Value;
                            
                            int depthValue = (int) depthArray[i * _width + j];
                            _imageIntArray[i * _width * 2 + j + _width] = depthValue;
                        }
                    }
                    
                    Marshal.Copy(_imageIntArray, 0, _imageDataBuffer, _imageIntArray.Length);
                }
            }
        }

        protected override void OnFrameRequested(in FrameRequest request)
        {
            if (_colorImage == null)
                return;

            Argb32VideoFrame frame = new Argb32VideoFrame();
            frame.data = _imageDataBuffer;
            frame.width = (uint) _width * 2;
            frame.height = (uint) _height;
            frame.stride = _colorImage.StrideBytes * 2;
            
            request.CompleteRequest(frame);
        }
    }
}
