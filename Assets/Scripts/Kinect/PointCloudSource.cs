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
        private static readonly int NUM_BYTES_PER_FLOAT = 4;
        private static readonly int WEBRTC_MESSAGE_SIZE = 24;

        public Microsoft.MixedReality.WebRTC.Unity.PeerConnection _peerConnection;
        private Microsoft.MixedReality.WebRTC.DataChannel _dataChannel;
        private byte[] _webRTCData;
        private bool _messageTransmissionFinished;

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

        private int _width;
        private int _height;
        
        private Image _colorImage;
        private Image _depthImage;
        private Image _testImage;

        private int[] _imageIntArray;
        private IntPtr _imageDataBuffer;

        private float cx, cy, fx, fy;

        private void Start()
        {
            //The method to initialize Kinect
            InitKinect();
            //Initialization for point cloud rendering
            InitMesh();
            InitWebRTC();
        }

        private void OnDestroy()
        {
            kinect.StopCameras();
            Marshal.FreeHGlobal(_imageDataBuffer);
        }

        public void StartKinect()
        {
            //Loop to get data from Kinect and rendering
            Task t = KinectLoop();
        }

        public void OnWebRTCInitialized()
        {
            _peerConnection.Peer.DataChannelAdded += OnDataChannelAdded;
        }

        public void OnDataChannelAdded(DataChannel channel)
        {
            _dataChannel = channel;
            _dataChannel.StateChanged += (() => {
                if (_dataChannel.State == DataChannel.ChannelState.Open)
                {
                    SendKinectInitialData();
                }
            });
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

            Intrinsics depthIntrinsics = kinect.GetCalibration().DepthCameraCalibration.Intrinsics;
            cx = depthIntrinsics.Parameters[0] * 0.9f;
            cy = depthIntrinsics.Parameters[1] * 0.9f;
            fx = depthIntrinsics.Parameters[2] * 0.9f;
            fy = depthIntrinsics.Parameters[3] * 0.9f;
        }

        //Prepare to draw point cloud.
        private void InitMesh()
        {
            //Get the width and height of the Depth image and calculate the number of all points
            _width = kinect.GetCalibration().DepthCameraCalibration.ResolutionWidth;
            _height = kinect.GetCalibration().DepthCameraCalibration.ResolutionHeight;
            numPoints = _width * _height;

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

        private void InitWebRTC()
        {
            _webRTCData = new byte[WEBRTC_MESSAGE_SIZE];

            int index = 0;

            BitConverter.GetBytes(_width).CopyTo(_webRTCData, index);
            index += NUM_BYTES_PER_FLOAT;
            BitConverter.GetBytes(_height).CopyTo(_webRTCData, index);
            index += NUM_BYTES_PER_FLOAT;
            BitConverter.GetBytes(cx).CopyTo(_webRTCData, index);
            index += NUM_BYTES_PER_FLOAT;
            BitConverter.GetBytes(cy).CopyTo(_webRTCData, index);
            index += NUM_BYTES_PER_FLOAT;
            BitConverter.GetBytes(fx).CopyTo(_webRTCData, index);
            index += NUM_BYTES_PER_FLOAT;
            BitConverter.GetBytes(fy).CopyTo(_webRTCData, index);
            index += NUM_BYTES_PER_FLOAT;
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

                    UInt16[] depthArray = capture.Depth.GetPixels<UInt16>().ToArray();

                    for (int i = 0; i < numPoints; i++)
                    {
                        int row = i / _width;
                        int col = i % _width;

                        vertices[i].x = depthArray[i] * (col - cx) / fx * 0.001f;
                        vertices[i].y = depthArray[i] * (row - cy) / fy * -0.001f;
                        vertices[i].z = depthArray[i] * 0.001f;

                        colors[i].b = colorArray[i].B;
                        colors[i].g = colorArray[i].G;
                        colors[i].r = colorArray[i].R;
                        colors[i].a = 255;
                    }

                    mesh.vertices = vertices;
                    mesh.colors32 = colors;
                    mesh.RecalculateBounds();

                    for (int i = 0; i < _height; i++)
                    {
                        for (int j = 0; j < _width; j++)
                        {
                            _imageIntArray[i * _width * 2 + j] = colorArray[i * _width + j].Value;
                            
                            int depthValueInMillimeter = (int) depthArray[i * _width + j];
                            (uint minDepth, uint maxDepth) = GetDepthModeRange(DepthMode.NFOV_Unbinned);

                            float hueAngle = (float) (depthValueInMillimeter - minDepth) / (float) (maxDepth - minDepth) * 360f;
                            BGRA outputColor;
                            if (depthValueInMillimeter != minDepth)
                            {
                                outputColor = HSVToRGB(hueAngle, 1.0f, 1.0f);
                            }
                            else 
                            {
                                outputColor = new BGRA(0, 0, 0, 0);
                            }
                            _imageIntArray[i * _width * 2 + j + _width] = outputColor.Value;
                        }
                    }
                    
                    Marshal.Copy(_imageIntArray, 0, _imageDataBuffer, _imageIntArray.Length);
                }
            }
        }

        /// <summary>
        /// From https://www.programmingalgorithms.com/algorithm/hsv-to-rgb/
        /// </summary>
        /// <param name="h">Hue</param>
        /// <param name="s">Saturation</param>
        /// <param name="v">Value</param>
        /// <returns>An RGBA equivalent color with alpha set to 255.</returns>
        public static BGRA HSVToRGB(float h, float s, float v)
        {
            float r = 0, g = 0, b = 0;

            if (s == 0)
            {
                r = v;
                g = v;
                b = v;
            }
            else
            {
                int i;
                float f, p, q, t;

                if (h == 360)
                    h = 0;
                else
                    h = h / 60;

                i = (int)Math.Truncate(h);
                f = h - i;

                p = v * (1.0f - s);
                q = v * (1.0f - (s * f));
                t = v * (1.0f - (s * (1.0f - f)));

                switch (i)
                {
                    case 0:
                        r = v;
                        g = t;
                        b = p;
                        break;

                    case 1:
                        r = q;
                        g = v;
                        b = p;
                        break;

                    case 2:
                        r = p;
                        g = v;
                        b = t;
                        break;

                    case 3:
                        r = p;
                        g = q;
                        b = v;
                        break;

                    case 4:
                        r = t;
                        g = p;
                        b = v;
                        break;

                    default:
                        r = v;
                        g = p;
                        b = q;
                        break;
                }

            }

            return new BGRA((byte)(b * 255), (byte)(g * 255), (byte)(r * 255), 255);
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

        private void SendKinectInitialData()
        {
            _dataChannel.SendMessage(_webRTCData);
            _messageTransmissionFinished = true;
        }
    }
}
