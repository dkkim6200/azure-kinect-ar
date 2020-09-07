using Microsoft.Azure.Kinect.Sensor;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.MixedReality.WebRTC;
using System;
using Unity.Profiling;

namespace DKDevelopment.AzureKinect.Client
{
    public class PointCloudClient : MonoBehaviour
    {
        [Tooltip("WebRTC Unity Peer Connection object reference")]
        public Microsoft.MixedReality.WebRTC.Unity.PeerConnection _peerConnection;
        
        [Tooltip("Max playback framerate, in frames per second")]
        [Range(0.001f, 120f)]
        public float MaxFramerate = 30f;

        public Shader YUV2RGBConverter;

        private ConcurrentQueue<Action> _mainThreadWorkQueue;
        private DataChannel _dataChannel;

        private static readonly int NUM_BYTES_PER_FLOAT = 4;
        private static readonly int WEBRTC_MESSAGE_SIZE = 131264;
        private static readonly int MAX_FRAGMENTS = 50;
        private static readonly byte MESSAGE_ENDING_INDICATOR = 0x42;

        private int _numPoints = -1;
        private int _width;
        private int _height;
        //Used to draw a set of points
        private Mesh mesh;
        //Array of coordinates for each point in PointCloud
        private Vector3[] _vertices;
        //Array of colors corresponding to each point in PointCloud
        private Color32[] _colors;
        //List of indexes of points to be rendered
        private int[] _indices;
        private Vector3[] _xyTable;
        private Color32[] _pixels;

        private bool pointCloudInitialized;

        private byte[] unfragData;
        private int unfragDataIndex;
        
        //==================================================================================
        // Video Frame Renderer
        //==================================================================================

        // Source that this renderer is currently subscribed to.
        private IVideoSource _source;

        /// <summary>
        /// Internal reference to the attached texture
        /// </summary>
        private Texture2D _textureY = null; // also used for ARGB32
        private Texture2D _textureU = null;
        private Texture2D _textureV = null;
        
        /// <summary>
        /// Output Texture2D of the converted ARGB image
        /// </summary>
        private Texture2D _textureOutput = null;
        private RenderTexture renderTex;

        /// <summary>
        /// Internal timing counter
        /// </summary>
        private float lastUpdateTime = 0.0f;

        /// <summary>
        /// Material with shader that converts YUV to RGB texture
        /// </summary>
        private Material videoMaterial;
        private float _minUpdateDelay;

        private VideoFrameQueue<I420AVideoFrameStorage> _i420aFrameQueue = null;
        private VideoFrameQueue<Argb32VideoFrameStorage> _argb32FrameQueue = null;

        private ProfilerMarker loadTextureDataMarker = new ProfilerMarker("LoadTextureData");
        private ProfilerMarker uploadTextureToGpuMarker = new ProfilerMarker("UploadTextureToGPU");



        //==================================================================================
        // MonoBehaviour
        //==================================================================================

        private void Start()
        {
            _mainThreadWorkQueue = new ConcurrentQueue<Action>();
            unfragData = new byte[WEBRTC_MESSAGE_SIZE * MAX_FRAGMENTS];
            unfragDataIndex = 0;

            _peerConnection.OnInitialized.AddListener(() => {
                _peerConnection.Peer.AddDataChannelAsync("xy-table", true, true);
                _peerConnection.Peer.DataChannelAdded += OnDataChannelAdded;
            });

            // Leave 3ms of margin, otherwise it misses 1 frame and drops to ~20 FPS
            // when Unity is running at 60 FPS.
            _minUpdateDelay = Mathf.Max(0f, 1f / Mathf.Max(0.001f, MaxFramerate) - 0.003f);
        }

        private void Update()
        {
            // Execute any pending work enqueued by background tasks
            while (_mainThreadWorkQueue.TryDequeue(out Action workload))
            {
                workload();
            }

            UpdateTextureRenderer();
            UpdatePointCloud();
        }

        private void UpdatePointCloud()
        {
            if (!pointCloudInitialized || _textureOutput == null)
                return;

            for (int i = 0; i < _height; i++)
            {
                for (int j = 0; j < _width; j++)
                {
                    _colors[i * _width + j] = _pixels[i * _width * 2 + j];

                    (float hue, float value) = RGBToHSV(_pixels[i * _width * 2 + j + _width]);
                    (uint minDepth, uint maxDepth) = GetDepthModeRange(DepthMode.NFOV_Unbinned);

                    float depthValue = (hue/360.0f * (float) (maxDepth - minDepth) + minDepth);
                    _vertices[i * _width + j].x = _xyTable[i * _width + j].x * depthValue * 0.001f;
                    _vertices[i * _width + j].y = _xyTable[i * _width + j].y * depthValue * 0.001f;
                    _vertices[i * _width + j].z = depthValue * 0.001f;
                }
            }

            mesh.vertices = _vertices;
            mesh.colors32 = _colors;
            mesh.RecalculateBounds();
        }

        //==================================================================================
        // PointCloud
        //==================================================================================

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

        public static (float hue, float value) RGBToHSV(Color32 rgb)
        {
            float delta, min;
            float h = 0, s, v;

            min = Math.Min(Math.Min(rgb.r, rgb.g), rgb.b);
            v = Math.Max(Math.Max(rgb.r, rgb.g), rgb.b);
            delta = v - min;

            if (v == 0.0f)
                s = 0;
            else
                s = delta / v;

            if (s == 0)
                h = 0.0f;

            else
            {
                if (rgb.r == v)
                    h = (rgb.g - rgb.b) / delta;
                else if (rgb.g == v)
                    h = 2 + (rgb.b - rgb.r) / delta;
                else if (rgb.b == v)
                    h = 4 + (rgb.r - rgb.g) / delta;

                h *= 60;

                if (h < 0.0f)
                    h = h + 360;
            }

            return (h, (v / 255));
        }

        private void InitPointCloud()
        {
            _numPoints = _width * _height;

            mesh = new Mesh();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

            _xyTable = new Vector3[_numPoints];

            //Allocation of vertex and color storage space for the total number of pixels in the depth image
            _vertices = new Vector3[_numPoints];
            _colors = new Color32[_numPoints];
            _indices = new int[_numPoints];

            //Initialization of index list
            for (int i = 0; i < _numPoints; i++)
            {
                _indices[i] = i;
            }

            //Allocate a list of point coordinates, colors, and points to be drawn to mesh
            mesh.vertices = _vertices;
            mesh.colors32 = _colors;
            mesh.SetIndices(_indices, MeshTopology.Points, 0);

            gameObject.GetComponent<MeshFilter>().mesh = mesh;

            // renderTex.width = _width * 2;
            // renderTex.height = _height;
            renderTex = new RenderTexture(_width * 2, _height, 16, RenderTextureFormat.ARGB32);
            renderTex.Create();

            _pixels = new Color32[_numPoints * 2];

            pointCloudInitialized = true;
        }

        //==================================================================================
        // WebRTC
        //==================================================================================

        public void StartPointCloud()
        {
            _peerConnection.StartConnection();
        }

        public void OnDataChannelAdded(DataChannel channel)
        {
            Debug.Log("OnDataChannelAdded");
            _mainThreadWorkQueue.Enqueue(() =>
            {
                _dataChannel = channel;
                _dataChannel.MessageReceived += OnWebRTCMessageReceived;
                Debug.Log("OnDataChannelAdded Main Thread");
            });
        }

        private void OnWebRTCMessageReceived(byte[] data)
        {
            Debug.Log("Received data of length " + data.Length);

            _mainThreadWorkQueue.Enqueue(() => {
                // Copy if the message is of length WEBRTC_MESSAGE_SIZE
                if (data.Length == WEBRTC_MESSAGE_SIZE)
                {
                    Buffer.BlockCopy(data, 0, unfragData, unfragDataIndex, WEBRTC_MESSAGE_SIZE);
                    unfragDataIndex += WEBRTC_MESSAGE_SIZE;
                    return;
                }

                // Check if the message is ending indicator
                if (data.Length != 1 || data[0] != MESSAGE_ENDING_INDICATOR)
                    return;

                // After we encountered an ending indicator, we initialize the xyTable.

                _width = BitConverter.ToInt32(unfragData, 0);
                _height = BitConverter.ToInt32(unfragData, 4);

                if (!pointCloudInitialized)
                {
                    InitPointCloud();
                }

                int index = 8;

                for (int i = 0; i < _numPoints; i++)
                {
                    _xyTable[i].x = BitConverter.ToSingle(unfragData, index);
                    index += NUM_BYTES_PER_FLOAT;
                    _xyTable[i].y = BitConverter.ToSingle(unfragData, index);
                    index += NUM_BYTES_PER_FLOAT;
                    _xyTable[i].z = BitConverter.ToSingle(unfragData, index);
                    index += NUM_BYTES_PER_FLOAT;
                }
            });
        }

        //==================================================================================
        // Video Frame Renderer
        //==================================================================================

        /// <summary>
        /// Start rendering the passed source.
        /// </summary>
        /// <remarks>
        /// Can be used to handle <see cref="VideoTrackSource.VideoStreamStarted"/> or <see cref="VideoReceiver.VideoStreamStarted"/>.
        /// </remarks>
        public void StartRendering(IVideoSource source)
        {
            bool isRemote = (source is RemoteVideoTrack);
            int frameQueueSize = (isRemote ? 5 : 3);

            Debug.Log("DK LOGG: " + source.FrameEncoding);
            switch (source.FrameEncoding)
            {
                case VideoEncoding.I420A:
                    _i420aFrameQueue = new VideoFrameQueue<I420AVideoFrameStorage>(frameQueueSize);
                    source.I420AVideoFrameReady += I420AVideoFrameReady;
                    break;

                case VideoEncoding.Argb32:
                    _argb32FrameQueue = new VideoFrameQueue<Argb32VideoFrameStorage>(frameQueueSize);
                    source.Argb32VideoFrameReady += Argb32VideoFrameReady;
                    break;
            }

            CreateEmptyVideoTextures();
        }

        /// <summary>
        /// Stop rendering the passed source. Must be called with the same source passed to <see cref="StartRendering(IVideoSource)"/>
        /// </summary>
        /// <remarks>
        /// Can be used to handle <see cref="VideoTrackSource.VideoStreamStopped"/> or <see cref="VideoReceiver.VideoStreamStopped"/>.
        /// </remarks>
        public void StopRendering(IVideoSource _)
        {
            // Clear the video display to not confuse the user who could otherwise
            // think that the video is still playing but is lagging/frozen.
            CreateEmptyVideoTextures();
        }

        protected void I420AVideoFrameReady(I420AVideoFrame frame)
        {
            // This callback is generally from a non-UI thread, but Unity object access is only allowed
            // on the main UI thread, so defer to that point.
            _i420aFrameQueue.Enqueue(frame);
        }

        protected void Argb32VideoFrameReady(Argb32VideoFrame frame)
        {
            // This callback is generally from a non-UI thread, but Unity object access is only allowed
            // on the main UI thread, so defer to that point.
            _argb32FrameQueue.Enqueue(frame);
        }

        private void CreateEmptyVideoTextures()
        {
            // Create a default checkboard texture which visually indicates
            // that no data is available. This is useful for debugging and
            // for the user to know about the state of the video.
            _textureY = new Texture2D(2, 2);
            _textureY.SetPixel(0, 0, Color.blue);
            _textureY.SetPixel(1, 1, Color.blue);
            _textureY.Apply();
            _textureU = new Texture2D(2, 2);
            _textureU.SetPixel(0, 0, Color.blue);
            _textureU.SetPixel(1, 1, Color.blue);
            _textureU.Apply();
            _textureV = new Texture2D(2, 2);
            _textureV.SetPixel(0, 0, Color.blue);
            _textureV.SetPixel(1, 1, Color.blue);
            _textureV.Apply();

            // Assign that texture to the video player's Renderer component
            // videoMaterial = GetComponent<Renderer>().material;
            videoMaterial = new Material(YUV2RGBConverter);
            if (_i420aFrameQueue != null)
            {
                videoMaterial.SetTexture("_YPlane", _textureY);
                videoMaterial.SetTexture("_UPlane", _textureU);
                videoMaterial.SetTexture("_VPlane", _textureV);
            }
            else if (_argb32FrameQueue != null)
            {
                videoMaterial.SetTexture("_MainTex", _textureY);
            }
        }

        private void UpdateTextureRenderer()
        {
            if ((_i420aFrameQueue != null) || (_argb32FrameQueue != null))
            {
#if UNITY_EDITOR
                // Inside the Editor, constantly update _minUpdateDelay to
                // react to user changes to MaxFramerate.

                // Leave 3ms of margin, otherwise it misses 1 frame and drops to ~20 FPS
                // when Unity is running at 60 FPS.
                _minUpdateDelay = Mathf.Max(0f, 1f / Mathf.Max(0.001f, MaxFramerate) - 0.003f);
#endif
                // FIXME - This will overflow/underflow the queue if not set at the same rate
                // as the one at which frames are enqueued!
                var curTime = Time.time;
                if (curTime - lastUpdateTime >= _minUpdateDelay)
                {
                    if (_i420aFrameQueue != null)
                    {
                        TryProcessI420AFrame();
                    }
                    else if (_argb32FrameQueue != null)
                    {
                        TryProcessArgb32Frame();
                    }
                    lastUpdateTime = curTime;
                }
            }
        }

        /// <summary>
        /// Internal helper that attempts to process frame data in the frame queue
        /// </summary>
        private void TryProcessI420AFrame()
        {
            if (_i420aFrameQueue.TryDequeue(out I420AVideoFrameStorage frame))
            {
                int lumaWidth = (int)frame.Width;
                int lumaHeight = (int)frame.Height;
                if (_textureY == null || (_textureY.width != lumaWidth || _textureY.height != lumaHeight))
                {
                    _textureY = new Texture2D(lumaWidth, lumaHeight, TextureFormat.R8, mipChain: false);
                    videoMaterial.SetTexture("_YPlane", _textureY);
                }
                int chromaWidth = lumaWidth / 2;
                int chromaHeight = lumaHeight / 2;
                if (_textureU == null || (_textureU.width != chromaWidth || _textureU.height != chromaHeight))
                {
                    _textureU = new Texture2D(chromaWidth, chromaHeight, TextureFormat.R8, mipChain: false);
                    videoMaterial.SetTexture("_UPlane", _textureU);
                }
                if (_textureV == null || (_textureV.width != chromaWidth || _textureV.height != chromaHeight))
                {
                    _textureV = new Texture2D(chromaWidth, chromaHeight, TextureFormat.R8, mipChain: false);
                    videoMaterial.SetTexture("_VPlane", _textureV);
                }

                // Copy data from C# buffer into system memory managed by Unity.
                // Note: This only "looks right" in Unity because we apply the
                // "YUVFeedShader(Unlit)" to the texture (converting YUV planar to RGB).
                // Note: Texture2D.LoadRawTextureData() expects some bottom-up texture data but
                // the WebRTC video frame is top-down, so the image is uploaded vertically flipped,
                // and needs to be flipped by in the shader used to sample it. See #388.
                using (var profileScope = loadTextureDataMarker.Auto())
                {
                    unsafe
                    {
                        fixed (void* buffer = frame.Buffer)
                        {
                            var src = new IntPtr(buffer);
                            int lumaSize = lumaWidth * lumaHeight;
                            _textureY.LoadRawTextureData(src, lumaSize);
                            src += lumaSize;
                            int chromaSize = chromaWidth * chromaHeight;
                            _textureU.LoadRawTextureData(src, chromaSize);
                            src += chromaSize;
                            _textureV.LoadRawTextureData(src, chromaSize);
                        }
                    }
                }

                // Upload from system memory to GPU
                using (var profileScope = uploadTextureToGpuMarker.Auto())
                {
                    _textureY.Apply();
                    _textureU.Apply();
                    _textureV.Apply();
                }

                // Color32[] textureYColors = _textureY.GetPixels32();
                // Color32[] textureUColors = _textureU.GetPixels32();
                // Color32[] textureVColors = _textureV.GetPixels32();

                // Debug.LogError(_pixels.Length + " " + textureYColors.Length + " " + textureVColors.Length + " ");

                // TextureU and TextureV are subsampled (https://en.wikipedia.org/wiki/Chroma_subsampling)
                // with 4:1 ratio, so we need to take care of that
                // for (int i = 0; i < _textureY.width * _textureY.height; i++)
                // {
                //     int row = (i / _textureY.width) / 4;
                //     int col = (i % _textureY.width) / 4;
                //     _pixels[i].r = (byte) (textureYColors[i].r + 1.370705f * (((int) textureVColors[row * _textureV.width + col].r) - 128));
                //     _pixels[i].g = (byte) (textureYColors[i].r - 0.698001f * (((int) textureVColors[row * _textureV.width + col].r) - 128) - 0.337633 * (((int) textureUColors[row * _textureU.width + col].r) - 128));
                //     _pixels[i].b = (byte) (textureYColors[i].r + 1.370705f * (((int) textureUColors[row * _textureU.width + col].r) - 128));
                // }

                if (pointCloudInitialized)
                {
                    Graphics.Blit(_textureY, renderTex, videoMaterial);
                    if (_textureOutput == null || (_textureOutput.width != _width || _textureOutput.height != _height))
                    {
                        _textureOutput = new Texture2D(_width * 2, _height, TextureFormat.ARGB32, mipChain: false);
                    }

                    RenderTexture.active = renderTex;
                    _textureOutput.ReadPixels(new Rect(0, 0, renderTex.width, renderTex.height), 0, 0);
                    _textureOutput.Apply();
                    // Graphics.CopyTexture(renderTex, _textureOutput);

                    _pixels = _textureOutput.GetPixels32();
                }

                // Recycle the video frame packet for a later frame
                _i420aFrameQueue.RecycleStorage(frame);
            }
        }

        /// <summary>
        /// Internal helper that attempts to process frame data in the frame queue
        /// </summary>
        private void TryProcessArgb32Frame()
        {
            if (_argb32FrameQueue.TryDequeue(out Argb32VideoFrameStorage frame))
            {
                int width = (int)frame.Width;
                int height = (int)frame.Height;
                if (_textureY == null || (_textureY.width != width || _textureY.height != height))
                {
                    _textureY = new Texture2D(width, height, TextureFormat.BGRA32, mipChain: false);
                    videoMaterial.SetTexture("_MainTex", _textureY);
                }

                // Copy data from C# buffer into system memory managed by Unity.
                // Note: Texture2D.LoadRawTextureData() expects some bottom-up texture data but
                // the WebRTC video frame is top-down, so the image is uploaded vertically flipped,
                // and needs to be flipped by in the shader used to sample it. See #388.
                using (var profileScope = loadTextureDataMarker.Auto())
                {
                    unsafe
                    {
                        fixed (void* buffer = frame.Buffer)
                        {
                            var src = new IntPtr(buffer);
                            int size = width * height * 4;
                            _textureY.LoadRawTextureData(src, size);
                        }
                    }
                }

                // Upload from system memory to GPU
                using (var profileScope = uploadTextureToGpuMarker.Auto())
                {
                    _textureY.Apply();
                }

                // Recycle the video frame packet for a later frame
                _argb32FrameQueue.RecycleStorage(frame);
            }
        }

    }
}