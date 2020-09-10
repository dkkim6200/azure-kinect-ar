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

        private Renderer renderer;

        private ConcurrentQueue<Action> _mainThreadWorkQueue;
        private DataChannel _dataChannel;

        private static readonly int NUM_BYTES_PER_FLOAT = 4;
        private static readonly int WEBRTC_MESSAGE_SIZE = 24;
        private static readonly float POINT_SCALE_FACTOR = 0.01f;

        private int _numPoints = -1;
        private int _width;
        private int _height;
        //Used to draw a set of points
        private Mesh mesh;
        //Array of coordinates for each point in PointCloud
        private Vector3[] _vertices;
        //List of indexes of points to be rendered
        private int[] _indices;

        private bool pointCloudInitialized;
        
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
        /// Internal timing counter
        /// </summary>
        private float lastUpdateTime = 0.0f;
        private float _minUpdateDelay;

        private VideoFrameQueue<I420AVideoFrameStorage> _i420aFrameQueue = null;
        private VideoFrameQueue<Argb32VideoFrameStorage> _argb32FrameQueue = null;

        private ProfilerMarker loadTextureDataMarker = new ProfilerMarker("LoadTextureData");
        private ProfilerMarker uploadTextureToGpuMarker = new ProfilerMarker("UploadTextureToGPU");


        private float cx, cy, fx, fy;


        //==================================================================================
        // MonoBehaviour
        //==================================================================================

        private void Start()
        {
            renderer = GetComponent<Renderer>();
            renderer.allowOcclusionWhenDynamic = false;

            _mainThreadWorkQueue = new ConcurrentQueue<Action>();

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

        private void InitPointCloud()
        {
            _numPoints = _width * _height;

            mesh = new Mesh();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

            //Allocation of vertex and color storage space for the total number of pixels in the depth image
            _vertices = new Vector3[(_numPoints + 8) * 3];
            Vector2[] uv = new Vector2[(_numPoints + 8) * 3];
            Vector2[] uv2 = new Vector2[(_numPoints + 8) * 3];
            Vector3[] normals = new Vector3[(_numPoints + 8) * 3];
            _indices = new int[(_numPoints + 8) * 3];

            // Setting the bounds to ensure frustrum culling doesn't cull the displaced vertices from the shader
            (float minDepth, float maxDepth) = GetDepthModeRange(DepthMode.NFOV_Unbinned);
            maxDepth *= 0.001f;

            _vertices[_numPoints * 3] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (-cy) / fy, maxDepth);
            _vertices[_numPoints * 3 + 1] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (-cy) / fy, maxDepth);
            _vertices[_numPoints * 3 + 2] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (-cy) / fy, maxDepth);

            _vertices[_numPoints * 3 + 3] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (-cy) / fy, maxDepth);
            _vertices[_numPoints * 3 + 4] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (-cy) / fy, maxDepth);
            _vertices[_numPoints * 3 + 5] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (-cy) / fy, maxDepth);

            _vertices[_numPoints * 3 + 6] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (_height-cy) / fy, maxDepth);
            _vertices[_numPoints * 3 + 7] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (_height-cy) / fy, maxDepth);
            _vertices[_numPoints * 3 + 8] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (_height-cy) / fy, maxDepth);

            _vertices[_numPoints * 3 + 9] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (_height-cy) / fy, maxDepth);
            _vertices[_numPoints * 3 + 10] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (_height-cy) / fy, maxDepth);
            _vertices[_numPoints * 3 + 11] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (_height-cy) / fy, maxDepth);

            _vertices[_numPoints * 3 + 12] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (-cy) / fy, 0);
            _vertices[_numPoints * 3 + 13] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (-cy) / fy, 0);
            _vertices[_numPoints * 3 + 14] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (-cy) / fy, 0);

            _vertices[_numPoints * 3 + 15] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (-cy) / fy, 0);
            _vertices[_numPoints * 3 + 16] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (-cy) / fy, 0);
            _vertices[_numPoints * 3 + 17] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (-cy) / fy, 0);

            _vertices[_numPoints * 3 + 18] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (_height-cy) / fy, 0);
            _vertices[_numPoints * 3 + 19] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (_height-cy) / fy, 0);
            _vertices[_numPoints * 3 + 20] = new Vector3(maxDepth * (_width-cx) / fx, maxDepth * (_height-cy) / fy, 0);

            _vertices[_numPoints * 3 + 21] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (_height-cy) / fy, 0);
            _vertices[_numPoints * 3 + 22] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (_height-cy) / fy, 0);
            _vertices[_numPoints * 3 + 23] = new Vector3(maxDepth * (-cx) / fx, maxDepth * (_height-cy) / fy, 0);

            for (int i = _numPoints * 3; i < _numPoints * 3 + 24; i++)
            {
                uv[i] = new Vector2(0, 0);
                _indices[i] = i;
                normals[i] = new Vector3(0, 1, 0);

                if (i % 3 == 0)
                {
                    uv2[i] = new Vector2(-POINT_SCALE_FACTOR, 0);
                }
                else if (i % 3 == 1)
                {
                    uv2[i] = new Vector2(POINT_SCALE_FACTOR, 0);
                }
                else if (i % 3 == 2)
                {
                    uv2[i] = new Vector2(0, 1.732f * POINT_SCALE_FACTOR);
                }
            }

            //Initialization of uv and normal 
            int index = 0;
            for (int y = 0; y < _height; y++)
            {
                for (int x = 0; x < _width; x++)
                {
                    for (int tri = 0; tri < 3; tri++)
                    {
                        uv[index] = new Vector2(((float)(x + 0.5f) / (float)(_width)), ((float)(y + 0.5f) / ((float)(_height))));
                        _indices[index] = index;
                        normals[index] = new Vector3(0, 1, 0);

                        if (index % 3 == 0)
                        {
                            uv2[index] = new Vector2(POINT_SCALE_FACTOR, 0);
                        }
                        else if (index % 3 == 1)
                        {
                            uv2[index] = new Vector2(-POINT_SCALE_FACTOR, 0);
                        }
                        else if (index % 3 == 2)
                        {
                            uv2[index] = new Vector2(0, 1.732f * POINT_SCALE_FACTOR);
                        }

                        index++;
                    }
                }
            }

            //Allocate a list of point coordinates, colors, and points to be drawn to mesh
            mesh.vertices = _vertices;
            mesh.uv = uv;
            mesh.uv2 = uv2;
            mesh.normals = normals;
            mesh.SetIndices(_indices, MeshTopology.Triangles, 0);
            mesh.RecalculateBounds();

            gameObject.GetComponent<MeshFilter>().mesh = mesh;

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
                _width = BitConverter.ToInt32(data, 0);
                _height = BitConverter.ToInt32(data, 4);
                cx = BitConverter.ToSingle(data, 8);
                cy = BitConverter.ToSingle(data, 12);
                fx = BitConverter.ToSingle(data, 16);
                fy = BitConverter.ToSingle(data, 20);

                if (!pointCloudInitialized)
                {
                    InitPointCloud();
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
            if (_i420aFrameQueue != null)
            {
                (uint minDepth, uint maxDepth) = GetDepthModeRange(DepthMode.NFOV_Unbinned);
                renderer.material.SetTexture("_YPlane", _textureY);
                renderer.material.SetTexture("_UPlane", _textureU);
                renderer.material.SetTexture("_VPlane", _textureV);
                renderer.material.SetFloat("_Width", _width);
                renderer.material.SetFloat("_Height", _height);
                renderer.material.SetFloat("_Cx", cx);
                renderer.material.SetFloat("_Cy", cy);
                renderer.material.SetFloat("_Fx", fx);
                renderer.material.SetFloat("_Fy", fy);
                renderer.material.SetFloat("_MinDepth", minDepth);
                renderer.material.SetFloat("_MaxDepth", maxDepth);
            }
            else if (_argb32FrameQueue != null)
            {
                renderer.material.SetTexture("_MainTex", _textureY);
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
                    (uint minDepth, uint maxDepth) = GetDepthModeRange(DepthMode.NFOV_Unbinned);

                    _textureY = new Texture2D(lumaWidth, lumaHeight, TextureFormat.R8, mipChain: false);
                    renderer.material.SetTexture("_YPlane", _textureY);
                    
                    renderer.material.SetFloat("_Width", _width);
                    renderer.material.SetFloat("_Height", _height);
                    renderer.material.SetFloat("_Cx", cx);
                    renderer.material.SetFloat("_Cy", cy);
                    renderer.material.SetFloat("_Fx", fx);
                    renderer.material.SetFloat("_Fy", fy);
                    renderer.material.SetFloat("_MinDepth", minDepth);
                    renderer.material.SetFloat("_MaxDepth", maxDepth);
                }
                int chromaWidth = lumaWidth / 2;
                int chromaHeight = lumaHeight / 2;
                if (_textureU == null || (_textureU.width != chromaWidth || _textureU.height != chromaHeight))
                {
                    _textureU = new Texture2D(chromaWidth, chromaHeight, TextureFormat.R8, mipChain: false);
                    renderer.material.SetTexture("_UPlane", _textureU);
                }
                if (_textureV == null || (_textureV.width != chromaWidth || _textureV.height != chromaHeight))
                {
                    _textureV = new Texture2D(chromaWidth, chromaHeight, TextureFormat.R8, mipChain: false);
                    renderer.material.SetTexture("_VPlane", _textureV);
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
                    renderer.material.SetTexture("_MainTex", _textureY);
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