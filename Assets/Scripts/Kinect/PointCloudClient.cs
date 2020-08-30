using Microsoft.Azure.Kinect.Sensor;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.MixedReality.WebRTC.Unity;
using System;

namespace DKDevelopment.AzureKinect.Client
{
    public class PointCloudClient : MonoBehaviour
    {
        private ConcurrentQueue<Action> _mainThreadWorkQueue;
        public PeerConnection _peerConnection;
        private Microsoft.MixedReality.WebRTC.DataChannel _dataChannel;

        // private static readonly int NUM_FLOATS_PER_VECTOR = 3;
        // private static readonly int NUM_BYTES_PER_COLOR = 4;
        private static readonly int NUM_BYTES_PER_FLOAT = 4;
        private static readonly int WEBRTC_MESSAGE_SIZE = 131264;

        private int numPoints = -1;
        //Used to draw a set of points
        private Mesh mesh;
        //Array of coordinates for each point in PointCloud
        private Vector3[] vertices;
        //Array of colors corresponding to each point in PointCloud
        private Color32[] colors;
        //List of indexes of points to be rendered
        private int[] indices;

        private bool pointCloudInitialized;

        private byte[] unfragData;
        private int unfragDataIndex;

        private void Start()
        {
            _mainThreadWorkQueue = new ConcurrentQueue<Action>();
            unfragData = new byte[WEBRTC_MESSAGE_SIZE * 47];
            unfragDataIndex = 0;

            _peerConnection.OnInitialized.AddListener(() => {
                _peerConnection.Peer.AddDataChannelAsync("chat", true, true);
                _peerConnection.Peer.DataChannelAdded += OnDataChannelAdded;
            });
        }

        public void StartPointCloud()
        {
            _peerConnection.StartConnection();
        }

        public void OnDataChannelAdded(Microsoft.MixedReality.WebRTC.DataChannel channel)
        {
            Debug.Log("OnDataChannelAdded");
            _mainThreadWorkQueue.Enqueue(() =>
            {
                _dataChannel = channel;
                _dataChannel.MessageReceived += OnKinectMessageReceived;
                Debug.Log("OnDataChannelAdded Main Thread");
            });
        }

        private void InitPointCloud(int num)
        {
            numPoints = num;

            mesh = new Mesh();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;

            //Allocation of vertex and color storage space for the total number of pixels in the depth image
            vertices = new Vector3[num];
            colors = new Color32[num];
            indices = new int[num];

            //Initialization of index list
            for (int i = 0; i < num; i++)
            {
                indices[i] = i;
            }

            //Allocate a list of point coordinates, colors, and points to be drawn to mesh
            mesh.vertices = vertices;
            mesh.colors32 = colors;
            mesh.SetIndices(indices, MeshTopology.Points, 0);

            gameObject.GetComponent<MeshFilter>().mesh = mesh;

            pointCloudInitialized = true;
        }

        private void OnKinectMessageReceived(byte[] data)
        {
            Debug.Log("Received data of length " + data.Length);

            _mainThreadWorkQueue.Enqueue(() => {
                if (data.Length == WEBRTC_MESSAGE_SIZE)
                {
                    if (unfragDataIndex == 0 && numPoints <= 0)
                    {
                        numPoints = BitConverter.ToInt32(data, 0);
                    }
                    
                    if (unfragDataIndex == 0)
                    {
                        Debug.LogError(BitConverter.ToInt32(data, 0));
                    }

                    Buffer.BlockCopy(data, 0, unfragData, unfragDataIndex * WEBRTC_MESSAGE_SIZE, WEBRTC_MESSAGE_SIZE);
                    unfragDataIndex++;

                    return;
                }

                if (data.Length != 1)
                {
                    return;
                }

                if (!pointCloudInitialized)
                {
                    Debug.LogError(numPoints);
                    InitPointCloud(numPoints);
                }

                int index = 4;

                try {
                    for (int i = 0; i < numPoints; i++)
                    {
                        vertices[i].x = BitConverter.ToSingle(unfragData, index);
                        index += NUM_BYTES_PER_FLOAT;
                        vertices[i].y = BitConverter.ToSingle(unfragData, index);
                        index += NUM_BYTES_PER_FLOAT;
                        vertices[i].z = BitConverter.ToSingle(unfragData, index);
                        index += NUM_BYTES_PER_FLOAT;
                    }

                    for (int i = 0; i < numPoints; i++)
                    {
                        colors[i].b = unfragData[index];
                        index++;
                        colors[i].g = unfragData[index];
                        index++;
                        colors[i].r = unfragData[index];
                        index++;
                        colors[i].a = unfragData[index];
                        index++;
                    }
                }
                catch (IndexOutOfRangeException e)
                {
                    Debug.LogError(e);
                    Debug.LogError("Index: " + index + ", numPoints: " + numPoints);
                }

                mesh.vertices = vertices;
                mesh.colors32 = colors;
                mesh.RecalculateBounds();

                unfragDataIndex = 0;
            });
        }

        private void Update()
        {
            // Execute any pending work enqueued by background tasks
            while (_mainThreadWorkQueue.TryDequeue(out Action workload))
            {
                workload();
            }
        }
    }
}