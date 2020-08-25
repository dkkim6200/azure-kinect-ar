using Microsoft.Azure.Kinect.Sensor;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.MixedReality.WebRTC.Unity;
using System;

namespace DKDevelopment.AzureKinect.Client
{
    public class PointCloudClient : MonoBehaviour
    {
        public PeerConnection _peerConnection;
        private Microsoft.MixedReality.WebRTC.DataChannel _dataChannel;

        // private static readonly int NUM_FLOATS_PER_VECTOR = 3;
        // private static readonly int NUM_BYTES_PER_COLOR = 4;
        private static readonly int NUM_BYTES_PER_FLOAT = 4;

        private int numPoints;
        //Used to draw a set of points
        private Mesh mesh;
        //Array of coordinates for each point in PointCloud
        private Vector3[] vertices;
        //Array of colors corresponding to each point in PointCloud
        private Color32[] colors;
        //List of indexes of points to be rendered
        private int[] indices;

        private bool pointCloudInitialized;

        private void Start()
        {
        }

        public void StartPointCloud()
        {
            Task t = StartPointCloudAsync();
        }

        private async Task StartPointCloudAsync()
        {
            _dataChannel = await _peerConnection.Peer.AddDataChannelAsync(42, "PointCloud", false, false);
            _dataChannel.MessageReceived += OnKinectMessageReceived;
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
            if (!pointCloudInitialized)
            {
                InitPointCloud(BitConverter.ToInt32(data, 0));
            }

            int index = 4;

            for (int i = 0; i < numPoints; i++)
            {
                vertices[i].x = BitConverter.ToSingle(data, index);
                index += NUM_BYTES_PER_FLOAT;
                vertices[i].y = BitConverter.ToSingle(data, index);
                index += NUM_BYTES_PER_FLOAT;
                vertices[i].z = BitConverter.ToSingle(data, index);
                index += NUM_BYTES_PER_FLOAT;
            }

            for (int i = 0; i < numPoints; i++)
            {
                colors[i].b = data[index];
                index++;
                colors[i].g = data[index];
                index++;
                colors[i].r = data[index];
                index++;
                colors[i].a = data[index];
                index++;
            }

            mesh.vertices = vertices;
            mesh.colors32 = colors;
            mesh.RecalculateBounds();
        }
        
        private void OnDestroy()
        {
            
        }
    }
}