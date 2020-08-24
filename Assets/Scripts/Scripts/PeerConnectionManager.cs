using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.WebRTC.Unity;
using Microsoft.MixedReality.WebRTC;

public class PeerConnectionManager : MonoBehaviour
{
    private Microsoft.MixedReality.WebRTC.Unity.PeerConnection _peerConnection;

    void Awake()
    {
        _peerConnection = GetComponent<Microsoft.MixedReality.WebRTC.Unity.PeerConnection>();
    }

    // Update is called once per frame
    void Update()
    {
        byte[] testArray = { 1, 2, 3, 4 };
        _peerConnection.Peer.DataChannels[0].SendMessage(testArray);
    }
}
