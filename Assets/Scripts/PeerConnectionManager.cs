using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.WebRTC.Unity;

public class PeerConnectionManager : MonoBehaviour
{
    public void StartConnection()
    {
        GetComponent<PeerConnection>().StartConnection();
    }
}
