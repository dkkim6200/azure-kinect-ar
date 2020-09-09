# azure-kinect-ar

What if you can be teleported right into anywhere in the world, in real time, in 3D? Azure Kinect AR is a project that does just that. Simply connect the Azure Kinect DK to your computer, run the server, and you can enter from your HoloLens 2 to bring yourself among the real time 3D scan of a room.


## Tech Stacks
- Mixed Reality Toolkit (MRTK) is used for rapid prototyping.
- WebRTC is used to stream the RGB-D video.
- Some shader magic goes in to achieve a fast, GPU-powered conversion of RGB-D video to point clouds.
