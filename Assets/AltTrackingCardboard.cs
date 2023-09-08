using UnityEngine;
using Antilatency.SDK;
using Antilatency.DeviceNetwork;
using Antilatency.Alt.Tracking;

public class AltTrackingCardboard : AltTracking
{
    public UnityEngine.SpatialTracking.TrackedPoseDriver HmdPoseDriver;

    private bool _lerpPosition;
    private bool _lerpRotation;

    private Antilatency.TrackingAlignment.ILibrary _alignmentLibrary;
    private Antilatency.TrackingAlignment.ITrackingAlignment _alignment;

    private bool _altInitialPositionApplied = false;
    private const float _bQuality = 0.15f;

    private Transform _aSpace;
    private Transform _bSpace;
    private Transform _b;

    protected override NodeHandle GetAvailableTrackingNode()
    {
        return GetUsbConnectedFirstIdleTrackerNode();
    }

    protected override Pose GetPlacement()
    {
        var result = Pose.identity;

        using (var localStorage = StorageClient.GetLocalStorage())
        {
            if (localStorage == null)
            {
                return result;
            }

            var placementCode = localStorage.read("placement", "default");

            if (string.IsNullOrEmpty(placementCode))
            {
                Debug.LogError("Failed to get placement code");
                result = Pose.identity;
            }
            else
            {
                result = _trackingLibrary.createPlacement(placementCode);
            }

            return result;
        }
    }

    protected virtual void OnFocusChanged(bool focus)
    {
        if (focus)
        {
            StartTrackingAlignment();
        }
        else
        {
            StopTrackingAlignment();
        }
    }

    private void StartTrackingAlignment()
    {
        if (_alignment != null)
        {
            StopTrackingAlignment();
        }

        var placement = GetPlacement();
        _alignment = _alignmentLibrary.createTrackingAlignment(placement.rotation, ExtrapolationTime);

        _altInitialPositionApplied = false;
    }

    private void StopTrackingAlignment()
    {
        if (_alignment == null)
        {
            return;
        }

        _alignment.Dispose();
        _alignment = null;
    }

    private void OnApplicationFocus(bool focus)
    {
        OnFocusChanged(focus);
    }

    private void OnApplicationPause(bool pause)
    {
        OnFocusChanged(!pause);
    }

    protected override void Awake()
    {
        base.Awake();

        _alignmentLibrary = Antilatency.TrackingAlignment.Library.load();

        var placement = GetPlacement();
        _alignment = _alignmentLibrary.createTrackingAlignment(placement.rotation, ExtrapolationTime);

        _lerpPosition = HmdPoseDriver.trackingType == UnityEngine.SpatialTracking.TrackedPoseDriver.TrackingType.PositionOnly ||
                        HmdPoseDriver.trackingType == UnityEngine.SpatialTracking.TrackedPoseDriver.TrackingType.RotationAndPosition;

        _lerpRotation = HmdPoseDriver.trackingType == UnityEngine.SpatialTracking.TrackedPoseDriver.TrackingType.RotationOnly ||
                        HmdPoseDriver.trackingType == UnityEngine.SpatialTracking.TrackedPoseDriver.TrackingType.RotationAndPosition;
        
        _b = HmdPoseDriver.transform;
        _bSpace = _b.parent;
        _aSpace = _bSpace.parent;
    }

    protected override void Update()
    {
        base.Update();
        if (!TrackingTaskState)
        {
            return;
        }

        Vector3 bPosition = _b.localPosition;
        Quaternion bRotation = _b.localRotation;

        if (!GetRawTrackingState(out State rawTrackingState))
        {
            return;
        }

        if (_lerpRotation && _alignment != null && 
            rawTrackingState.stability.stage == Stage.Tracking6Dof)
        {
            var result = _alignment.update(
                rawTrackingState.pose.rotation,
                bRotation,
                Time.realtimeSinceStartup);

            ExtrapolationTime = result.timeBAheadOfA;
            _placement.rotation = result.rotationARelativeToB;
            _bSpace.localRotation = result.rotationBSpace;
        }

        if (!GetTrackingState(out State trackingState))
        {
            return;
        }

        if (!_lerpRotation)
        {
            _bSpace.localRotation = trackingState.pose.rotation;
            _b.localRotation = Quaternion.identity;
        }

        if (_lerpPosition)
        {
            if (trackingState.stability.stage == Stage.Tracking6Dof)
            {
                Vector3 a = trackingState.pose.position;
                Vector3 b = _bSpace.TransformPoint(bPosition);
                if (_aSpace != null)
                {
                    b = _aSpace.InverseTransformPoint(b);
                }

                Vector3 averagePositionInASpace;
                if (!_altInitialPositionApplied)
                {
                    averagePositionInASpace = a;
                    _altInitialPositionApplied = true;
                }
                else
                {
                    averagePositionInASpace = (b * _bQuality + a * trackingState.stability.value) / (trackingState.stability.value + _bQuality);
                }

                _bSpace.localPosition += averagePositionInASpace - b;
            }
        }
        else
        {
            _bSpace.localPosition = trackingState.pose.position;
            _b.localPosition = Vector3.zero;
        }
    }
}
