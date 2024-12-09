using Android.Hardware.Camera2;
using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Camera.MAUI.Platforms.Android;

internal class CaptureCompletedArgs
{
    public CameraCaptureSession Session { get; set; }
    public CaptureRequest Request { get; set; }
    public TotalCaptureResult Result { get; set; }
}

internal class CaptureFailedArgs
{
    public CameraCaptureSession Session { get; set; }
    public CaptureRequest Request { get; set; }
    public CaptureFailure Failure { get; set; }
}


internal class GenericCaptureCallback : CameraCaptureSession.CaptureCallback
{
    readonly MauiCameraView _mauiCameraView;
    readonly ILogger _logger;

    public GenericCaptureCallback(MauiCameraView mauiCameraView, ILogger logger)
    {
        _mauiCameraView = mauiCameraView;
        _logger = logger;
    }

    public event EventHandler<CaptureCompletedArgs> CaptureCompleted;
    public override void OnCaptureCompleted(CameraCaptureSession session, CaptureRequest request, TotalCaptureResult result)
    {
        try
        {
            base.OnCaptureCompleted(session, request, result);
            CaptureCompleted?.Invoke(this, new CaptureCompletedArgs { Session = session, Request = request, Result = result });
        }
        catch (Exception ex)
        {
            _logger.LogWarning($"{nameof(OnCaptureCompleted)}: failed: {ex.Message}");
        }

    }
    public event EventHandler<CaptureFailedArgs> CaptureFailed;
    public override void OnCaptureFailed(CameraCaptureSession session, CaptureRequest request, CaptureFailure failure)
    {
        try
        {
            base.OnCaptureFailed(session, request, failure);
            var failureReason = (string)failure.Reason.ToString();
            _logger.LogWarning($"{nameof(OnCaptureFailed)}: {failureReason}");
            CaptureFailed?.Invoke(this,  new CaptureFailedArgs { Session = session, Request = request, Failure = failure });
        }
        catch (Exception ex)
        {
            _logger.LogWarning($"{nameof(OnCaptureFailed)}: failed: {ex.Message}");
        }
    }
}
